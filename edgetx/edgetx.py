#!/usr/bin/env python3
"""
EdgeTX → RC_CHANNELS_OVERRIDE bridge + WiFi-health G-VAR
=======================================================

• **Base score** – strongest-antenna *avg RSSI* mapped linearly  
  –100 dBm … –30 dBm → **1000 … 2000** (RC-PWM units).  
• **Penalties** (max 750 pts, final link clamped 1000-2000) are *subtracted*:

    1. **Lost packets**   (largest weight)  
    2. **FEC-recovered**  (medium)  
    3. **Packet diversity** ratio (total/unique) (smallest)

• `--disable-penalty` ignores deductions and sends raw RSSI.

Everything prints with `flush=True`, the serial link auto-reconnects every
3 s, and the Wi-Fi reader itself now also **auto-reconnects** forever.
"""

from __future__ import annotations
import argparse, signal, subprocess, sys, threading, time, re
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import serial  # type: ignore
except ModuleNotFoundError:
    sys.stderr.write("pip install pyserial\n")
    sys.exit(1)

# ─────────────────────────── basic constants ────────────────────────────
MIN_EDGETX, MAX_EDGETX = -1024, 1024
PWM_MIN, PWM_MAX       = 1000, 2000
PWM_RANGE              = PWM_MAX - PWM_MIN

RSSI_MIN_DBM, RSSI_MAX_DBM = -100, -30
RSSI_RANGE                 = RSSI_MAX_DBM - RSSI_MIN_DBM   # 70 dB

MAX_PENALTY     = 750
SERIAL_RETRY_SEC = 3
WFBRX_RETRY_SEC  = 3

SCRIPT = Path("/usr/bin/channels.sh")

r_pkt   = re.compile(r"\bPKT\b")
r_rxant = re.compile(r"\bRX_ANT\b")

# ─────────────────────────── helper funcs ───────────────────────────────
def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))

def rssi_to_pwm(rssi: int) -> int:
    """–100 … –30 dBm → 1000 … 2000 (int)."""
    frac = (_clamp(rssi, RSSI_MIN_DBM, RSSI_MAX_DBM) - RSSI_MIN_DBM) / RSSI_RANGE
    return int(round(PWM_MIN + frac * PWM_RANGE))

def pwm_to_gvar(pwm: int) -> int:
    """1000 … 2000 µs → –1024 … +1024."""
    pwm = _clamp(pwm, PWM_MIN, PWM_MAX)
    return int(round((pwm - 1500) * 1024 / 500))

signal.signal(signal.SIGPIPE, signal.SIG_DFL)

# ───────────────────────── Wi-Fi reader thread ──────────────────────────
class WfbReader(threading.Thread):
    """Keeps reading `wfb_rx` forever; updates GV when each PKT chunk closes."""

    def __init__(self, sniffer: 'EdgeTXSniffer', *, disable_penalty: bool) -> None:
        super().__init__(daemon=True)
        self.sniffer = sniffer
        self.disable_penalty = disable_penalty

    # ------------ penalty calculator ------------
    def _penalty(self, lost: int, fec: int, ratio: float) -> tuple[int, int, int, int]:
        # lost packets
        p_lost = (lost * 200) if lost <= 3 else 600 + (lost - 3) * 50
        p_lost = _clamp(p_lost, 0, 500)
        # FEC
        if fec <= 10:
            p_fec = fec * 5
        elif fec <= 25:
            p_fec = 50 + (fec - 10) * 10
        else:
            p_fec = 200 + (fec - 25) * 15
        p_fec = _clamp(p_fec, 0, 200)
        # diversity
        if ratio < 1.5:
            p_div = int(round((1.5 - ratio) / 0.5 * 100))
        elif ratio < 1.8:
            p_div = int(round((1.8 - ratio) / 0.3 * 50))
        elif ratio < 2.8:
            p_div = int(round((2.8 - ratio) / 1.0 * 20))
        else:
            p_div = 0
        p_div = max(0, p_div)

        p_total = _clamp(p_lost + p_fec + p_div, 0, MAX_PENALTY)
        return p_total, p_lost, p_fec, p_div

    # -------------- main loop (auto-reconnect) --------------
    def run(self) -> None:
        while True:  # outer forever loop – reconnect on any failure/EOF
            try:
                proc = subprocess.Popen(
                    ["nc", "127.0.0.1", "9500"],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    text=True,
                )
                assert proc.stdin
                proc.stdin.write("wfb_rx\n")
                proc.stdin.flush()

                best_rssi: Optional[int] = None

                for line in proc.stdout:   # type: ignore[arg-type]
                    # -------- RX_ANT lines --------
                    if r_rxant.search(line):
                        try:
                            avg = int(line.split()[4].split(":")[2])
                            if (best_rssi is None) or (avg > best_rssi):
                                best_rssi = avg
                        except (IndexError, ValueError):
                            continue

                    # -------- PKT line closes a chunk --------
                    elif r_pkt.search(line):
                        nums_token = next(
                            (tok for tok in line.split()
                             if tok[0].isdigit() and ':' in tok), ""
                        )
                        try:
                            nums = [int(x) for x in nums_token.split(':')]
                            if len(nums) < 8:
                                continue
                            total_p  = nums[0]
                            unique_p = nums[5]
                            fec_cnt  = nums[6]
                            lost_cnt = nums[7]
                            ratio    = total_p / unique_p if unique_p else 0.0
                        except ValueError:
                            continue

                        if best_rssi is not None:
                            pwm_base = rssi_to_pwm(best_rssi)

                            if self.disable_penalty:
                                pwm_final = pwm_base
                                p_total = p_lost = p_fec = p_div = 0
                            else:
                                p_total, p_lost, p_fec, p_div = \
                                    self._penalty(lost_cnt, fec_cnt, ratio)
                                pwm_final = _clamp(pwm_base - p_total, PWM_MIN, PWM_MAX)

                            gv_val = _clamp(pwm_to_gvar(pwm_final),
                                            MIN_EDGETX, MAX_EDGETX)

                            # Debug
                            print(
                                f"[DBG] RSSI={best_rssi:<4}  base={pwm_base}  "
                                f"lost={lost_cnt}({p_lost})  "
                                f"fec={fec_cnt}({p_fec})  "
                                f"div={ratio:.2f}({p_div})  "
                                f"total_pen={p_total}  link={pwm_final}  gv={gv_val}",
                                flush=True,
                            )

                            self.sniffer.send_gv(gv_val)

                        best_rssi = None  # reset for next chunk
            except Exception as exc:       # pylint: disable=broad-except
                print(f"[WARN] WiFi reader error: {exc}", flush=True)

            # clean-up / retry
            try:
                if proc and proc.poll() is None:
                    proc.kill()
            except Exception:
                pass
            print(f"[WARN] WiFi stream lost – reconnecting in {WFBRX_RETRY_SEC}s",
                  flush=True)
            time.sleep(WFBRX_RETRY_SEC)

# ───────────────────────────── main sniffer ─────────────────────────────
class EdgeTXSniffer:                                  # pylint: disable=too-many-instance-attributes
    _POLL_SLEEP = 0.01

    def __init__(self, *, port: str, baud: int, period_ms: int,
                 command_parse: bool, min_change: int, persist_ms: int, pause_ms: int,
                 channels_filter: Optional[set[int]], disable_penalty: bool) -> None:
        self.port, self.baud = port, baud
        self.ser: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self._connect_serial()

        self.period_ms  = max(1, period_ms)
        self.period_sec = self.period_ms / 1000
        self._start     = time.time()
        self._next      = self._start + self.period_sec

        self.latest_pwm: List[int] = [1500] * 16
        self.rc_cnt = 0

        # command-parse
        self.cmd_parse, self.min_change, self.persist_ms, self.pause_ms = \
            command_parse, min_change, persist_ms, pause_ms
        self.allowed = channels_filter
        self.pending: List[Optional[Dict[str, Any]]] = [None] * 16
        self.last   : List[Optional[int]]            = [None] * 16
        self.next_allowed_ms = 0
        self.exec_idx = 0
        self.idx_lock = threading.Lock()

        print(f"[INFO] {self.port}@{self.baud}  agg={self.period_ms}ms  "
              f"cmd_parse={self.cmd_parse}", flush=True)

        # spawn Wi-Fi health reader
        WfbReader(self, disable_penalty=disable_penalty).start()

    # -------------- serial connect / reconnect --------------
    def _connect_serial(self) -> None:
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                print("[INFO] Serial connected", flush=True)
                return
            except serial.SerialException as exc:
                print(f"[WARN] Serial open failed: {exc} – retrying in "
                      f"{SERIAL_RETRY_SEC}s", flush=True)
                time.sleep(SERIAL_RETRY_SEC)

    def _reset_serial(self) -> None:
        with self.serial_lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
        print("[WARN] Serial lost – reconnecting …", flush=True)
        self._connect_serial()

    # ---------------- G-VAR sender ----------------
    def send_gv(self, val: int) -> None:
        with self.serial_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(f"GV,0,{val}\n".encode())
            except serial.SerialException:
                self._reset_serial()
                return
        print(f"[>>] GV,0,{val}", flush=True)

    # ---------------- main loop ------------------
    def run(self) -> None:  # pylint: disable=too-many-branches
        try:
            while True:
                now = time.time()

                line = ""
                if self.ser:
                    try:
                        line = self.ser.readline().decode(errors="replace").strip()
                    except serial.SerialException:
                        self._reset_serial()

                if line.startswith("CH,"):
                    self._handle_ch(line)

                if now >= self._next:
                    if self.rc_cnt:
                        self._print_override()
                        self.rc_cnt = 0
                    self._next += self.period_sec

                if not line:
                    time.sleep(self._POLL_SLEEP)
        except KeyboardInterrupt:
            print("\n[INFO] Terminated", flush=True)

    # ---------- packet helpers ----------
    def _handle_ch(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) != 17:
            return
        try:
            raws = [int(x) for x in parts[1:]]
        except ValueError:
            return
        self.latest_pwm = [_clamp(x, MIN_EDGETX, MAX_EDGETX) for x in raws]
        self.rc_cnt += 1
        if self.cmd_parse:
            self._eval_channels(self.latest_pwm)

    def _print_override(self) -> None:
        ts = self._now_ms()
        print(f"{ts} RC_CHANNELS_OVERRIDE {self.rc_cnt} 0 0 "
              f"{':'.join(map(str, self.latest_pwm))}", flush=True)

    # ---------- command-parse ----------
    def _eval_channels(self, pwm: List[int]) -> None:
        now = self._now_ms()
        for idx, val in enumerate(pwm):
            ch = idx + 1
            if self.allowed and ch not in self.allowed:
                continue
            prev = self.last[idx]
            if prev is None:
                self.last[idx] = val
                continue

            diff = abs(val - prev)
            cand = self.pending[idx]

            if cand:
                if diff < self.min_change:
                    self.pending[idx] = None
                    continue
                if now - cand["start"] >= self.persist_ms:
                    if self._try_exec(ch, prev, val, now):
                        self.last[idx] = val
                        self.pending[idx] = None
                continue

            if diff >= self.min_change:
                self.pending[idx] = {"start": now}

    def _try_exec(self, ch: int, old: int, new: int, now_ms: int) -> bool:
        if now_ms < self.next_allowed_ms:
            return False
        if not SCRIPT.exists():
            sys.stderr.write(f"{SCRIPT} missing\n")
            return False

        with self.idx_lock:
            self.exec_idx += 1
            idx = self.exec_idx

        def work() -> None:
            start = time.time()
            try:
                res = subprocess.run([str(SCRIPT), str(ch), str(new)],
                                     capture_output=True)
                rt = int(round((time.time() - start) * 1000))
                if res.returncode != 0:
                    sys.stderr.write(
                        f"channels.sh err ch{ch}: "
                        f"{res.stderr.decode(errors='ignore')}\n"
                    )
            except Exception as exc:  # pylint: disable=broad-except
                sys.stderr.write(f"exec fail {exc}\n")
                return
            ts = int((time.time() - self._start) * 1000)
            print(f"{ts} MAVLINK_EXEC {idx}:{ch}:{old}:{new}:{rt}", flush=True)

        threading.Thread(target=work, daemon=True).start()
        self.next_allowed_ms = now_ms + self.pause_ms
        return True

    def _now_ms(self) -> int:
        return int((time.time() - self._start) * 1000)

# ───────────────────────────── CLI plumbing ─────────────────────────────
def _parse_filter(txt: Optional[str]) -> Optional[set[int]]:
    if not txt:
        return None
    try:
        chans = {int(x) for x in txt.split(",") if x.strip()}
        if any(not 1 <= c <= 16 for c in chans):
            raise ValueError
        return chans
    except ValueError:
        sys.stderr.write("--channels expects 1-16\n")
        sys.exit(1)

def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser("EdgeTX RC bridge + Wi-Fi G-VAR health")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--period", type=int, default=1000, metavar="MS")
    # command-parse
    p.add_argument("--command-parse", action="store_true")
    p.add_argument("--min-change", type=int, default=100)
    p.add_argument("--persist", type=int, default=500, metavar="MS")
    p.add_argument("--pause", type=int, default=500, metavar="MS")
    p.add_argument("--channels")
    # penalties
    p.add_argument("--disable-penalty", action="store_true",
                   help="use raw RSSI (ignore FEC/lost/diversity penalties)")
    return p.parse_args()

def main() -> None:
    a = _args()
    EdgeTXSniffer(
        port=a.port,
        baud=a.baud,
        period_ms=a.period,
        command_parse=a.command_parse,
        min_change=a.min_change,
        persist_ms=a.persist,
        pause_ms=a.pause,
        channels_filter=_parse_filter(a.channels),
        disable_penalty=a.disable_penalty,
    ).run()

if __name__ == "__main__":
    main()
