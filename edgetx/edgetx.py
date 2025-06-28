#!/usr/bin/env python3
"""
EdgeTX → RC_CHANNELS_OVERRIDE bridge (non-blocking)

• Reads CH,<16> lines from EdgeTX, maps –1024…+1024 → 1000…2000 µs.
• Emits RC_CHANNELS_OVERRIDE once per --period **only if** at least one frame
  arrived in that window.
• Optional --command-parse launches /usr/bin/channels.sh in a background
  thread – so no main-loop stalls – and prints MAVLINK_EXEC when done.
• Still bumps GV 0 every second by --gvar-step (set 0 to disable).
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import serial  # type: ignore
except ModuleNotFoundError:
    sys.stderr.write("Missing pyserial – install with:\n  pip install pyserial\n")
    sys.exit(1)

# ─────────────────────────── constants & helpers ────────────────────────────
MIN_EDGETX, MAX_EDGETX = -1024, 1024
MIN_PWM, MAX_PWM = 1000, 2000
_EDGETX_RANGE, _PWM_RANGE = 2048, 1000
SCRIPT = Path("/usr/bin/channels.sh")


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def edgetx_to_pwm(v: int) -> int:
    """Linear map –1024…+1024 → 1000…2000 (rounded int)."""
    return int(round((_clamp(v, MIN_EDGETX, MAX_EDGETX) - MIN_EDGETX)
                     * _PWM_RANGE / _EDGETX_RANGE + MIN_PWM))


signal.signal(signal.SIGPIPE, signal.SIG_DFL)  # respect downstream pipe


# ──────────────────────────────── core class ────────────────────────────────
class EdgeTXSniffer:  # pylint: disable=too-many-instance-attributes
    _POLL_SLEEP = 0.01  # 10 ms CPU nap when idle

    def __init__(  # pylint: disable=too-many-arguments
        self,
        *,
        port: str,
        baud: int,
        period_ms: int,
        gvar_step: int,
        command_parse: bool,
        min_change: int,
        persist_ms: int,
        pause_ms: int,
        channels_filter: Optional[set[int]],
    ) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # periodic reporting
        self.period_ms = max(1, period_ms)
        self.period_sec = self.period_ms / 1000.0
        self._start_ts = time.time()
        self._next_rep = self._start_ts + self.period_sec

        # most-recent PWM frame
        self.latest_pwm: List[int] = [1500] * 16
        self.rc_cnt = 0

        # GV demo
        self.g_step = gvar_step
        self._g_min, self._g_max = MIN_EDGETX, MAX_EDGETX
        self._g_current = self._g_min
        self._next_gv = time.time() + 1

        # command-parse state
        self.cmd_parse = command_parse
        self.min_change = min_change
        self.persist_ms = persist_ms
        self.pause_ms = pause_ms
        self.allowed = channels_filter
        self.pending: List[Optional[Dict[str, Any]]] = [None] * 16
        self.last: List[Optional[int]] = [None] * 16
        self.next_allowed_ms = 0
        self.exec_idx = 0
        self.idx_lock = threading.Lock()

        print(
            f"[INFO] Serial {self.ser.port}@{self.ser.baudrate} "
            f"agg={self.period_ms}ms  cmd_parse={self.cmd_parse}",
            flush=True,
        )

    # ─────────────────────────── main loop ────────────────────────────
    def run(self) -> None:  # pylint: disable=too-many-branches
        try:
            while True:
                now = time.time()

                # 1) serial read (non-blocking)
                line = self.ser.readline().decode(errors="replace").strip()
                if line.startswith("CH,"):
                    self._handle_ch(line)

                # 2) periodic RC_CHANNELS_OVERRIDE
                if now >= self._next_rep:
                    if self.rc_cnt:  # only if data arrived
                        self._print_override()
                        self.rc_cnt = 0
                    self._next_rep += self.period_sec

                # 3) GV bump demo
                if self.g_step and now >= self._next_gv:
                    self._send_gv()
                    self._next_gv = now + 1

                if not line:
                    time.sleep(self._POLL_SLEEP)
        except KeyboardInterrupt:
            print("\n[INFO] Terminated")

    # ───────────────────── line / data handling ─────────────────────
    def _handle_ch(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) != 17:
            return
        try:
            raws = [int(x) for x in parts[1:]]
        except ValueError:
            return
        self.latest_pwm = [edgetx_to_pwm(v) for v in raws]
        self.rc_cnt += 1
        if self.cmd_parse:
            self._eval_channels(self.latest_pwm)

    # ───────────────────────── GV helper ────────────────────────────
    def _send_gv(self) -> None:
        cmd = f"GV,0,{self._g_current}\n"
        try:
            self.ser.write(cmd.encode())
            print(f"[>>] {cmd.strip()}")
        except serial.SerialException as exc:
            print(f"[WARN] write error: {exc}")

        if self.g_step:
            self._g_current += self.g_step
            if self._g_current > self._g_max:
                self._g_current = self._g_min

    # ───────────────────── periodic override ───────────────────────
    def _print_override(self) -> None:
        ts = self._now_ms()
        chan_str = ":".join(map(str, self.latest_pwm))
        print(f"{ts} RC_CHANNELS_OVERRIDE {self.rc_cnt} 0 0 {chan_str}", flush=True)

    # ───────────────────── command-parse logic ─────────────────────
    def _eval_channels(self, pwm: List[int]) -> None:
        now_ms = self._now_ms()
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

            # candidate already in progress?
            if cand is not None:
                if diff < self.min_change:  # cancelled
                    self.pending[idx] = None
                    continue
                if now_ms - cand["start"] >= self.persist_ms:
                    if self._try_exec(ch, prev, val, now_ms):
                        self.last[idx] = val
                        self.pending[idx] = None
                continue

            # start new candidate?
            if diff >= self.min_change:
                self.pending[idx] = {"start": now_ms}

    # ───────────────────── non-blocking exec ──────────────────────
    def _try_exec(self, ch: int, old: int, new: int, now_ms: int) -> bool:
        if now_ms < self.next_allowed_ms:
            return False
        if not SCRIPT.exists():
            sys.stderr.write(f"[EdgeTX] {SCRIPT} missing\n")
            return False

        with self.idx_lock:
            self.exec_idx += 1
            idx = self.exec_idx

        def _worker() -> None:
            start = time.time()
            try:
                res = subprocess.run(
                    [str(SCRIPT), str(ch), str(new)],
                    capture_output=True,
                )
                runtime_ms = int(round((time.time() - start) * 1000))
                if res.returncode != 0:
                    sys.stderr.write(
                        f"[EdgeTX] channels.sh error (ch {ch}): "
                        f"{res.stderr.decode(errors='ignore')}\n"
                    )
            except Exception as exc:  # pylint: disable=broad-except
                sys.stderr.write(f"[EdgeTX] Exec failed (ch {ch}): {exc}\n")
                return

            ts = int((time.time() - self._start_ts) * 1000)
            print(f"{ts} MAVLINK_EXEC {idx}:{ch}:{old}:{new}:{runtime_ms}", flush=True)

        threading.Thread(target=_worker, daemon=True).start()
        self.next_allowed_ms = now_ms + self.pause_ms
        return True

    # ───────────────────────── misc ─────────────────────────
    def _now_ms(self) -> int:
        return int((time.time() - self._start_ts) * 1000)


# ─────────────────────────────── CLI ───────────────────────────────
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


def _args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser("EdgeTX serial → RC_CHANNELS_OVERRIDE bridge")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--period", type=int, default=1000, metavar="MS")
    p.add_argument("--gvar-step", type=int, default=1)
    p.add_argument("--command-parse", action="store_true")
    p.add_argument("--min-change", type=int, default=100)
    p.add_argument("--persist", type=int, default=500, metavar="MS")
    p.add_argument("--pause", type=int, default=500, metavar="MS")
    p.add_argument("--channels")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> None:
    a = _args(argv)
    EdgeTXSniffer(
        port=a.port,
        baud=a.baud,
        period_ms=a.period,
        gvar_step=a.gvar_step,
        command_parse=a.command_parse,
        min_change=a.min_change,
        persist_ms=a.persist,
        pause_ms=a.pause,
        channels_filter=_parse_filter(a.channels),
    ).run()


if __name__ == "__main__":
    main()
