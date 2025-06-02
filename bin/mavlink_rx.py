#!/usr/bin/env python3
"""
MAVLink sniffer with RC-triggered commands
=========================================

v1.3 – adds **peek / gap sampling** (periodic “listen-windows”) while keeping every
other feature intact.

* ``--peek-window`` N   → process incoming bytes for *N ms* each cycle
* ``--peek-gap``   M   → discard / sleep for *M ms* between windows  
  (default: same as *peek-window*).  
  Set ``--peek-window 0`` (default) to restore the original continuous mode.

The rest of the v1.2 perf tweaks are still here:

* ``use_native=True`` when available.
* 10 ms nap when the RX queue is empty.

Typical usage
-------------

::

   mavlink_rx.py --port /dev/ttyS3 --baud 460800 \
       --period 1000 --robust \
       --peek-window 50 --peek-gap 50 \
       --command-parse --channels 5,6 --min-change 100 \
       --persist 500 --pause 500
"""

from __future__ import annotations

import argparse
import inspect
import signal
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# --------------------------------------------------------------------- deps
try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    sys.stderr.write("Missing pymavlink – install with:\n  pip install pymavlink pyserial\n")
    sys.exit(1)

# Honour SIGPIPE (otherwise BrokenPipeError spam when downstream consumer quits)
signal.signal(signal.SIGPIPE, signal.SIG_DFL)


# ---------------------------------------------------------------------------
class MavlinkSniffer:
    CHANNEL_MSG = "RC_CHANNELS_OVERRIDE"
    RADIO_MSG = "RADIO_STATUS"
    SCRIPT = Path("/usr/bin/channels.sh")
    _POLL_SLEEP = 0.01  # 10 ms pause when no message was decoded

    # .....................................................................
    def __init__(  # pylint: disable=too-many-locals,too-many-arguments
        self,
        *,
        port: str = "/dev/ttyUSB0",
        baud: int = 460_800,
        period_ms: int = 1000,
        raw: bool = False,
        dialect: str = "common",
        robust: bool = False,
        # command-mode
        command_parse: bool = False,
        min_change: int = 100,
        persist_ms: int = 500,
        pause_ms: int = 500,
        channels_filter: Optional[set[int]] = None,
        # peek / gap sampling
        peek_window_ms: int = 0,
        peek_gap_ms: Optional[int] = None,
    ) -> None:

        # ---------- open serial (native/C parser & robust only if supported)
        conn_kwargs = dict(
            baud=baud,
            dialect=dialect,
            autoreconnect=True,
            bytesize=8,
            parity="N",
            stopbits=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        if "use_native" in inspect.signature(mavutil.mavlink_connection).parameters:
            conn_kwargs["use_native"] = True
        if robust and "robust_parsing" in inspect.signature(mavutil.mavlink_connection).parameters:
            conn_kwargs["robust_parsing"] = True

        self.master = mavutil.mavlink_connection(port, **conn_kwargs)

        # ---------- capability checks
        self._recv_supports_exc = "exception_on_bad_data" in inspect.signature(
            self.master.recv_match
        ).parameters
        self._has_crc_fail = hasattr(self.master.mav, "total_crc_fail")

        # ---------- timing
        self.period_ms = max(1, period_ms)
        self.period_sec = self.period_ms / 1000.0
        self._start_ts = time.time()
        self._next_report_ts = self._start_ts + self.period_sec

        # ---------- command-execution parameters
        self.cmd_parse = command_parse
        self.min_change = min_change
        self.persist_ms = persist_ms
        self.pause_ms = pause_ms
        self.allowed_channels = channels_filter  # None ⇒ all

        # ---------- peek / gap configuration
        self.peek_window_ms = max(0, int(peek_window_ms))
        self.peek_gap_ms = (
            int(peek_gap_ms) if peek_gap_ms is not None else self.peek_window_ms
        )

        # ---------- runtime state
        self.raw = raw
        self.total_bytes = 0
        self.parse_errors = 0
        self.next_allowed_exec_ms = 0
        # per-channel state (create *before* first reset_counters call)
        self.pending: List[Optional[Dict[str, Any]]] = [None] * 16
        self.last_values: List[Optional[int]] = [None] * 16
        self.exec_events: List[Tuple[int, int, int, int]] = []  # (ch, old, new, runtime)

        self._reset_counters()

        print(
            f"Listening on {self.master.port} @{self.master.baud}  "
            f"| agg={self.period_ms} ms  raw={self.raw}  robust={robust}  "
            f"peek={self.peek_window_ms}/{self.peek_gap_ms} ms",
            flush=True,
        )

    # ----------------------------------------------------------------------
    # public entry point
    # ----------------------------------------------------------------------
    def run(self) -> None:
        if self.peek_window_ms <= 0:
            # old continuous mode
            self._run_continuous()
        else:
            # new peek / gap mode
            self._run_peek()

    # ----------------------------------------------------------------------
    # continuous (legacy) loop
    # ----------------------------------------------------------------------
    def _run_continuous(self) -> None:
        try:
            while True:
                msg = self._recv_once()
                if msg:
                    if self.raw:
                        print(msg, flush=True)
                    else:
                        self._record(msg)

                # periodic report
                self._maybe_report()

                # nothing decoded → tiny nap to spare the CPU
                if not msg:
                    time.sleep(self._POLL_SLEEP)
        except KeyboardInterrupt:
            pass  # graceful exit – SIGPIPE already handled

    # ----------------------------------------------------------------------
    # peek / gap loop
    # ----------------------------------------------------------------------
    def _run_peek(self) -> None:
        w_sec = self.peek_window_ms / 1000.0
        gap_sec = self.peek_gap_ms / 1000.0
        try:
            while True:
                # -- gap / discard phase --------------------------------------
                self._purge_serial_buffer()
                if gap_sec > 0:
                    time.sleep(gap_sec)

                # -- active peek window ---------------------------------------
                t0 = time.monotonic()
                while (time.monotonic() - t0) < w_sec:
                    msg = self._recv_once()
                    if msg:
                        if self.raw:
                            print(msg, flush=True)
                        else:
                            self._record(msg)

                    # periodic report (same logic as continuous)
                    self._maybe_report()

                    # spare CPU when idle during window
                    if not msg:
                        time.sleep(self._POLL_SLEEP)
        except KeyboardInterrupt:
            pass

    # ----------------------------------------------------------------------
    # helper: read one MAVLink frame (non-blocking)
    # ----------------------------------------------------------------------
    def _recv_once(self):
        try:
            if self._recv_supports_exc:
                return self.master.recv_match(
                    blocking=False, timeout=0.02, exception_on_bad_data=False
                )
            return self.master.recv_match(blocking=False, timeout=0.02)
        except mavutil.mavlink.MAVError:
            self.parse_errors += 1
            return None

    # ----------------------------------------------------------------------
    # helper: periodic report & counter reset
    # ----------------------------------------------------------------------
    def _maybe_report(self) -> None:
        if self.raw:
            return
        now = time.time()
        if now >= self._next_report_ts:
            self._report()
            self._reset_counters()
            self._next_report_ts += self.period_sec

    # ----------------------------------------------------------------------
    # helper: purge driver / OS receive FIFO quickly
    # ----------------------------------------------------------------------
    def _purge_serial_buffer(self) -> None:
        try:
            port = getattr(self.master, "port", None)
            if port is None:
                return
            if hasattr(port, "reset_input_buffer"):
                port.reset_input_buffer()  # pyserial ≥ 3.0
            elif hasattr(port, "flushInput"):
                port.flushInput()  # older pyserial
        except Exception:  # pylint: disable=broad-except
            pass  # never fatal – just best-effort

    # ------------------------------------------------------------------
    # Counters & state
    # ------------------------------------------------------------------
    def _reset_counters(self) -> None:
        self.total_cnt = 0
        self.bytes_rx = 0
        self.crc_errors = 0
        if self._has_crc_fail:
            self.start_crc_fail = self.master.mav.total_crc_fail

        self.msg_counts: Dict[str, int] = defaultdict(int)

        self.rc_cnt = 0
        self.rc_latest = None

        self.radio_cnt = 0
        self.radio_latest = None

        self.exec_events.clear()

    # ------------------------------------------------------------------
    # Packet processing
    # ------------------------------------------------------------------
    def _record(self, msg: Any) -> None:
        self.total_cnt += 1
        self.bytes_rx += len(msg.get_msgbuf())
        mtype = msg.get_type()

        if mtype == self.CHANNEL_MSG:
            self.rc_cnt += 1
            self.rc_latest = msg
            if self.cmd_parse:
                self._evaluate_channels(msg)
        elif mtype == self.RADIO_MSG:
            self.radio_cnt += 1
            self.radio_latest = msg
        else:
            self.msg_counts[mtype] += 1

    # ------------------------------------------------------------------
    # RC-trigger logic
    # ------------------------------------------------------------------
    def _evaluate_channels(self, msg: Any) -> None:
        now_ms = self._now_ms()

        for ch_idx in range(16):
            if self.allowed_channels and (ch_idx + 1) not in self.allowed_channels:
                continue

            val = getattr(msg, f"chan{ch_idx + 1}_raw", None)
            if val is None:
                continue

            prev = self.last_values[ch_idx]
            if prev is None:
                self.last_values[ch_idx] = val
                continue

            diff = abs(val - prev)
            cand = self.pending[ch_idx]

            # candidate in progress?
            if cand:
                # cancelled (returned close to baseline)
                if diff < self.min_change:
                    self.pending[ch_idx] = None
                    continue
                # elapsed long enough?
                if now_ms - cand["start_ms"] >= self.persist_ms:
                    executed, runtime = self._try_execute(ch_idx + 1, cand["old_val"], val, now_ms)
                    if executed:
                        self.exec_events.append((ch_idx + 1, cand["old_val"], val, runtime))
                        self.last_values[ch_idx] = val
                        self.pending[ch_idx] = None
                continue

            # start new candidate?
            if diff >= self.min_change:
                self.pending[ch_idx] = dict(start_ms=now_ms, old_val=prev)

    # ------------------------------------------------------------------
    # Command launcher
    # ------------------------------------------------------------------
    def _try_execute(
        self,
        channel: int,
        old_val: int,
        new_val: int,
        now_ms: int,
    ) -> Tuple[bool, int]:
        if now_ms < self.next_allowed_exec_ms:
            return False, 0
        if not self.SCRIPT.exists():
            sys.stderr.write(f"[mavlink_rx] channels.sh not found at {self.SCRIPT}\n")
            return False, 0

        start = time.time()
        try:
            res = subprocess.run(
                [str(self.SCRIPT), str(channel), str(new_val)],
                capture_output=True,
            )
            runtime_ms = int((time.time() - start) * 1000)

            if res.returncode != 0:
                sys.stderr.write(
                    f"[mavlink_rx] channels.sh error (ch {channel}): "
                    f"{res.stderr.decode(errors='ignore')}\n"
                )
        except Exception as exc:  # pylint: disable=broad-except
            sys.stderr.write(f"[mavlink_rx] Exec failed (ch {channel}): {exc!s}\n")
            return False, 0

        self.next_allowed_exec_ms = now_ms + self.pause_ms
        return True, runtime_ms

    # ------------------------------------------------------------------
    # Reporting helpers
    # ------------------------------------------------------------------
    def _now_ms(self) -> int:
        return int((time.time() - self._start_ts) * 1000)

    def _report(self) -> None:
        ts = self._now_ms()
        unique_cnt = len(self.msg_counts) + (1 if self.rc_cnt else 0) + (1 if self.radio_cnt else 0)
        crc_delta = self.parse_errors
        if self._has_crc_fail:
            crc_delta += self.master.mav.total_crc_fail - self.start_crc_fail

        # MAVLINK_STATS
        print(
            f"{ts} MAVLINK_STATS "
            f"{self.period_ms}:{self.total_cnt}:{unique_cnt}:{self.bytes_rx}:{crc_delta}"
        )

        # RC_CHANNELS_OVERRIDE
        if self.rc_cnt and self.rc_latest:
            m = self.rc_latest
            chan_str = ":".join(str(getattr(m, f"chan{i}_raw", 0)) for i in range(1, 17))
            print(
                f"{ts} {self.CHANNEL_MSG} {self.rc_cnt} "
                f"{m.target_system} {m.target_component} {chan_str}"
            )

        # RADIO_STATUS
        if self.radio_cnt and self.radio_latest:
            r = self.radio_latest
            radio_str = (
                f"{r.rssi}:{r.remrssi}:{r.txbuf}:{r.noise}:"
                f"{r.remnoise}:{r.rxerrors}:{r.fixed}"
            )
            print(f"{ts} {self.RADIO_MSG} {self.radio_cnt} {radio_str}")

        # MAVLINK_EXEC lines
        for idx, (ch, old_v, new_v, runtime) in enumerate(self.exec_events, 1):
            print(f"{ts} MAVLINK_EXEC {idx}:{ch}:{old_v}:{new_v}:{runtime}")

        sys.stdout.flush()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_args(argv: List[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MAVLink sniffer with RC-triggered commands.")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial device")
    p.add_argument("--baud", default=460800, type=int, help="Baud rate (default 460800)")
    p.add_argument("--period", default=1000, type=int, help="Aggregation period in ms")
    p.add_argument("--raw", action="store_true", help="Dump every decoded packet")
    p.add_argument("--dialect", default="common", help="MAVLink dialect")
    p.add_argument("--robust", action="store_true", help="Enable robust parser")

    # peek / gap sampling
    p.add_argument(
        "--peek-window",
        type=int,
        default=0,
        metavar="MS",
        help="Listen / decode for this many ms each cycle (0 = continuous)",
    )
    p.add_argument(
        "--peek-gap",
        type=int,
        default=None,
        metavar="MS",
        help="Discard / sleep for this many ms between windows "
        "(default: same as --peek-window)",
    )

    # command-execution flags
    p.add_argument("--command-parse", action="store_true", help="Enable RC ➜ command execution")
    p.add_argument("--min-change", type=int, default=100, help="Min Δ to trigger (default 100)")
    p.add_argument("--persist", type=int, default=500, help="Δ must persist (ms)")
    p.add_argument("--pause", type=int, default=500, help="Pause after each command (ms)")
    p.add_argument(
        "--channels",
        help="Comma-separated list of channels to monitor (e.g. 5,6,7). Default: all",
    )
    return p.parse_args(argv)


def _parse_channel_filter(s: Optional[str]) -> Optional[set[int]]:
    if not s:
        return None
    try:
        chans = {int(x) for x in s.split(",") if x.strip()}
        if any(ch < 1 or ch > 16 for ch in chans):
            raise ValueError
        return chans
    except ValueError:
        sys.stderr.write("--channels expects numbers 1-16 separated by commas\n")
        sys.exit(1)


def main(argv: List[str] | None = None) -> None:
    args = _parse_args(argv)
    sniffer = MavlinkSniffer(
        port=args.port,
        baud=args.baud,
        period_ms=args.period,
        raw=args.raw,
        dialect=args.dialect,
        robust=args.robust,
        command_parse=args.command_parse,
        min_change=args.min_change,
        persist_ms=args.persist,
        pause_ms=args.pause,
        channels_filter=_parse_channel_filter(args.channels),
        peek_window_ms=args.peek_window,
        peek_gap_ms=args.peek_gap,
    )
    sniffer.run()


if __name__ == "__main__":
    main()
