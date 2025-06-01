#!/usr/bin/env python3
"""
mavlink_rx.py – lightweight MAVLink sniffer / command bridge
------------------------------------------------------------

* Aggregates statistics every *period* ms (default 1000).
* Prints three records per aggregation window:
      MAVLINK_STATS  RC_CHANNELS_OVERRIDE  RADIO_STATUS
* Optional RC-command execution through /usr/bin/channels.sh.
* Low-CPU implementation: **blocking serial reads** with a deadline equal
  to the next aggregation timestamp.

Author: 2025-06 – ChatGPT (o3) and user iterative design
"""

from __future__ import annotations

import argparse
import inspect
import queue
import shlex
import signal
import subprocess
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------+
#  Graceful exit on closed stdout pipe (| head, | grep, …)                   |
# ---------------------------------------------------------------------------+
signal.signal(signal.SIGPIPE, signal.SIG_DFL)

# ---------------------------------------------------------------------------+
#  Third-party                                                               |
# ---------------------------------------------------------------------------+
try:
    from pymavlink import mavutil
except ModuleNotFoundError as exc:  # pragma: no cover
    sys.stderr.write("Missing dependency – install with:  pip install pymavlink\n")
    raise SystemExit(1) from exc


# ===========================================================================+
#  Sniffer                                                                   |
# ===========================================================================+
class MavlinkSniffer:
    # ------------------------------------------------------------------ init
    def __init__(
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
    ) -> None:
        # ------------- open serial (robust_parsing only when supported)
        conn_kwargs = dict(baud=baud, dialect=dialect, autoreconnect=True)
        if robust and "robust_parsing" in inspect.signature(mavutil.mavlink_connection).parameters:
            conn_kwargs["robust_parsing"] = True

        self.master = mavutil.mavlink_connection(port, **conn_kwargs)

        # ------------- capability flags
        self._recv_supports_exc = (
            "exception_on_bad_data" in inspect.signature(self.master.recv_match).parameters
        )

        # ------------- timing
        self.period_ms = max(1, period_ms)
        self.period_sec = self.period_ms / 1000.0
        self._start_ts = time.time()
        self._next_report_ts = self._start_ts + self.period_sec

        # ------------- runtime state
        self.raw = raw
        self.robust = robust
        self.total_bytes = 0
        self.parse_errors = 0
        self._reset_counters()

        # ------------- command execution parameters
        self.command_parse = command_parse
        self.min_change = min_change
        self.persist_ms = persist_ms
        self.pause_ms = pause_ms
        self.channels_filter = channels_filter
        self.cmd_busy = False
        self.exec_queue: "queue.Queue[Tuple[int,int,int,int]]" = queue.Queue()
        self.pending: Dict[int, Tuple[int, float]] = {}  # ch -> (orig_val, timestamp_ms)
        self.last_exec_time_ms = 0

        print(
            f"Listening on {self.master.port} @{self.master.baud}  "
            f"| agg={self.period_ms} ms  raw={self.raw}  robust={robust}",
            flush=True,
        )

    # =============================================================== run loop
    def run(self) -> None:  # noqa: D401
        try:
            while True:
                msg = self._recv_once()
                if msg:
                    if self.raw:
                        print(msg, flush=True)
                        continue
                    self._process_msg(msg)

                now = time.time()
                if not self.raw and now >= self._next_report_ts:
                    self._report()
                    self._reset_counters()
                    self._next_report_ts += self.period_sec
        except KeyboardInterrupt:
            sys.stderr.write("Interrupted – bye!\n")

    # ----------------------------------------------------- message processing
    def _process_msg(self, msg: Any) -> None:
        mtype = msg.get_type()
        self.msg_counts[mtype] += 1
        self.total_bytes += len(msg.get_msgbuf() or b"")

        if mtype == "RC_CHANNELS_OVERRIDE":
            self._handle_rc(msg)
        elif mtype == "RADIO_STATUS":
            self.last_radio = msg

    # ------------------------------------------------------------ RC handler
    def _handle_rc(self, msg: Any) -> None:
        self.rc_count += 1
        now_ms = int((time.time() - self._start_ts) * 1000)
        for i in range(16):
            ch_num = i + 1
            field = f"chan{ch_num}_raw"
            if not hasattr(msg, field):
                continue
            val = getattr(msg, field)
            self.rc_latest[ch_num] = val

            if not self.command_parse:
                continue
            if self.channels_filter and ch_num not in self.channels_filter:
                continue

            last_val = self.last_sent.get(ch_num, val)
            if abs(val - last_val) < self.min_change:
                # no significant change
                if ch_num in self.pending:
                    del self.pending[ch_num]  # reset pending
                continue

            # change detected
            if ch_num not in self.pending:
                self.pending[ch_num] = (last_val, now_ms)
                continue

            orig_val, t0 = self.pending[ch_num]
            if now_ms - t0 >= self.persist_ms:
                # Persisted – fire command if allowed
                if not self.cmd_busy and (now_ms - self.last_exec_time_ms) >= self.pause_ms:
                    self._exec_command(ch_num, orig_val, val)
                self.last_sent[ch_num] = val
                self.pending.pop(ch_num, None)

    # ----------------------------------------------------------- exec helper
    def _exec_command(self, ch: int, old_val: int, new_val: int) -> None:
        def _worker():
            nonlocal ch, old_val, new_val
            start = time.perf_counter()
            try:
                subprocess.run(
                    ["/usr/bin/channels.sh", str(ch), str(new_val)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    check=True,
                )
            except subprocess.CalledProcessError as exc:
                sys.stderr.write(
                    f"CMD_ERROR channel {ch}: {exc.stderr.decode().strip()}\n"
                )
            except FileNotFoundError:
                sys.stderr.write("/usr/bin/channels.sh not found\n")
            finally:
                elapsed_ms = int((time.perf_counter() - start) * 1000)
                self.exec_queue.put((ch, old_val, new_val, elapsed_ms))
                self.cmd_busy = False
                self.last_exec_time_ms = int((time.time() - self._start_ts) * 1000)

        self.cmd_busy = True
        threading.Thread(target=_worker, daemon=True).start()

    # ------------------------------------------------------------- recv once
    def _recv_once(self) -> Optional[Any]:
        """
        Blocking read until either:
        * at least one MAVLink message parsed, or
        * aggregation window deadline reached.
        """
        timeout = max(0.0, self._next_report_ts - time.time())
        try:
            if self._recv_supports_exc:
                return self.master.recv_match(
                    blocking=True, timeout=timeout, exception_on_bad_data=False
                )
            return self.master.recv_match(blocking=True, timeout=timeout)
        except mavutil.mavlink.MAVError:
            self.parse_errors += 1
            return None

    # --------------------------------------------------------------- counters
    def _reset_counters(self) -> None:
        self.msg_counts: Dict[str, int] = defaultdict(int)
        self.rc_count = 0
        self.rc_latest: Dict[int, int] = {}
        self.last_radio: Optional[Any] = None
        # Stats deltas
        self.start_parsed = getattr(self.master.mav, "total_packets_received", 0)
        self.start_crc_fail = getattr(self.master.mav, "total_crc_fail", 0)
        self.start_parse_err = getattr(self, "parse_errors", 0)
        # Command tracking
        self.exec_records: List[Tuple[int, int, int, int]] = []
        self.last_sent: Dict[int, int] = {}

    # ---------------------------------------------------------------- report
    def _report(self) -> None:
        ts_ms = int((time.time() - self._start_ts) * 1000)

        # Pull any finished execs
        while True:
            try:
                self.exec_records.append(self.exec_queue.get_nowait())
            except queue.Empty:
                break

        # ---------- MAVLINK_STATS
        total_pkts = getattr(self.master.mav, "total_packets_received", 0)
        total_crc = getattr(self.master.mav, "total_crc_fail", 0)
        parsed_delta = total_pkts - self.start_parsed
        crc_delta = total_crc - self.start_crc_fail
        unique_msgs = len(self.msg_counts)
        stats_line = (
            f"{ts_ms} MAVLINK_STATS {self.period_ms}:{parsed_delta}:{unique_msgs}:"
            f"{self.total_bytes}:{crc_delta}:{self.parse_errors - self.start_parse_err}"
        )
        print(stats_line)

        # ---------- RC_CHANNELS_OVERRIDE
        if self.rc_count:
            rc_line = (
                f"{ts_ms} RC_CHANNELS_OVERRIDE {self.rc_count} "
                f"{' '.join(str(self.rc_latest.get(i, 0)) for i in range(1, 17))}"
            )
            print(rc_line)

        # ---------- RADIO_STATUS
        if self.last_radio:
            r = self.last_radio
            radio_line = (
                f"{ts_ms} RADIO_STATUS {self.msg_counts['RADIO_STATUS']} "
                f"{r.rssi}:{r.remrssi}:{r.txbuf}:{r.noise}:{r.remnoise}:"
                f"{r.rxerrors}:{r.fixed}"
            )
            print(radio_line)

        # ---------- command executions
        for idx, (ch, old_v, new_v, elapsed) in enumerate(self.exec_records, 1):
            print(f"{ts_ms} MAVLINK_EXEC {idx}:{ch}:{old_v}:{new_v}:{elapsed}")
        self.exec_records.clear()

        # ensure downstream reader sees the lines immediately
        sys.stdout.flush()


# ===========================================================================+
#  CLI                                                                       |
# ===========================================================================+
def _parse_channels(text: str) -> set[int]:
    try:
        return {int(tok) for tok in text.split(",") if tok}
    except ValueError as exc:
        raise argparse.ArgumentTypeError("channels must be comma-separated ints") from exc


def build_cli() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser("mavlink_rx.py – MAVLink sniffer / RC executor")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=460800, help="Baud rate (default 460800)")
    p.add_argument("--period", type=int, default=1000, help="Aggregation period in ms (default 1000)")
    p.add_argument("--dialect", default="common", help="MAVLink dialect (default common)")
    p.add_argument("--robust", action="store_true", help="Enable robust parser (if supported)")
    p.add_argument("--raw", action="store_true", help="Print every decoded message; no stats/exec")

    # Command-parsing options
    p.add_argument("--command-parse", action="store_true", help="Enable RC→command execution")
    p.add_argument("--min-change", type=int, default=100, help="Min Δ to trigger (default 100)")
    p.add_argument("--persist", type=int, default=500, help="Δ must persist this long ms (default 500)")
    p.add_argument("--pause", type=int, default=500, help="Pause after exec (default 500 ms)")
    p.add_argument("--channels", type=_parse_channels, help="Comma-sep list, e.g. 5,6,7")
    return p


def main(argv: Optional[List[str]] = None) -> None:
    args = build_cli().parse_args(argv)
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
        channels_filter=args.channels,
    )
    sniffer.run()


if __name__ == "__main__":  # pragma: no cover
    main()
