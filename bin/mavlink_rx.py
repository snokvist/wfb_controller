#!/usr/bin/env python3
"""
MAVLink Sniffer with RC-triggered commands (v1.1)
=================================================
* Aggregation period in ms (`--period`).
* Logs MAVLINK_STATS / RC_CHANNELS_OVERRIDE / RADIO_STATUS.
* Adds bytes-received & CRC-error counts.
* RC-value command execution:
    /usr/bin/channels.sh <channel> <value>
  – options: --command-parse --channels 5,6 --min-change 100
             --persist 500 --pause 500
"""

from __future__ import annotations

import argparse
import inspect
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    sys.stderr.write("Missing pymavlink – install with:\n  pip install pymavlink pyserial\n")
    sys.exit(1)


# ---------------------------------------------------------------------------
class MavlinkSniffer:
    CHANNEL_MSG = "RC_CHANNELS_OVERRIDE"
    RADIO_MSG = "RADIO_STATUS"
    SCRIPT = Path("/usr/bin/channels.sh")

    # .....................................................................
    def __init__(
        self,
        port: str,
        baud: int,
        period_ms: int,
        raw: bool,
        dialect: str,
        robust: bool,
        cmd_parse: bool,
        min_change: int,
        persist_ms: int,
        pause_ms: int,
        channel_filter: Optional[set[int]],
    ) -> None:

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
        if robust and "robust_parsing" in inspect.signature(
            mavutil.mavlink_connection
        ).parameters:
            conn_kwargs["robust_parsing"] = True

        self.master = mavutil.mavlink_connection(port, **conn_kwargs)
        self._recv_supports_exc = "exception_on_bad_data" in inspect.signature(
            self.master.recv_match
        ).parameters
        self._has_crc_fail = hasattr(self.master.mav, "total_crc_fail")

        # timings
        self.period_ms = period_ms
        self.period_sec = period_ms / 1000.0
        self.raw = raw

        # command trigger settings
        self.cmd_parse = cmd_parse
        self.min_change = min_change
        self.persist_ms = persist_ms
        self.pause_ms = pause_ms
        self.next_allowed_exec_ms = 0
        self.allowed_channels = channel_filter  # None => all

        # per-channel state
        self.pending: List[Optional[Dict[str, Any]]] = [None] * 16
        self.last_values: List[Optional[int]] = [None] * 16

        # bookkeeping
        self._start_ts = time.time()
        self._last_tick = self._start_ts
        self._reset_counters()

    # ------------------------------------------------------------------
    def run(self) -> None:
        print(
            f"Listening on {self.master.port} @{self.master.baud} | agg={self.period_ms} ms "
            f"| cmd_parse={self.cmd_parse} | minΔ={self.min_change} | persist={self.persist_ms} ms "
            f"| pause={self.pause_ms} ms"
        )
        try:
            while True:
                msg = self._recv_once()
                if msg:
                    if self.raw:
                        print(msg)
                        continue
                    self._record(msg)

                if not self.raw and (time.time() - self._last_tick) >= self.period_sec:
                    self._report()
                    self._reset_counters()
                    self._last_tick = time.time()
        except KeyboardInterrupt:
            print("\nInterrupted – bye!")

    # ------------------------------------------------------------------
    # Serial receive
    # ------------------------------------------------------------------
    def _recv_once(self):
        try:
            if self._recv_supports_exc:
                return self.master.recv_match(
                    blocking=False, timeout=0.02, exception_on_bad_data=False
                )
            return self.master.recv_match(blocking=False, timeout=0.02)
        except mavutil.mavlink.MAVError:
            self.crc_errors += 1
            return None

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

        self.exec_events: List[Tuple[int, int, int, int]] = []  # (chan, old, new, runtime_ms)

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

            # existing candidate waiting?
            if cand:
                # abort if value returned close to baseline
                if diff < self.min_change:
                    self.pending[ch_idx] = None
                    continue
                # still outside – ready?
                if now_ms - cand["start_ms"] >= self.persist_ms:
                    executed, runtime = self._try_execute(
                        ch_idx + 1, cand["old_val"], val, now_ms
                    )
                    if executed:
                        self.exec_events.append(
                            (ch_idx + 1, cand["old_val"], val, runtime)
                        )
                        self.last_values[ch_idx] = val
                        self.pending[ch_idx] = None
                # keep waiting otherwise
                continue

            # start new candidate?
            if diff >= self.min_change:
                self.pending[ch_idx] = dict(start_ms=now_ms, old_val=prev)

    # ------------------------------------------------------------------
    # Command launcher
    # ------------------------------------------------------------------
    def _try_execute(
        self, channel: int, old_val: int, new_val: int, now_ms: int
    ) -> Tuple[bool, int]:
        if now_ms < self.next_allowed_exec_ms:
            return False, 0

        if not self.SCRIPT.exists():
            sys.stderr.write(f"channels.sh not found at {self.SCRIPT}\n")
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
                    f"channels.sh error (ch {channel}): {res.stderr.decode(errors='ignore')}\n"
                )
            # even on non-zero rc we treat it as executed to honour the pause
        except Exception as exc:
            sys.stderr.write(f"Exec failed (ch {channel}): {exc}\n")
            return False, 0

        # impose pause
        self.next_allowed_exec_ms = now_ms + self.pause_ms
        return True, runtime_ms

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------
    def _now_ms(self) -> int:
        return int((time.time() - self._start_ts) * 1000)

    def _report(self) -> None:
        ts = self._now_ms()
        unique_cnt = len(self.msg_counts) + (1 if self.rc_cnt else 0) + (
            1 if self.radio_cnt else 0
        )
        crc_delta = self.crc_errors
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
            chan_str = ":".join(str(getattr(m, f'chan{i}_raw', 0)) for i in range(1, 17))
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
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    p.add_argument("--baud", default=460800, type=int, help="Baud rate (default 460800)")
    p.add_argument("--period", default=1000, type=int, help="Aggregation period in ms")
    p.add_argument("--raw", action="store_true", help="Dump every decoded packet")
    p.add_argument("--dialect", default="common", help="MAVLink dialect")
    p.add_argument("--robust", action="store_true", help="Enable robust parser")

    # command-execution flags
    p.add_argument("--command-parse", action="store_true", help="Enable RC → command execution")
    p.add_argument("--min-change", type=int, default=100, help="Min Δ to trigger (default 100)")
    p.add_argument("--persist", type=int, default=500, help="Δ must persist this long (ms)")
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
        lst = {int(x) for x in s.split(",") if x.strip()}
        bad = [ch for ch in lst if ch < 1 or ch > 16]
        if bad:
            raise ValueError
        return lst
    except ValueError:
        sys.stderr.write("--channels accepts 1-16 comma separated\n")
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
        cmd_parse=args.command_parse,
        min_change=args.min_change,
        persist_ms=args.persist,
        pause_ms=args.pause,
        channel_filter=_parse_channel_filter(args.channels),
    )
    sniffer.run()


if __name__ == "__main__":
    main()
