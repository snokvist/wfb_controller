#!/usr/bin/env python3
"""
MAVLink Sniffer (v0.9.1)
========================
* Aggregation period in **milliseconds**.
* Logs MAVLINK_STATS / RC_CHANNELS_OVERRIDE / RADIO_STATUS.
* MAVLINK_STATS now includes bytes-received + CRC-error counts and works
  even when `MAVLink.total_crc_fail` is missing.
* Flushes stdout after every report.
"""

from __future__ import annotations

import argparse
import inspect
import sys
import time
from collections import defaultdict
from typing import Any, Dict, List

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    sys.stderr.write("Missing pymavlink – install with:  pip install pymavlink pyserial\n")
    sys.exit(1)


class MavlinkSniffer:
    CHANNEL_MSG = "RC_CHANNELS_OVERRIDE"
    RADIO_MSG = "RADIO_STATUS"

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 460_800,
        period_ms: int = 1000,
        raw: bool = False,
        dialect: str = "common",
        robust: bool = False,
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

        # Some pymavlink builds lack total_crc_fail – detect once
        self._has_crc_fail = hasattr(self.master.mav, "total_crc_fail")

        self.period_ms = period_ms
        self.period_sec = period_ms / 1000.0
        self.raw = raw

        self._start_ts = time.time()
        self._last_tick = self._start_ts
        self._reset_counters()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self) -> None:
        print(
            f"Listening on {self.master.port} @{self.master.baud} "
            f"| agg={self.period_ms} ms | raw={self.raw} | robust={getattr(self.master,'robust_parsing',False)}"
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
    # Helpers
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

    def _record(self, msg: Any) -> None:
        self.total_cnt += 1
        self.bytes_rx += len(msg.get_msgbuf())
        mtype = msg.get_type()

        if mtype == self.CHANNEL_MSG:
            self.rc_cnt += 1
            self.rc_latest = msg
        elif mtype == self.RADIO_MSG:
            self.radio_cnt += 1
            self.radio_latest = msg
        else:
            self.msg_counts[mtype] += 1

    # ------------------------------------------------------------------
    # Printing
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

        sys.stdout.flush()  # ensure immediate delivery


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_args(argv: List[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Lightweight MAVLink sniffer.")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default /dev/ttyUSB0)")
    p.add_argument("--baud", default=460800, type=int, help="Baud rate (default 460800)")
    p.add_argument("--period", default=1000, type=int, help="Aggregation period in ms (default 1000)")
    p.add_argument("--raw", action="store_true", help="Dump every decoded packet")
    p.add_argument("--dialect", default="common", help="MAVLink dialect")
    p.add_argument("--robust", action="store_true", help="Enable robust parser")
    return p.parse_args(argv)


def main(argv: List[str] | None = None) -> None:
    args = _parse_args(argv)
    sniffer = MavlinkSniffer(
        port=args.port,
        baud=args.baud,
        period_ms=args.period,
        raw=args.raw,
        dialect=args.dialect,
        robust=args.robust,
    )
    sniffer.run()


if __name__ == "__main__":
    main()
