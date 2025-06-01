#!/usr/bin/env python3
"""
MAVLink Sniffer (v0.4)
======================

* Aggregates message counts in user-set windows (default 1 s).
* Prints RC channel 1-16 data separately.
* `--raw` dumps every decoded message.
* `--robust` engages the parser’s robust mode if available.
* Minimal trigger framework for future “if-this-then-that”.
* **Monkey-patch** included to accept MAVLink-2 frames whose
  incompat-flags byte sets any of the still-reserved bits 4–7.
  (These frames were rejected by pymavlink ≤ 2.4.x.)

Tested with pymavlink 2.4.8 → 2.4.47 and the current 3.x-dev branch.
"""

from __future__ import annotations

import argparse
import inspect
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

# ----------------------------------------------------------------------
# Import pymavlink and monkey-patch the parser
# ----------------------------------------------------------------------
try:
    from pymavlink import mavutil
except ModuleNotFoundError as exc:
    sys.stderr.write("Missing dependency – install with:  pip install pymavlink pyserial\n")
    raise SystemExit(1) from exc

# ------------ Monkey-patch begins here ----------------------------------------
# Older pymavlink rejects any MAVLink-2 frame whose incompat-flags byte
# has bits other than bit-0 (signing).  New autopilot builds set bits
# 4-7 for timestamp/encryption experiments, so we mask that error.
_orig_parse_char = mavutil.mavlink.MAVLink.parse_char

def _patched_parse_char(self, c):
    """
    Wrapper around MAVLink.parse_char that silently drops frames flagged
    only for *invalid incompat_flags*.  All other MAVErrors still raise.
    """
    try:
        return _orig_parse_char(self, c)
    except mavutil.mavlink.MAVError as e:
        if "invalid incompat_flags" in str(e):
            # Ignore the offending frame, stay in sync, return None
            return None
        raise

mavutil.mavlink.MAVLink.parse_char = _patched_parse_char
# ------------ Monkey-patch ends here ------------------------------------------

# ---------------------------------------------------------------------------
# Trigger skeleton (future expansion)
# ---------------------------------------------------------------------------
class Trigger:
    """Call *action* when *predicate* on *msg* returns True."""

    def __init__(
        self,
        msg_type: str,
        predicate: Callable[[Any], bool],
        action: Callable[[Any], None],
    ) -> None:
        self.msg_type = msg_type
        self.predicate = predicate
        self.action = action

    def evaluate(self, msg: Any) -> None:
        if msg.get_type() == self.msg_type and self.predicate(msg):
            self.action(msg)


# ---------------------------------------------------------------------------
# Sniffer core
# ---------------------------------------------------------------------------
class MavlinkSniffer:
    """MAVLink sniffer with aggregation, raw dump & triggers."""

    def __init__(
        self,
        port: str = "/dev/ttyS3",
        baud: int = 460_800,
        period: float = 1.0,
        raw: bool = False,
        dialect: str = "common",
        robust: bool = False,
        sign_key: bytes | None = None,
        link_id: int = 0,
    ) -> None:
        # ----------- open serial connection --------------------------------
        conn_kwargs = dict(baud=baud, dialect=dialect, autoreconnect=True)
        # *robust_parsing* arrived in pymavlink 2.4.15
        if robust and "robust_parsing" in inspect.signature(mavutil.mavlink_connection).parameters:
            conn_kwargs["robust_parsing"] = True
        self.master = mavutil.mavlink_connection(port, **conn_kwargs)

        # ---- detect whether recv_match supports exception_on_bad_data ------
        self._recv_supports_exc = "exception_on_bad_data" in inspect.signature(
            self.master.recv_match
        ).parameters

        # ---- signing verification (RX-only) --------------------------------
        if sign_key:
            if len(sign_key) != 32:
                raise ValueError("Signing key must be 32 bytes (64-hex chars)")
            self.master.mav.signing.secret_key = sign_key
            self.master.mav.signing.link_id = link_id
            self.master.mav.signing.sign_outgoing = False
            print(f"Signing key installed (link-id {link_id}).")

        # ---- bookkeeping ---------------------------------------------------
        self.period = period
        self.raw = raw
        self.triggers: List[Trigger] = []

        self._reset_counters()
        self._last_tick = time.time()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def run(self) -> None:
        print(
            f"Listening on {self.master.port} @{self.master.baud} | "
            f"agg={self.period}s | raw={self.raw} | robust={getattr(self.master, 'robust_parsing', False)}"
        )
        try:
            while True:
                msg = self._recv_once()
                if msg:
                    if self.raw:
                        print(msg)
                        continue
                    self._record(msg)
                    self._evaluate_triggers(msg)

                if not self.raw and (time.time() - self._last_tick) >= self.period:
                    self._report()
                    self._reset_counters()
                    self._last_tick = time.time()
        except KeyboardInterrupt:
            print("\nInterrupted – bye!")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _recv_once(self):
        if self._recv_supports_exc:
            return self.master.recv_match(
                blocking=False, timeout=0.02, exception_on_bad_data=False
            )
        # older pymavlink
        return self.master.recv_match(blocking=False, timeout=0.02)

    def _reset_counters(self) -> None:
        self.msg_counts: Dict[str, int] = defaultdict(int)
        self.rc_count = 0
        self.rc_latest: list[int | None] = [None] * 16

    def _record(self, msg: Any) -> None:
        t = msg.get_type()
        if t == "RC_CHANNELS":
            self.rc_count += 1
            for i in range(16):
                field = f"chan{i + 1}_raw"
                if hasattr(msg, field):
                    self.rc_latest[i] = getattr(msg, field)
        else:
            self.msg_counts[t] += 1

    def _report(self) -> None:
        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(f"\n[{ts}] Aggregation window {self.period:.1f}s")
        if self.rc_count:
            print(f"RC_CHANNELS: {self.rc_count} msgs")
            for ch, val in enumerate(self.rc_latest, 1):
                if val is not None:
                    print(f"  CH{ch:>2}: {val}")
        for mtype in sorted(self.msg_counts):
            print(f"{mtype}: {self.msg_counts[mtype]}")
        print("-" * 60)

    def _evaluate_triggers(self, msg: Any) -> None:
        for trig in self.triggers:
            trig.evaluate(msg)


# ---------------------------------------------------------------------------
# CLI glue
# ---------------------------------------------------------------------------

def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Simple MAVLink sniffer with aggregation and raw dump.")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    p.add_argument("--baud", default=420000, type=int, help="Baud rate (default: 420000)")
    p.add_argument("--period", default=1.0, type=float, help="Aggregation period in seconds (default: 1)")
    p.add_argument("--raw", action="store_true", help="Dump every decoded message – bypass aggregation/triggers")
    p.add_argument("--dialect", default="common", help="MAVLink dialect (default: common)")
    p.add_argument("--robust", action="store_true", help="Enable robust parser (silences BAD_DATA spam if supported)")

    key_grp = p.add_mutually_exclusive_group()
    key_grp.add_argument("--sign-key", help="32-byte signing key as hex string (MAVLink 2 signing)")
    key_grp.add_argument("--sign-key-file", type=Path, help="File containing 32-byte signing key as hex")
    p.add_argument("--link-id", type=int, default=0, help="MAVLink signing link-id (default 0)")
    return p.parse_args(argv)


def _load_key(args: argparse.Namespace) -> bytes | None:
    if args.sign_key:
        return bytes.fromhex(args.sign_key.strip())
    if args.sign_key_file:
        content = args.sign_key_file.read_text().strip()
        return bytes.fromhex(content)
    return None


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    key = _load_key(args)
    sniffer = MavlinkSniffer(
        port=args.port,
        baud=args.baud,
        period=args.period,
        raw=args.raw,
        dialect=args.dialect,
        robust=args.robust,
        sign_key=key,
        link_id=args.link_id,
    )
    sniffer.run()


if __name__ == "__main__":
    main()
