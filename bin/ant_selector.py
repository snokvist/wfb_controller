#!/usr/bin/env python3
"""
antenna_selector.py – lean antenna‑adapter selector
--------------------------------------------------
* Reads a TCP text stream that alternates PKT / RX_ANT lines (as produced by
  `echo "stream video_agg_rx stdout" | nc 127.0.0.1 9500`).
* Keeps the most recent (packet_count, rssi_avg) for every individual antenna.
* Collapses antennas that belong to the same physical adapter: the adapter id
  is the first 14 hex digits of the antenna id, e.g. `7f000001000000` is the
  adapter for both `7f00000100000000` and `7f00000100000001`.
* Computes a simple score ≈ “higher rssi_avg is better, higher packet_count is
  better” and keeps the adapters sorted by that score.
* Whenever the best adapter becomes **significantly** better than the current
  active adapter **and** at least `--min-switch-interval` seconds have passed,
  the program issues a switch command.
* If `--switch-cmd` is omitted or empty, the command is printed to stdout
  instead of executed – handy for dry-runs & debugging.
* `--verbose / -v` prints every successful RX_ANT parse, every score table, and
  detailed messages whenever a switch is triggered. In any switch event, the
  full sorted ranking with scores is always printed to ease troubleshooting.

2025‑05‑18 – v0.2 : added ranking printout on every switch.
"""

import argparse
import socket
import sys
import time
import re
import subprocess
from collections import defaultdict, namedtuple
from typing import Dict, List

###############################################################################
# Helpers & data structures
###############################################################################
AntSample = namedtuple("AntSample", "timestamp packet_count rssi_avg")

# Regex that extracts the interesting parts of an RX_ANT line
_RX_RE = re.compile(r"RX_ANT\s+[^\s]+\s+(?P<antid>[0-9a-fA-F]{16})\s+(?P<metrics>[0-9:-]+)")

###############################################################################
# Core class
###############################################################################
class AntennaSelector:
    def __init__(
        self,
        *,
        host: str,
        port: int,
        tx_adapters: List[str],
        switch_cmd: str,
        min_switch_interval: float,
        rssi_hysteresis: float,
        verbose: bool,
    ):
        self.host = host
        self.port = port
        self.tx_adapters = {a.lower() for a in tx_adapters}
        self.switch_cmd = switch_cmd
        self.min_switch_interval = min_switch_interval
        self.rssi_hysteresis = rssi_hysteresis  # positive dB value
        self.verbose = verbose

        # runtime state
        self.samples: Dict[str, AntSample] = {}          # antenna‑id → latest sample
        self.adapter_best_score: Dict[str, float] = {}   # adapter‑id → score
        self.current_adapter: str | None = None
        self.last_switch_time = 0.0

    # ------------------------------------------------------------------
    # Top‑level loop: reconnect forever
    # ------------------------------------------------------------------
    def run_forever(self):
        while True:
            try:
                self._run_once()
            except (ConnectionRefusedError, OSError) as e:
                print(f"[warn] connection error: {e}; retrying in 3 s", file=sys.stderr)
                time.sleep(3)

    # ------------------------------------------------------------------
    def _run_once(self):
        with socket.create_connection((self.host, self.port)) as sock:
            sock_file = sock.makefile("r", encoding="utf-8", errors="replace")
            # send the same line you type when using nc interactively
            sock.sendall(b"stream video_agg_rx stdout\n")

            for line in sock_file:
                if "RX_ANT" in line:
                    self._handle_rx_ant(line.strip())

    # ------------------------------------------------------------------
    def _handle_rx_ant(self, line: str):
        m = _RX_RE.search(line)
        if not m:
            return  # ignore unparsable lines

        ant_id_full = m.group("antid").lower()
        adapter_id = ant_id_full[:14]

        # metrics looks like 1054:-40:-33:-24:-4:12:16
        metrics = m.group("metrics").split(":")
        if len(metrics) < 3:
            return
        try:
            pkt_cnt = int(metrics[0])
            rssi_avg = int(metrics[2])
        except ValueError:
            return

        ts = time.time()
        self.samples[ant_id_full] = AntSample(ts, pkt_cnt, rssi_avg)
        if self.verbose:
            print(f"[DBG] {ant_id_full}  pkts={pkt_cnt}  rssi_avg={rssi_avg}")

        self._recompute_scores()
        self._maybe_switch()

    # ------------------------------------------------------------------
    def _recompute_scores(self):
        self.adapter_best_score.clear()
        for ant_id, s in self.samples.items():
            adapter_id = ant_id[:14]
            if adapter_id not in self.tx_adapters:
                continue
            score = (-s.rssi_avg) * 10_000 + s.packet_count  # primary key RSSI, secondary pkts
            if score > self.adapter_best_score.get(adapter_id, -float("inf")):
                self.adapter_best_score[adapter_id] = score

        if self.verbose and self.adapter_best_score:
            ordered = sorted(self.adapter_best_score.items(), key=lambda kv: kv[1], reverse=True)
            table = " | ".join(f"{a}:{sc:.0f}" for a, sc in ordered)
            print(f"[DBG] scores → {table}")

    # ------------------------------------------------------------------
    def _maybe_switch(self):
        if not self.adapter_best_score:
            return

        best_adapter, best_score = max(self.adapter_best_score.items(), key=lambda kv: kv[1])
        now = time.time()

        if self.current_adapter is None:
            self._do_switch(best_adapter, reason="initial pick")
            return

        if best_adapter == self.current_adapter:
            return

        cur_score = self.adapter_best_score.get(self.current_adapter, -float("inf"))
        if cur_score == -float("inf"):
            improved = True
            rssi_delta = self.rssi_hysteresis  # force switch
        else:
            cur_rssi = -(cur_score // 10_000)
            best_rssi = -(best_score // 10_000)
            rssi_delta = best_rssi - cur_rssi
            improved = rssi_delta >= self.rssi_hysteresis

        long_enough = (now - self.last_switch_time) >= self.min_switch_interval

        if improved and long_enough:
            self._do_switch(best_adapter, reason=f"improved by {rssi_delta} dB")

    # ------------------------------------------------------------------
    def _do_switch(self, adapter_id: str, *, reason: str):
        # Always print ranking first (useful even if not verbose)
        ordered = sorted(self.adapter_best_score.items(), key=lambda kv: kv[1], reverse=True)
        ranking_str = " | ".join(f"{a}:{sc:.0f}" for a, sc in ordered)
        print(f"[SWITCH] ranking → {ranking_str}")

        if self.verbose:
            print(f"[DBG] switching to {adapter_id} – {reason}")

        self.current_adapter = adapter_id
        self.last_switch_time = time.time()

        cmd = (
            self.switch_cmd.replace("{adapter}", adapter_id)
            if self.switch_cmd
            else f"SWITCH {adapter_id}"
        )

        if self.switch_cmd:
            try:
                subprocess.run(cmd, shell=True, check=True)
            except subprocess.CalledProcessError as e:
                print(f"[ERR] switch‑cmd failed: {e}", file=sys.stderr)
        else:
            print(cmd)

###############################################################################
# CLI & bootstrap
###############################################################################

def parse_args():
    p = argparse.ArgumentParser(description="Adaptive antenna/adapter selector")
    p.add_argument("--host", default="127.0.0.1", help="receiver host (default 127.0.0.1)")
    p.add_argument("--port", type=int, default=9500, help="receiver port (default 9500)")
    p.add_argument("--tx-antennas", required=True, help="comma‑separated list of adapter ids (14 hex chars) that are TX capable")
    p.add_argument("--switch-cmd", default="", help="shell command to perform the switch. Use {adapter} placeholder. If omitted, command is printed.")
    p.add_argument("--min-switch-interval", type=float, default=1.0, help="minimum seconds between two switch commands (default 1.0)")
    p.add_argument("--rssi-hysteresis", type=float, default=3.0, help="required rssi_avg improvement in dB to trigger a switch (default 3 dB)")
    p.add_argument("-v", "--verbose", action="store_true", help="enable verbose/debug output")
    return p.parse_args()


def main():
    args = parse_args()
    tx_list = [a.strip().lower() for a in args.tx_antennas.split(",") if a.strip()]
    if not tx_list:
        sys.exit("tx-antennas list is empty – nothing to do")

    sel = AntennaSelector(
        host=args.host,
        port=args.port,
        tx_adapters=tx_list,
        switch_cmd=args.switch_cmd,
        min_switch_interval=args.min_switch_interval,
        rssi_hysteresis=args.rssi_hysteresis,
        verbose=args.verbose,
    )
    sel.run_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[info] interrupted by user – exiting.")
