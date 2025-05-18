#!/usr/bin/env python3
"""
Fetch metrics from http://192.168.2.20:5010/metrics every 1.5 seconds,
select the best antenna per adapter from the latest RX_ANT timestamp,
and write a compact row to /run/pixelpilot.msg:
  RSSI1/RSSI2/RSSI3 PKT1/PKT2/PKT3 COUNT_P_UNIQ/COUNT_P_FEC_RECOVERED/COUNT_P_LOST
"""

import requests
import time

URL = "http://192.168.2.20:5010/metrics"
OUTPUT_FILE = "/run/pixelpilot.msg"
SLEEP_INTERVAL = 1.5

def fetch_metrics():
    """Fetch JSON metrics from the server."""
    r = requests.get(URL, timeout=5)
    r.raise_for_status()
    return r.json()

def get_latest_entries(entries):
    """Return a list of entries with the maximum 'ts' value."""
    if not entries:
        return []
    max_ts = max(entry['ts'] for entry in entries)
    return [entry for entry in entries if entry['ts'] == max_ts]

def group_by_adapter(rx_ants):
    """Group antenna entries by adapter prefix (all but last two hex chars)."""
    groups = {}
    for ant in rx_ants:
        adapter_id = ant['antenna_id'][:-2]
        groups.setdefault(adapter_id, []).append(ant)
    return groups

def select_best_antenna(ants):
    """
    Select the antenna with highest average SNR.
    If all SNR==0, fallback to highest RSSI.
    """
    best = max(ants, key=lambda a: a['snr'].get('snr_avg', 0))
    if best['snr'].get('snr_avg', 0) == 0:
        best = max(ants, key=lambda a: a['rssi'].get('rssi_avg', -9999))
    return best

def format_line(metrics):
    """Produce the compact output line from the metrics JSON."""
    # Get latest RX_ANT entries
    rx_all = metrics['video_agg_rx']['RX_ANT']
    rx_latest = get_latest_entries(rx_all)

    # Group antennas and pick best per adapter
    grouped = group_by_adapter(rx_latest)
    bests = [select_best_antenna(ants) for _, ants in sorted(grouped.items())]

    # Build RSSI and packet_count lists
    rssis = [str(b['rssi']['rssi_avg']) for b in bests]
    pkts = [str(b['packet_count']) for b in bests]

    # Get latest PKT entry
    pkt_all = metrics['video_agg_rx']['PKT']
    pkt = max(pkt_all, key=lambda p: p['ts'])

    # Extract packet-level aggregates
    uniq = pkt['count_p_uniq']
    fec  = pkt['count_p_fec_recovered']
    lost = pkt['count_p_lost']

    return f"{'/'.join(rssis)} {'/'.join(pkts)} {uniq}/{fec}/{lost}"

def main():
    """Main loop: fetch, format, write, and sleep."""
    while True:
        try:
            metrics = fetch_metrics()
            line = format_line(metrics)
            with open(OUTPUT_FILE, "w") as f:
                f.write(line)
        except Exception:
            # Optionally log errors if desired
            pass
        time.sleep(SLEEP_INTERVAL)

if __name__ == "__main__":
    main()

