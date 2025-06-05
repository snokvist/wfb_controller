#!/usr/bin/env python3
"""
Adaptive-link ground-station helper (raw-stream, *no config file*)
=================================================================

* Connects to the TCP stats stream (`PKT` followed by `RX_ANT` lines).
* Computes link quality and pushes results via UDP.
* **All tunables are now hard-coded in this file** – no `.conf` file is
  read or written.

Change whatever values you like directly in the **CONSTANTS** section
below and the new numbers take effect the next time you run the script.
"""

from __future__ import annotations

import argparse
import random
import re
import socket
import struct
import time
from typing import Dict, List, Optional

# ────────────────────────────────────────────────────────────────────────────
# CONSTANTS – edit to taste                                                   |
# ────────────────────────────────────────────────────────────────────────────
# Outgoing UDP
UDP_IP           = "10.5.0.10"
UDP_PORT         = 9999

# Weights applied to normalised metrics
SNR_WEIGHT       = 0.5
RSSI_WEIGHT      = 0.5

# Normalisation ranges
SNR_MIN          = 30          # dB
SNR_MAX          = 38
RSSI_MIN         = -75         # dBm
RSSI_MAX         = 20

# Key-frame injection
ALLOW_IDR        = True
IDR_MAX_MESSAGES = 20          # number of consecutive UDP messages to append the key-frame code

# Dynamic refinement
ALLOW_PENALTY        = True
ALLOW_FEC_INCREASE   = True

# Noise → penalty mapping
MIN_NOISE                     = 0.01
MAX_NOISE                     = 0.10
DEDUCTION_EXPONENT            = 0.5
MIN_NOISE_FOR_FEC_CHANGE      = 0.01
NOISE_FOR_MAX_FEC_CHANGE      = 0.10

# Kalman filter (noise smoothing)
KALMAN_ESTIMATE_INITIAL       = 0.005
KALMAN_ERROR_ESTIMATE_INITIAL = 0.1
PROCESS_VARIANCE              = 1e-5
MEASUREMENT_VARIANCE          = 0.01

# Raw-stream endpoint
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9500
START_CMD    = b"stream video_agg_rx stdout\n"

# ────────────────────────────────────────────────────────────────────────────
# Global state                                                               |
# ────────────────────────────────────────────────────────────────────────────
best_rssi: int = -101
best_snr: int = 0
all_packets: int = 0
fec_rec_packets: int = 0
lost_packets: int = 0
num_antennas: int = 0

final_score: float = 1000
penalty: float = 0
fec_change: int = 0

keyframe_request_code: Optional[str] = None
keyframe_request_remaining: int = 0

kalman_estimate   = KALMAN_ESTIMATE_INITIAL
kalman_error_estimate = KALMAN_ERROR_ESTIMATE_INITIAL

verbose_mode: bool = False
udp_socket        = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ────────────────────────────────────────────────────────────────────────────
# UDP helper                                                                 |
# ────────────────────────────────────────────────────────────────────────────
def send_udp(msg: str) -> None:
    body = msg.encode()
    udp_socket.sendto(struct.pack("!I", len(body)) + body, (UDP_IP, UDP_PORT))
    if verbose_mode:
        print(f"\nUDP → {UDP_IP}:{UDP_PORT}  {msg}\n")

# ────────────────────────────────────────────────────────────────────────────
# Kalman filter                                                              |
# ────────────────────────────────────────────────────────────────────────────
def kalman_update(measurement: float) -> float:
    global kalman_estimate, kalman_error_estimate
    pred_est = kalman_estimate
    pred_err = kalman_error_estimate + PROCESS_VARIANCE
    gain     = pred_err / (pred_err + MEASUREMENT_VARIANCE)
    kalman_estimate       = pred_est + gain * (measurement - pred_est)
    kalman_error_estimate = (1 - gain) * pred_err
    return kalman_estimate

# ────────────────────────────────────────────────────────────────────────────
# Scoring logic                                                              |
# ────────────────────────────────────────────────────────────────────────────
def calculate_link() -> None:
    global final_score, penalty, fec_change
    global keyframe_request_code, keyframe_request_remaining

    # Key-frame request trigger
    if lost_packets > 0 and ALLOW_IDR and keyframe_request_remaining == 0:
        keyframe_request_code      = ''.join(random.choices('abcdefghijklmnopqrstuvwxyz', k=4))
        keyframe_request_remaining = IDR_MAX_MESSAGES
        if verbose_mode:
            print(f"Generated key-frame code {keyframe_request_code}")

    # Noise ratio (5× lost + fec-recovered) / packets-per-antenna
    if all_packets == 0 or num_antennas == 0:
        noise_ratio = 0.0
    else:
        noise_ratio = (5*lost_packets + fec_rec_packets) / (all_packets / num_antennas)

    filtered_noise = kalman_update(noise_ratio)

    # Normalise SNR & RSSI
    snr_norm  = max(0.0, min(1.0, (best_snr  - SNR_MIN)  / (SNR_MAX  - SNR_MIN)))
    rssi_norm = max(0.0, min(1.0, (best_rssi - RSSI_MIN) / (RSSI_MAX - RSSI_MIN)))
    score_norm = SNR_WEIGHT * snr_norm + RSSI_WEIGHT * rssi_norm
    raw_score  = 1000 + score_norm * 1000

    # Penalty from noise
    if not ALLOW_PENALTY or filtered_noise <= MIN_NOISE:
        penalty = 0.0
    else:
        ratio   = min(((filtered_noise - MIN_NOISE) / (MAX_NOISE - MIN_NOISE)) ** DEDUCTION_EXPONENT, 1.0)
        penalty = (raw_score - 1000) * ratio

    final_score = raw_score - penalty

    # FEC change advice
    if not ALLOW_FEC_INCREASE or filtered_noise <= MIN_NOISE_FOR_FEC_CHANGE:
        fec_change = 0
    elif filtered_noise >= NOISE_FOR_MAX_FEC_CHANGE:
        fec_change = 5
    else:
        fec_change = int(round(((filtered_noise - MIN_NOISE_FOR_FEC_CHANGE) /
                                (MAX_NOISE - MIN_NOISE_FOR_FEC_CHANGE)) * 5))

    if verbose_mode:
        print(f"noise={noise_ratio:.3f} filt={filtered_noise:.3f} "
              f"rssi={best_rssi} snr={best_snr} score={final_score:.1f}")

    send_udp_message()

def send_udp_message() -> None:
    global keyframe_request_code, keyframe_request_remaining
    ts  = int(time.time())
    msg = f"{ts}:{int(final_score)}:{int(final_score)}:{fec_rec_packets}:{lost_packets}:{best_rssi}:{best_snr}:{num_antennas}:{int(penalty)}:{fec_change}"
    if keyframe_request_code and keyframe_request_remaining > 0:
        msg += f":{keyframe_request_code}"
        keyframe_request_remaining -= 1
        if keyframe_request_remaining == 0:
            keyframe_request_code = None
    send_udp(msg)

# ────────────────────────────────────────────────────────────────────────────
# Raw-stream parsing                                                          |
# ────────────────────────────────────────────────────────────────────────────
RXANT_RE = re.compile(r"""
    ^[^:]+:\d+\s+RX_ANT\s+
    \d+:\d+:\d+\s+[0-9a-fA-F]+\s+
    (?P<vals>(?:-?\d+:?)+)$
""", re.VERBOSE)

PKT_RE = re.compile(r"""
    ^[^:]+:\d+\s+PKT\s+
    (?P<vals>(?:\d+:?)+)$
""", re.VERBOSE)

def parse_rx_ant(line: str) -> Optional[Dict]:
    m = RXANT_RE.match(line)
    if not m:
        return None
    v = list(map(int, m.group("vals").split(":")))
    return {"rssi_avg": v[2], "snr_avg": v[5]}

def parse_pkt(line: str) -> Optional[Dict]:
    m = PKT_RE.match(line)
    if not m:
        return None
    v = list(map(int, m.group("vals").split(":")))
    keys = ["p_all","b_all","p_dec_err","p_sess","p_data",
            "p_uniq","p_fec_rec","p_lost","p_bad","p_out","b_out"]
    return dict(zip(keys, v))

# ────────────────────────────────────────────────────────────────────────────
# TCP listener                                                                |
# ────────────────────────────────────────────────────────────────────────────
def connect_raw(host: str, port: int) -> socket.socket:
    s = socket.create_connection((host, port))
    s.sendall(START_CMD)
    print(f"Connected to raw stats {host}:{port}")
    return s

def finalize_tick(pkt: Dict, ants: List[Dict]) -> None:
    global best_rssi, best_snr, num_antennas
    global all_packets, fec_rec_packets, lost_packets

    num_antennas    = len(ants)
    best_rssi       = max((a["rssi_avg"] for a in ants), default=-101)
    best_snr        = max((a["snr_avg"]  for a in ants), default=0)
    all_packets     = pkt["p_all"]
    fec_rec_packets = pkt["p_fec_rec"]
    lost_packets    = pkt["p_lost"]

    calculate_link()

def listen_raw(sock: socket.socket) -> None:
    ant_buffer: List[Dict] = []
    current_pkt: Optional[Dict] = None

    with sock.makefile() as f:
        for line in f:
            if (pkt := parse_pkt(line)):
                if current_pkt is not None:                  # close previous block
                    finalize_tick(current_pkt, ant_buffer)
                current_pkt = pkt
                ant_buffer.clear()
                continue

            if (ant := parse_rx_ant(line)):
                ant_buffer.append(ant)

        # EOF – finalise last block if any
        if current_pkt is not None:
            finalize_tick(current_pkt, ant_buffer)

# ────────────────────────────────────────────────────────────────────────────
# Entry point                                                                 |
# ────────────────────────────────────────────────────────────────────────────
def main() -> None:
    global verbose_mode

    argp = argparse.ArgumentParser(description="alink_gs (raw-stream, no-config)")
    argp.add_argument('--verbose', action='store_true', help='Verbose logging')
    argp.add_argument('--host', default=DEFAULT_HOST, help='Stats TCP host')
    argp.add_argument('--port', type=int, default=DEFAULT_PORT, help='Stats TCP port')
    args = argp.parse_args()
    verbose_mode = args.verbose

    while True:
        sock = None
        try:
            sock = connect_raw(args.host, args.port)
            listen_raw(sock)
        except (OSError, ConnectionRefusedError) as e:
            print(f"Connection error: {e} – retrying in 3 s")
            time.sleep(3)
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

if __name__ == '__main__':
    main()
