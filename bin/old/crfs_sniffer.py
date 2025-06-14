#!/usr/bin/env python3
"""
crsf_rx.py – simple Crossfire / CRSF UART sniffer
=================================================

*   Shows **RC packets**, **link-stats** and now **common telemetry frames**
    (GPS, battery, vario, attitude, flight-mode).

*   Defaults: `/dev/ttyUSB0` @ **416 000 baud** (8-N-1).

Usage
-----
$ python3 crsf_rx.py                               # summaries only
$ python3 crsf_rx.py --port /dev/ttyACM0 -r        # raw hex dump of every frame
"""

from __future__ import annotations
import argparse
import signal
import struct
import sys
import time
from typing import List

import serial               # pip install pyserial

# --------------------------------------------------------------------------- constants
SYNC_BYTES = {0xC8, 0xEC, 0xEE}   # RX, TX and FC addresses that begin every frame
CRC_POLY = 0xD5

# frame-type IDs -------------------------------------------------------------
FT_RC_CHANNELS   = 0x16
FT_LINK_STATS    = 0x14
FT_GPS           = 0x02
FT_VARIO         = 0x07
FT_BATT          = 0x08
FT_ATTITUDE      = 0x1E
FT_FLIGHT_MODE   = 0x21
# 0x28–0x32 are “device/parameter” system frames (left as generic)

# --------------------------------------------------------------------------- helpers
def crc8(data: bytes) -> int:
    """CRC-8 using polynomial 0xD5 (same as CRSF spec)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_rc(payload: bytes) -> List[int]:
    """22-byte packed RC payload → list[16] of 0-2047 ticks."""
    if len(payload) != 22:
        raise ValueError("RC payload length must be 22 bytes")
    bits = int.from_bytes(payload, "little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


# --------------------------------------------------------------------------- sniffer
class CrsfSniffer:
    POLL_SLEEP = 0.005        # 5 ms pause when idle

    def __init__(self, port: str, baud: int, raw: bool):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,                         # fully non-blocking
        )
        self.raw = raw
        self.buf = bytearray()

        signal.signal(signal.SIGPIPE, signal.SIG_DFL)  # honour downstream close
        print(f"Listening on {self.ser.port} @ {self.ser.baudrate} baud  raw={self.raw}", flush=True)

    # ----------------------------------------------------------------------
    def run(self):
        try:
            while True:
                self._pump()
                time.sleep(self.POLL_SLEEP)
        except KeyboardInterrupt:
            pass

    # ----------------------------------------------------------------------
    def _pump(self):
        self.buf.extend(self.ser.read(256))     # grab whatever is available

        # keep buffer bounded & synced
        if len(self.buf) > 512:
            self._resync()

        # decode as many whole frames as possible
        while True:
            frame = self._try_extract_one()
            if not frame:
                break
            self._handle_frame(*frame)

    # ----------------------------------------------------------------------
    def _resync(self):
        while self.buf and self.buf[0] not in SYNC_BYTES:
            self.buf.pop(0)

    def _try_extract_one(self):
        if len(self.buf) < 2 or self.buf[0] not in SYNC_BYTES:
            self._resync()
            return None

        length = self.buf[1]
        total = length + 2                      # addr + len + payload+crc
        if len(self.buf) < total:
            return None                         # not enough yet

        frame = bytes(self.buf[:total])
        del self.buf[:total]

        if crc8(frame[2:-1]) != frame[-1]:
            return None                         # bad CRC → drop silently

        addr   = frame[0]
        ftype  = frame[2]
        payload = frame[3:-1]
        return addr, ftype, payload

    # ----------------------------------------------------------------------
    def _handle_frame(self, addr: int, ftype: int, payload: bytes):
        ts = int(time.time() * 1000)

        # ----- raw dump ----------------------------------------------------
        if self.raw:
            print(f"{ts} 0x{addr:02X} len={len(payload)+2} type=0x{ftype:02X} "
                  f"payload={payload.hex()} crcOK", flush=True)
            return

        # ----- human-friendly summaries -----------------------------------
        if ftype == FT_RC_CHANNELS:
            ch = unpack_rc(payload)
            print(f"{ts} RC {','.join(map(str, ch))}", flush=True)

        elif ftype == FT_LINK_STATS and len(payload) == 10:
            rssi, rem_rssi, tx_pw, snr, rem_snr, rf_mode = payload[:6]
            print(f"{ts} LINK rssi={rssi} rem={rem_rssi} tx_pw={tx_pw} "
                  f"snr={snr} rem_snr={rem_snr} mode={rf_mode}", flush=True)

        elif ftype == FT_GPS and len(payload) == 15:
            lat, lon, gs, hdg, alt, sats = struct.unpack("<llHHHB", payload)
            print(f"{ts} GPS lat={lat/1e7:.7f} lon={lon/1e7:.7f} "
                  f"gs={gs/100:.2f}m/s hdg={hdg/100:.2f}° alt={alt/100:.2f}m sats={sats}",
                  flush=True)

        elif ftype == FT_VARIO and len(payload) == 2:
            (climb,) = struct.unpack("<h", payload)
            print(f"{ts} VARIO {climb/100:.2f}m/s", flush=True)

        elif ftype == FT_BATT and len(payload) == 8:
            v, i, cap_lsb, cap_mid, cap_msb, rem = struct.unpack("<HHBBB", payload)
            mah = cap_lsb | (cap_mid << 8) | (cap_msb << 16)
            print(f"{ts} BATT {v/100:.2f}V {i/100:.2f}A {mah}mAh {rem}%", flush=True)

        elif ftype == FT_ATTITUDE and len(payload) == 6:
            pitch, roll, yaw = struct.unpack("<hhh", payload)
            print(f"{ts} ATT {pitch/100:.2f} {roll/100:.2f} {yaw/100:.2f} deg", flush=True)

        elif ftype == FT_FLIGHT_MODE:           # variable-length ASCII
            mode = payload.decode(errors="ignore").rstrip("\0")
            print(f"{ts} MODE {mode}", flush=True)

        else:                                   # catch-all
            print(f"{ts} FRAME addr=0x{addr:02X} type=0x{ftype:02X} len={len(payload)}",
                  flush=True)


# --------------------------------------------------------------------------- CLI
def parse_args():
    p = argparse.ArgumentParser(description="Tiny Crossfire (CRSF) UART sniffer.")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial device (default /dev/ttyUSB0)")
    p.add_argument("--baud", default=416000, type=int, help="Baud rate (default 416000)")
    p.add_argument("-r", "--raw", action="store_true",
                   help="Dump every valid frame verbatim in hex")
    return p.parse_args()


def main():
    args = parse_args()
    sniffer = CrsfSniffer(args.port, args.baud, args.raw)
    sniffer.run()


if __name__ == "__main__":
    main()
