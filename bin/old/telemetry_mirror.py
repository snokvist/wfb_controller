#!/usr/bin/env python3
"""
simple_crsf.py – minimal CRSF frame reader.

 * Opens /dev/ttyACM0 at 115 200 baud, 8-N-1
 * Validates each frame with CRC-8 (poly 0xD5)
 * Prints:
      • Flight-mode text          (type 0x21)
      • Link statistics           (type 0x14)
      • Attitude (roll/pitch/yaw) (type 0x1E)
Adapt or extend as you wish; the hard part – pulling clean frames – is done.
"""

import serial
from collections import deque

PORT   = "/dev/ttyACM0"
BAUD   = 115_200
POLY   = 0xD5                 # CRC-8 polynomial (x⁸ + x⁷ + x² + x + 1)
SYNC   = 0xEA                 # “transmitter → host” address, always first byte

# ---------------------------------------------------------------------------

def crc8(buf: bytes) -> int:
    """CRSF CRC-8 (poly = 0xD5, init = 0, refin = false, refout = false)."""
    crc = 0
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ POLY) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def _to_int16(le_bytes):
    return int.from_bytes(le_bytes, 'little', signed=True)

def _to_int32(le_bytes):
    return int.from_bytes(le_bytes, 'little', signed=True)

def parse_frame(frame: bytes):
    ftype  = frame[0]
    pl     = frame[1:]

    # ─── 0x08 Battery sensor ───────────────────────────────────────────────
    if ftype == 0x08 and len(pl) >= 8:
        mv   = int.from_bytes(pl[0:2], 'little') / 100.0      # V
        ma   = int.from_bytes(pl[2:4], 'little') / 100.0      # A
        mah  = int.from_bytes(pl[4:7], 'little')              # mAh
        rem  =  pl[7]                                         # %
        print(f"BAT  : {mv:.2f} V  {ma:.1f} A  {mah} mAh  {rem}%")

    # ─── 0x14 Link statistics ──────────────────────────────────────────────
    elif ftype == 0x14 and len(pl) >= 10:
        rssi  = pl[0] - 256 if pl[0] > 127 else pl[0]         # signed dBm
        lq    = pl[2]                                         # %
        snr   = pl[4] - 256 if pl[4] > 127 else pl[4]         # dB
        rfmd  = pl[8]                                         # 0..3
        print(f"LINK : RSSI {rssi:+} dBm  LQ {lq}%  SNR {snr:+} dB  RFmode {rfmd}")

    # ─── 0x1E Attitude ─────────────────────────────────────────────────────
    elif ftype == 0x1E and len(pl) >= 6:
        roll, pitch, yaw = (_to_int16(pl[0:2])/100,
                            _to_int16(pl[2:4])/100,
                            _to_int16(pl[4:6])/100)
        print(f"ATT  : roll={roll:+.2f} rad pitch={pitch:+.2f} rad yaw={yaw:+.2f} rad")

    # ─── 0x21 Flight-mode text ─────────────────────────────────────────────
    elif ftype == 0x21:
        txt = pl.split(b'\0', 1)[0].decode(errors='replace')
        print("MODE :", txt)

    # ─── 0x02 GPS ───────────────────────────────────────────────────────────
    elif ftype == 0x02 and len(pl) >= 15:
        lat  = _to_int32(pl[0:4]) / 1e7
        lon  = _to_int32(pl[4:8]) / 1e7
        alt  = _to_int32(pl[8:12]) / 100.0          # from cm
        gspd = _to_int16(pl[12:14]) / 100.0         # m/s
        sats = pl[14]
        print(f"GPS  : {lat:.6f},{lon:.6f}  {alt:.1f} m  {gspd:.1f} m/s  {sats} sat")

    # ─── 0x07 Vario ────────────────────────────────────────────────────────
    elif ftype == 0x07 and len(pl) >= 2:
        vspd = _to_int16(pl[0:2]) / 100.0           # m/s
        print(f"VARIO: {vspd:+.2f} m/s")

    # ─── 0x09 Baro altitude ────────────────────────────────────────────────
    elif ftype == 0x09 and len(pl) >= 4:
        alt  = _to_int32(pl[0:4]) / 100.0           # m
        print(f"BARO : {alt:+.1f} m")

    # ─── 0x16 RC channels packed (first 4 sticks as example) ───────────────
    elif ftype == 0x16 and len(pl) >= 22:
        bits = int.from_bytes(pl[0:22], 'little')
        ch   = [((bits >> (11*i)) & 0x7FF) for i in range(16)]
        # scale 11-bit [172,1811] to µs:
        us   = [round((c * 1000 / 1024) + 988) for c in ch[:4]]
        print("CH1-4:", *us, "µs")

    # ─── 0x3A Radio-ID ─────────────────────────────────────────────────────
    elif ftype == 0x3A and len(pl) >= 6:
        dst, org = pl[0], pl[1]
        uid      = int.from_bytes(pl[2:6], 'little')
        print(f"RADIO: UID=0x{uid:08X}  dst={dst:#02x}  org={org:#02x}")

    # ─── 0x10 OpenTX sync & 0x0B Heartbeat (silence) ───────────────────────
    elif ftype in (0x10, 0x0B):
        pass  # nothing useful for humans

    else:
        print(f"*Unhandled* type 0x{ftype:02X} ({len(pl)} bytes)")


# ---------------------------------------------------------------------------

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    buf = deque()                       # we’ll pop from the left

    while True:
        buf.extend(ser.read(256))       # grab some bytes
        while True:                     # keep pulling frames out of buf
            # 1. seek sync/address byte
            try:
                idx = buf.index(SYNC)
            except ValueError:
                buf.clear()             # nothing useful yet
                break
            for _ in range(idx):
                buf.popleft()           # discard leading junk

            # Need at least address+len+CRC = 3 bytes
            if len(buf) < 3:
                break
            _, length = buf[0], buf[1]

            if length < 2 or length > 62:       # spec: max 62 (type+pl+CRC)
                buf.popleft()                   # bad len – resync
                continue
            if len(buf) < 2 + length:           # full frame not here yet
                break

            # 2 bytes header already present; grab complete frame
            header = [buf.popleft(), buf.popleft()]          # SYNC, LEN
            payload_crc = bytes(buf.popleft() for _ in range(length))
            data, rx_crc = payload_crc[:-1], payload_crc[-1]

            if crc8(data) != rx_crc:           # bad CRC → resync
                continue                       # (next loop iteration)

            parse_frame(data)                  # OK – handle it

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye!")

