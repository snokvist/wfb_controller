#!/usr/bin/env python3
"""
Continuously read EdgeTX serial output and, once per cycle, bump G-VAR 0 (FM0)
by +1, wrapping in the range [-1024 … +1024].

Channel‐dump script on the radio must already be running.
"""

import sys
import time
import serial           # pip install pyserial

PORT  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD  = int(sys.argv[2]) if len(sys.argv) > 2 else 115_200
MINV, MAXV = -1024, 1024          # G-VAR legal limits
STEP_INTERVAL = 1.0               # seconds

def main() -> None:
    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
            print(f"[INFO] Connected to {PORT} @ {BAUD} baud")
            gval = MINV
            next_send = time.time()

            while True:
                # 1) dump any incoming lines (non-blocking)
                try:
                    line = ser.readline().decode(errors='replace').strip()
                    if line:
                        print(f"<< {line}")
                except serial.SerialException as e:
                    print(f"[WARN] read error: {e}")

                # 2) time to send the next GV command?
                now = time.time()
                if now >= next_send:
                    cmd = f"GV,0,{gval}\n"
                    try:
                        ser.write(cmd.encode())
                        print(f">> {cmd.strip()}")
                    except serial.SerialException as e:
                        print(f"[WARN] write error: {e}")

                    # increment and wrap
                    gval = gval + 1 if gval < MAXV else MINV
                    next_send = now + STEP_INTERVAL
    except KeyboardInterrupt:
        print("\n[INFO] Terminated by user")
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {PORT}: {e}")

if __name__ == "__main__":
    main()
