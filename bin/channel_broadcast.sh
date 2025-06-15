#!/usr/bin/env python3
import http.server
import socketserver
import subprocess
import time

PORT = 8000
INTERFACE = "service2"
FREQ_BASE = 5000

class ChannelHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/channel.txt":
            self.send_error(404)
            return

        try:
            iw_output = subprocess.check_output(["iw", "dev"]).decode()
            reg_output = subprocess.check_output(["iw", "reg", "get"]).decode()
        except subprocess.CalledProcessError:
            self.send_error(500)
            return

        # Extract block for interface
        block = ""
        found = False
        for line in iw_output.splitlines():
            if line.strip().startswith("Interface"):
                found = INTERFACE in line
            if found and "channel" in line:
                block = line.strip()
                break

        # Parse fields
        try:
            parts = block.split(',')
            chan_part = parts[0].split()
            chan = int(chan_part[1])  # e.g. "channel 104"
            width_raw = parts[1].split(':')[1].strip().split()[0]  # "40"
            center1 = int(parts[2].split(':')[1].strip().split()[0])  # "5530"
            freq = FREQ_BASE + 5 * chan
        except Exception:
            self.send_error(500)
            return

        # Determine width
        if width_raw == "20":
            width = "HT20"
        elif width_raw == "40":
            width = "HT40+" if center1 > freq else "HT40-"
        elif width_raw == "80":
            width = "80MHz"
        elif width_raw == "160":
            width = "160MHz"
        elif width_raw == "10":
            width = "10MHz"
        elif width_raw == "5":
            width = "5MHz"
        else:
            width = "HT20"

        # Region
        try:
            region_line = next(l for l in reg_output.splitlines() if "country" in l)
            region = region_line.split()[1].split(":")[0]
        except Exception:
            region = "US"

        # Output response
        response = f"channel={chan};width={width};region={region}\n"
        print(f"[HTTP] Served: {response.strip()}")

        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response.encode())

# Run server
if __name__ == "__main__":
    with socketserver.TCPServer(("", PORT), ChannelHandler) as httpd:
        print(f"[HTTP] Serving on port {PORT} — interface: {INTERFACE}")
        httpd.serve_forever()
