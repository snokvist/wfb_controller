#!/usr/bin/env python3
import http.server, socketserver, subprocess, re

PORT = 8000
INTERFACE = "service2"
FREQ_BASE = 5000

def current_channel_blob() -> bytes:
    """Return the `channel=…;width=…;region=…\n` line as bytes."""
    iw   = subprocess.check_output(["iw", "dev"]).decode()
    reg  = subprocess.check_output(["iw", "reg", "get"]).decode()

    # --- parse iw dev ---
    m = re.search(rf"Interface {INTERFACE}.*?channel (\d+).*?width:\s*(\d+).*?center1:\s*(\d+)",
                  iw, re.S)
    if not m:
        raise RuntimeError("interface not found")
    chan, width_raw, center1 = map(int, m.groups())
    freq = FREQ_BASE + 5 * chan

    width = {20: "HT20", 80: "80MHz", 160: "160MHz", 10: "10MHz", 5: "5MHz"}.get(
        width_raw,
        "HT40+" if width_raw == 40 and center1 > freq else
        "HT40-" if width_raw == 40 else
        "HT20"
    )

    # --- parse reg domain ---
    region = re.search(r"country\s+([A-Z]{2}):", reg).group(1)

    return f"channel={chan};width={width};region={region}\n".encode()

class ChannelHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/channel.txt":
            try:
                blob = current_channel_blob()
            except Exception as exc:
                self.send_error(500, explain=str(exc))
                return

            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
        else:
            # Anything else is served off disk as usual
            super().do_GET()

if __name__ == "__main__":
    with socketserver.TCPServer(("", PORT), ChannelHandler) as httpd:
        print(f"Serving dynamic /channel.txt on :{PORT}")
        httpd.serve_forever()

