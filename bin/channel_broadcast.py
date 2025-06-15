#!/usr/bin/env python3
import http.server
import socketserver
import subprocess
import re

PORT       = 8000
INTERFACE  = "service2"
FREQ_BASE  = 5000            # MHz

# ───────────────────────────────── helpers ────────────────────────────────────
def current_channel_blob() -> bytes:
    """Return `channel=…;width=…;region=…\n` as bytes, or raise RuntimeError."""
    iw  = subprocess.check_output(["iw", "dev"]).decode()
    reg = subprocess.check_output(["iw", "reg", "get"]).decode()

    # --- parse `iw dev` block for this interface ---
    m = re.search(
        rf"Interface {INTERFACE}.*?channel (\d+).*?width:\s*(\d+).*?center1:\s*(\d+)",
        iw, re.S)
    if not m:
        raise RuntimeError(f"interface {INTERFACE!r} not found in `iw dev` output")
    chan, width_raw, center1 = map(int, m.groups())
    freq = FREQ_BASE + 5 * chan

    width = {
        20:  "HT20", 80:  "80MHz", 160: "160MHz",
        10:  "10MHz", 5: "5MHz"
    }.get(
        width_raw,
        "HT40+" if width_raw == 40 and center1 > freq else
        "HT40-" if width_raw == 40 else
        "HT20"
    )

    region = re.search(r"country\s+([A-Z]{2}):", reg).group(1)
    return f"channel={chan};width={width};region={region}\n".encode()

# ───────────────────────────── HTTP request handler ───────────────────────────
class ChannelHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/channel.txt":
            try:
                blob = current_channel_blob()
            except Exception as exc:
                self.send_error(500, explain=str(exc))
                return

            self.send_response(200)
            self.send_header("Content-Type",  "text/plain")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
        else:
            super().do_GET()        # normal static-file handling

    def log_message(self, *args):    # make output quieter (optional)
        return                      # comment out to get default logging

# TCPServer subclass that immediately re-binds to the same port
class ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True

# ──────────────────────────────── main entrypoint ─────────────────────────────
if __name__ == "__main__":
    print(f"[HTTP] Serving dynamic /channel.txt on :{PORT} for interface={INTERFACE}")
    with ReusableTCPServer(("", PORT), ChannelHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[HTTP] Caught Ctrl+C → shutting down…")
            httpd.shutdown()        # break serve_forever() loop
    print("[HTTP] Server closed. Bye!")
