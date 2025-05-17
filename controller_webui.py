#!/usr/bin/env python3
"""
Thread-Manager live monitor

* Connects to the manager on <host>:9500
* Sends 'list stream all' and stays on the socket forever
* Parses RX_ANT, TX_ANT and PKT lines
* Stores the last N samples in memory
* Serves a tiny Flask UI + JSON API for live inspection
"""
import argparse
import json
import re
import socket
import threading
import time
from collections import defaultdict, deque
from typing import Dict, Deque, Any

from flask import Flask, jsonify, Response

###############################################################################
# Parsing helpers
###############################################################################
_RX_ANT_RE = re.compile(
    r"^(?P<thread>[\w\-]+):(?P<ts>\d+)\s+RX_ANT\s+"
    r"(?P<freq>\d+):(?P<mcs>\d+):(?P<bw>\d+)\s+"
    r"(?P<ant>[0-9a-fA-F]+)\s+"
    r"(?P<count>\d+):(?P<rssi_min>-?\d+):(?P<rssi_avg>-?\d+):(?P<rssi_max>-?\d+):"
    r"(?P<snr_min>-?\d+):(?P<snr_avg>-?\d+):(?P<snr_max>-?\d+)"
)

_TX_ANT_RE = re.compile(
    r"^(?P<thread>[\w\-]+):(?P<ts>\d+)\s+TX_ANT\s+"
    r"(?P<ant>[0-9a-fA-F]+)\s+"
    r"(?P<p_inj>\d+):(?P<p_drop>\d+):"
    r"(?P<lat_min>\d+):(?P<lat_avg>\d+):(?P<lat_max>\d+)"
)

_PKT_RE = re.compile(
    r"^(?P<thread>[\w\-]+):(?P<ts>\d+)\s+PKT\s+"
    r"(?P<vals>(\d+:){10}\d+)$"
)

PKT_FIELD_NAMES = [
    "count_p_all", "count_b_all",
    "count_p_dec_err",
    "count_p_session", "count_p_data",
    "count_p_uniq",
    "count_p_fec_recovered", "count_p_lost",
    "count_p_bad",
    "count_p_outgoing", "count_b_outgoing",
]

###############################################################################
# Metrics store – in-memory, thread-safe
###############################################################################
SAMPLES_PER_TYPE = 100
_metrics_lock = threading.Lock()
_metrics: Dict[str, Dict[str, Deque[Dict[str, Any]]]] = defaultdict(
    lambda: {
        "RX_ANT": deque(maxlen=SAMPLES_PER_TYPE),
        "TX_ANT": deque(maxlen=SAMPLES_PER_TYPE),
        "PKT": deque(maxlen=SAMPLES_PER_TYPE),
    }
)


def _store(thread: str, entry_type: str, data: Dict[str, Any]) -> None:
    """Remember a parsed line."""
    with _metrics_lock:
        _metrics[thread][entry_type].append(data)


###############################################################################
# Thread-Manager streaming client
###############################################################################
class StreamClient(threading.Thread):
    """Background thread that maintains a persistent streaming connection."""

    def __init__(self, host: str, port: int = 9500, reconnect_delay: int = 5):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.reconnect_delay = reconnect_delay

    def run(self) -> None:
        while True:
            try:
                self._run_once()
            except Exception as exc:
                print(f"[StreamClient] connection error: {exc!r}")
            time.sleep(self.reconnect_delay)

    def _run_once(self) -> None:
        print(f"[StreamClient] connecting to {self.host}:{self.port} …")
        with socket.create_connection((self.host, self.port), timeout=10) as sock:
            sock_file = sock.makefile("r", encoding="ascii", newline="\n")
            # Start streaming
            sock.sendall(b"list stream all\n")
            first = sock_file.readline()
            print(f"[StreamClient] {first.strip()!r}")

            for line in sock_file:
                if not line.strip():          # blank line means "stop stream" (never seen here)
                    continue
                self._parse_line(line.rstrip("\n"))

    # --------------------------------------------------------------------- #
    def _parse_line(self, line: str) -> None:
        """Detect the line kind and push into the store."""
        m = _RX_ANT_RE.match(line)
        if m:
            _store(
                m["thread"],
                "RX_ANT",
                {
                    "ts": int(m["ts"]),
                    "freq": int(m["freq"]),
                    "mcs": int(m["mcs"]),
                    "bw": int(m["bw"]),
                    "antenna_id": int(m["ant"], 16),
                    "packet_count": int(m["count"]),
                    "rssi": {
                        "min": int(m["rssi_min"]),
                        "avg": int(m["rssi_avg"]),
                        "max": int(m["rssi_max"]),
                    },
                    "snr": {
                        "min": int(m["snr_min"]),
                        "avg": int(m["snr_avg"]),
                        "max": int(m["snr_max"]),
                    },
                },
            )
            return

        m = _TX_ANT_RE.match(line)
        if m:
            _store(
                m["thread"],
                "TX_ANT",
                {
                    "ts": int(m["ts"]),
                    "antenna_id": int(m["ant"], 16),
                    "pkt_injected": int(m["p_inj"]),
                    "pkt_dropped": int(m["p_drop"]),
                    "latency": {
                        "min": int(m["lat_min"]),
                        "avg": int(m["lat_avg"]),
                        "max": int(m["lat_max"]),
                    },
                },
            )
            return

        m = _PKT_RE.match(line)
        if m:
            values = list(map(int, m["vals"].split(":")))
            if len(values) == 11:
                _store(
                    m["thread"],
                    "PKT",
                    {"ts": int(m["ts"]), **dict(zip(PKT_FIELD_NAMES, values))},
                )
            return

        # Unknown line – optional: print or ignore
        # print(f"[StreamClient] unparsed: {line}")


###############################################################################
# Flask web-server
###############################################################################
def create_app() -> Flask:
    app = Flask(__name__)

    # ------------------------------------------------------------------ API #
    @app.route("/metrics")
    def api_metrics() -> Response:
        """Return the full, current metrics as JSON."""
        with _metrics_lock:
            serializable = {
                thread: {
                    typ: list(deq)  # convert deque → list for jsonify
                    for typ, deq in kinds.items()
                }
                for thread, kinds in _metrics.items()
            }
        return jsonify(serializable)

    # -------------------------------------------------------------- UI page #
    INDEX_HTML = """
    <!doctype html>
    <html lang="en">
      <head>
        <meta charset="utf-8">
        <title>Thread-Manager Live Metrics</title>
        <style>
          body{font-family:sans-serif;margin:1em;}
          pre{background:#f3f3f3;padding:.5em;border-radius:6px;}
        </style>
      </head>
      <body>
        <h1>Thread-Manager Live Metrics</h1>
        <p>Updated every second – data comes directly from <code>list stream all</code>.</p>
        <div id="content">Loading…</div>
        <script>
          async function refresh(){
            const r = await fetch('/metrics');
            const data = await r.json();
            let html = '';
            for(const [thread,kinds] of Object.entries(data)){
              html += `<h2>${thread}</h2>`;
              for(const [typ,arr] of Object.entries(kinds)){
                if(!arr.length) continue;
                html += `<h3>${typ} (latest)</h3><pre>` +
                        JSON.stringify(arr[arr.length-1], null, 2) +
                        '</pre>';
              }
            }
            document.getElementById('content').innerHTML = html || 'No data yet…';
          }
          setInterval(refresh, 1000);
          refresh();
        </script>
      </body>
    </html>
    """
    @app.route("/")
    def index() -> str:
        return INDEX_HTML

    return app


###############################################################################
# Main
###############################################################################
def main() -> None:
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1",
                        help="IP/hostname of the Thread-Manager")
    parser.add_argument("--port", type=int, default=9500,
                        help="TCP port of the Thread-Manager")
    parser.add_argument("--web-port", type=int, default=5000,
                        help="Port for the Flask UI")
    args = parser.parse_args()

    # Start the streaming client (daemon thread)
    StreamClient(args.host, args.port).start()

    # Start the Flask server (main thread, blocking)
    create_app().run(host="0.0.0.0", port=args.web_port, threaded=True)


if __name__ == "__main__":
    main()
