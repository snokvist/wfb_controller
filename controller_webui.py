#!/usr/bin/env python3
"""
Thread-Manager live monitor – v2 plus (adds on-page PKT display)

Only change from your v2:
  • Front-end JS appends a second card with the latest PKT metrics
    for the same thread, right after the RX_ANT or TX_ANT card.
"""
import argparse, json, re, socket, threading, time
from collections import defaultdict, deque
from typing import Dict, Deque, Any

from flask import Flask, jsonify, Response

###############################################################################
# Parsing helpers – unchanged
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
    r"(?P<vals>(?:\d+:){10}\d+)$"
)
PKT_FIELD_NAMES = [
    "count_p_all", "count_b_all", "count_p_dec_err",
    "count_p_session", "count_p_data", "count_p_uniq",
    "count_p_fec_recovered", "count_p_lost", "count_p_bad",
    "count_p_outgoing", "count_b_outgoing",
]

###############################################################################
# In-memory store
###############################################################################
SAMPLES = 100
_metrics_lock = threading.Lock()
_metrics: Dict[str, Dict[str, Deque[Dict[str, Any]]]] = defaultdict(
    lambda: {"RX_ANT": deque(maxlen=SAMPLES),
             "TX_ANT": deque(maxlen=SAMPLES),
             "PKT":    deque(maxlen=SAMPLES)}
)

def _store(thread: str, kind: str, data: Dict[str, Any]) -> None:
    with _metrics_lock:
        _metrics[thread][kind].append(data)

###############################################################################
# Stream client – unchanged
###############################################################################
class StreamClient(threading.Thread):
    def __init__(self, host: str, port: int = 9500, reconnect_delay: int = 5):
        super().__init__(daemon=True)
        self.host, self.port, self.reconnect_delay = host, port, reconnect_delay

    def run(self) -> None:
        while True:
            try: self._once()
            except Exception as e: print("[StreamClient]", e)
            time.sleep(self.reconnect_delay)

    def _once(self) -> None:
        print(f"[StreamClient] connect {self.host}:{self.port}")
        with socket.create_connection((self.host, self.port), timeout=10) as s:
            f = s.makefile("r", encoding="ascii", newline="\n")
            s.sendall(b"list stream all\n")
            print("[StreamClient]", f.readline().strip())
            for line in f:
                if not line.strip(): continue
                self._parse(line.rstrip("\n"))

    def _parse(self, line: str) -> None:
        if (m := _RX_ANT_RE.match(line)):
            _store(m["thread"], "RX_ANT", {
                "ts": int(m["ts"]), "freq": int(m["freq"]),
                "mcs": int(m["mcs"]), "bw": int(m["bw"]),
                "antenna_id": int(m["ant"], 16),
                "packet_count": int(m["count"]),
                "rssi": {k:int(m[k]) for k in ("rssi_min","rssi_avg","rssi_max")},
                "snr":  {k:int(m[k]) for k in ("snr_min","snr_avg","snr_max")},
            }); return
        if (m := _TX_ANT_RE.match(line)):
            _store(m["thread"], "TX_ANT", {
                "ts": int(m["ts"]), "antenna_id": int(m["ant"],16),
                "pkt_injected": int(m["p_inj"]), "pkt_dropped": int(m["p_drop"]),
                "latency": {k:int(m[k]) for k in ("lat_min","lat_avg","lat_max")},
            }); return
        if (m := _PKT_RE.match(line)):
            vals = list(map(int, m["vals"].split(":")))
            if len(vals)==11:
                _store(m["thread"], "PKT",
                       {"ts": int(m["ts"]),
                        **dict(zip(PKT_FIELD_NAMES, vals))})
###############################################################################
# Flask app
###############################################################################
def create_app() -> Flask:
    app = Flask(__name__)

    @app.route("/metrics")
    def metrics() -> Response:
        with _metrics_lock:
            return jsonify({t:{k:list(d) for k,d in v.items()} for t,v in _metrics.items()})

    INDEX_HTML = r"""
<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Thread-Manager Live Metrics</title>
<style>
:root{--bg:#e8f0ff;--fg:#002f6c;--accent:#1976d2;--card:#fff}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{background:var(--accent);color:#fff;padding:1rem;text-align:center;font-weight:600}
.tabs{display:flex;position:sticky;top:0;z-index:1;background:var(--bg)}
.tab{flex:1;padding:.75rem 0;text-align:center;font-weight:600;border:none;background:#cde0ff;color:var(--fg);cursor:pointer;font-size:1rem}
.tab.active{background:#fff;border-bottom:3px solid var(--accent)}
section.tab-content{display:none;padding:1rem}section.tab-content.active{display:block}
.thread{margin:.5rem 0 .25rem;font-size:1.1rem;font-weight:600}
.card{background:var(--card);border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,.12);padding:.5rem .75rem;margin:.4rem 0;white-space:pre-wrap;font-family:monospace;font-size:.85rem;overflow-x:auto}
.label{color:var(--accent);font-weight:600}
@media(max-width:600px){.card{font-size:.78rem}}
</style></head><body>
<header>Thread-Manager Live Metrics</header>
<div class="tabs">
  <button class="tab active" data-tab="rx">RX</button>
  <button class="tab" data-tab="tx">TX</button>
</div>
<section id="rx" class="tab-content active"></section>
<section id="tx" class="tab-content"></section>
<script>
const tabs=[...document.querySelectorAll('.tab')];
tabs.forEach(btn=>btn.onclick=()=>{tabs.forEach(b=>b.classList.toggle('active',b===btn));
  document.querySelectorAll('.tab-content').forEach(sec=>sec.classList.toggle('active',sec.id===btn.dataset.tab));});

async function refresh(){
  const r=await fetch('/metrics'); if(!r.ok) return;
  const data=await r.json(); let rx='',tx='';
  for(const [thread,k] of Object.entries(data)){
    const pkt = k.PKT?.length ? k.PKT[k.PKT.length-1] : null;
    if(k.RX_ANT?.length){
      rx+=`<div class="thread">${thread}</div>`;
      rx+=`<div class="card"><span class="label">RX_ANT</span>\n${JSON.stringify(k.RX_ANT.at(-1),null,2)}</div>`;
      if(pkt) rx+=`<div class="card"><span class="label">PKT</span>\n${JSON.stringify(pkt,null,2)}</div>`;
    }
    if(k.TX_ANT?.length){
      tx+=`<div class="thread">${thread}</div>`;
      tx+=`<div class="card"><span class="label">TX_ANT</span>\n${JSON.stringify(k.TX_ANT.at(-1),null,2)}</div>`;
      if(pkt) tx+=`<div class="card"><span class="label">PKT</span>\n${JSON.stringify(pkt,null,2)}</div>`;
    }
  }
  document.getElementById('rx').innerHTML = rx || '<p>No RX data yet…</p>';
  document.getElementById('tx').innerHTML = tx || '<p>No TX data yet…</p>';
}
setInterval(refresh,1000); refresh();
</script></body></html>"""
    @app.route("/")
    def index() -> str:
        return INDEX_HTML
    return app

###############################################################################
def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=9500)
    p.add_argument("--web-port", type=int, default=5000)
    a = p.parse_args()
    StreamClient(a.host, a.port).start()
    create_app().run(host="0.0.0.0", port=a.web_port, threaded=True)

if __name__ == "__main__":
    main()
