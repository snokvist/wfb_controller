#!/usr/bin/env python3
"""
Thread-Manager Live Monitor  (v5.0)

* Optional --ui  path/to/webui.html   (served at '/')
* Optional --stream-cmd "stream <thread> stdout"
"""
import argparse, socket, threading, time, re, pathlib
from collections import defaultdict, deque
from typing import Dict, Deque, Any
from flask import Flask, jsonify, request, Response, abort, send_file
from typing import Optional

# ───────── regex helpers (unchanged) ─────────
_RX_ANT_RE = re.compile(
    r"^(?P<thread>[\w\-]+):(?P<ts>\d+)\s+RX_ANT\s+"
    r"(?P<freq>\d+):(?P<mcs>\d+):(?P<bw>\d+)\s+"
    r"(?P<ant>[0-9a-fA-F]+)\s+"
    r"(?P<count>\d+):(?P<rssi_min>-?\d+):(?P<rssi_avg>-?\d+):(?P<rssi_max>-?\d+):"
    r"(?P<snr_min>-?\d+):(?P<snr_avg>-?\d+):(?P<snr_max>-?\d+)")
_TX_ANT_RE = re.compile(
    r"^(?P<thread>[\w\-]+):(?P<ts>\d+)\s+TX_ANT\s+"
    r"(?P<ant>[0-9a-fA-F]+)\s+"
    r"(?P<p_inj>\d+):(?P<p_drop>\d+):"
    r"(?P<lat_min>\d+):(?P<lat_avg>\d+):(?P<lat_max>\d+)")
_PKT_RE = re.compile(
    r"^(?P<thread>[\w\-]+):(?P<ts>\d+)\s+PKT\s+"
    r"(?P<vals>(?:\d+:){10}\d+)$")
PKT_FIELDS = [
    "count_p_all","count_b_all","count_p_dec_err",
    "count_p_session","count_p_data","count_p_uniq",
    "count_p_fec_recovered","count_p_lost","count_p_bad",
    "count_p_outgoing","count_b_outgoing",
]

# ───────── in-memory store (unchanged) ────────
BUFFER = 200
_metrics_lock = threading.Lock()
_metrics: Dict[str, Dict[str, Deque[Dict[str, Any]]]] = defaultdict(
    lambda: {"RX_ANT": deque(maxlen=BUFFER),
             "TX_ANT": deque(maxlen=BUFFER),
             "PKT":    deque(maxlen=BUFFER)}
)
def _store(thread:str, kind:str, data:Dict[str,Any]):  # noqa
    with _metrics_lock:
        _metrics[thread][kind].append(data)

# ─────────── streaming client ───────────
class StreamClient(threading.Thread):
    def __init__(self, host:str, port:int, stream_cmd:str, retry:int=5):
        super().__init__(daemon=True)
        self.h,self.p,self.cmd,self.r = host,port,stream_cmd,retry
    def run(self):
        while True:
            try: self._once()
            except Exception as e: print("[Stream]",e)
            time.sleep(self.r)
    def _once(self):
        with socket.create_connection((self.h,self.p),10) as s:
            f=s.makefile("r",encoding="ascii",newline="\n")
            s.sendall((self.cmd+"\n").encode()); f.readline()  # banner / ack
            for ln in f:
                if ln.strip(): self._parse(ln.rstrip("\n"))
    def _parse(self,line:str):
        if (m:=_RX_ANT_RE.match(line)):
            _store(m["thread"],"RX_ANT",{
                "ts":int(m["ts"]), "freq":int(m["freq"]),"mcs":int(m["mcs"]),"bw":int(m["bw"]),
                "antenna_id":f"0x{int(m['ant'],16):016x}","packet_count":int(m["count"]),
                "rssi":{k:int(m[k]) for k in ("rssi_min","rssi_avg","rssi_max")},
                "snr": {k:int(m[k]) for k in ("snr_min","snr_avg","snr_max")}}); return
        if (m:=_TX_ANT_RE.match(line)):
            _store(m["thread"],"TX_ANT",{
                "ts":int(m["ts"]),"antenna_id":f"0x{int(m['ant'],16):016x}",
                "pkt_injected":int(m["p_inj"]),"pkt_dropped":int(m["p_drop"]),
                "latency":{k:int(m[k]) for k in ("lat_min","lat_avg","lat_max")}}); return
        if (m:=_PKT_RE.match(line)):
            vals=list(map(int,m["vals"].split(":")))
            if len(vals)==11:_store(m["thread"],"PKT",
                {"ts":int(m["ts"]),**dict(zip(PKT_FIELDS,vals))})

# ───── one-shot command helpers (unchanged) ────
def tm_request(host:str,port:int,cmd:str)->str:
    try:
        with socket.create_connection((host,port),5) as s:
            s.sendall((cmd+"\n").encode())
            return s.recv(4096).decode(errors='ignore').strip()
    except Exception as e: return f"ERROR: {e}"
def list_commands(host,port):
    return [l for l in tm_request(host,port,"list commands").splitlines() if l]
    
def list_settings(host: str, port: int) -> Dict[str, str]:
    """
    Ask the thread-manager for `list settings`, and return
    a dict mapping each placeholder → its current value.
    """
    out = tm_request(host, port, "list settings")
    settings = {}
    for line in out.splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        # split only on first “:”
        if ":" in line:
            key, val = line.split(":", 1)
            settings[key.strip()] = val.strip()
    return settings

# ───────────────── flask app ────────────────
def create_app(host: str, port: int, ui_path: Optional[pathlib.Path]) -> Flask:
    app=Flask(__name__, static_folder=None)  # disable default /static

    @app.route("/metrics")
    def metrics() -> Response:
        with _metrics_lock:
            return jsonify({t:{k:list(d) for k,d in v.items()} for t,v in _metrics.items()})

    @app.route("/commands")
    def commands_api() -> Response:
        return jsonify(list_commands(host,port))
        
    @app.route("/settings")
    def settings_api() -> Response:
        """
        Return current TM settings as JSON:
          { "passphrase": "openipc", "private_key_b64": "UVsWvM8P…", … }
        """
        return jsonify(list_settings(host, port))

    @app.route("/exec", methods=["POST"])
    def exec_api() -> Response:
        cmd = request.json.get("cmd")
        if not cmd: abort(400)
        return jsonify({"output": tm_request(host,port,f"exec {cmd}")})

    # ---------------- serve UI ----------------
    if ui_path:
        ui_path = ui_path.expanduser().resolve()
        if not ui_path.is_file():
            raise FileNotFoundError(ui_path)
        @app.route("/")
        def index_file():
            return send_file(ui_path)
    else:
        # built-in page (identical to v4.5 HTML/JS)
        from textwrap import dedent
        INDEX_HTML = dedent("""\
        <!doctype html><html lang=en><head>
        <!--  identical HTML/JS from v4.5  -->
        </head></html>""")
        @app.route("/")
        def index_inline():
            return INDEX_HTML

    return app

# ─────────────── main ────────────────
def main():
    p=argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--host", default="192.168.2.20", help="Thread-Manager IP")
    p.add_argument("--port", type=int, default=9500, help="Thread-Manager TCP port")
    p.add_argument("--web-port", type=int, default=5000, help="Flask UI port")
    p.add_argument("--stream-cmd", default="list stream all",
                   help='Command sent after connect (e.g. "stream video_agg_rx stdout")')
    p.add_argument("--ui", type=pathlib.Path,
                   help="External HTML/JS file to serve instead of built-in UI")
    a=p.parse_args()

    StreamClient(a.host, a.port, a.stream_cmd).start()
    create_app(a.host, a.port, a.ui).run(host="0.0.0.0", port=a.web_port, threaded=True)

if __name__=="__main__":
    main()
