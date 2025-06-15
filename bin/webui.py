#!/usr/bin/env python3
"""
Thread-Manager Live Monitor (v5.3 “ini-commands”)

Changes vs v5.2
────────────────
•  Loads `[commands]` from /etc/commands.ini once at startup
   – `/commands` → list of keys  
   – `/exec`     → run mapped shell line, returns
     `{"stdout":…, "stderr":…, "code":…}` (404 if unknown)
•  New `/wifi_channel` endpoint  
   – Replies with `channel=…;width=…;region=…\n`
•  Removed external “settings” code + endpoint
•  Existing metrics/stream logic untouched
"""

import argparse, socket, threading, time, re, pathlib, json, subprocess, configparser
from collections import defaultdict, deque
from typing import Dict, Deque, Any, List, Optional
from flask import Flask, Response, request, abort, send_file

# ───────── load /etc/commands.ini ─────────
COMMANDS_INI = "/etc/commands.ini"
_ini = configparser.ConfigParser(interpolation=None)
_ini.read(COMMANDS_INI)
COMMANDS_MAP: Dict[str, str] = (
    dict(_ini.items("commands")) if _ini.has_section("commands") else {}
)

def run_command(key: str) -> Dict[str, Any]:
    """Run the shell line mapped to *key* via /bin/sh -c and return stdout/stderr/code."""
    cmd_line = COMMANDS_MAP[key]
    proc = subprocess.run(
        ["/bin/sh", "-c", cmd_line],
        capture_output=True, text=True
    )
    return {"stdout": proc.stdout, "stderr": proc.stderr, "code": proc.returncode}

# ───────── wifi-channel helper ─────────
INTERFACE  = "service2"
FREQ_BASE  = 5000          # MHz

def current_channel_blob() -> str:
    """Return `channel=…;width=…;region=…\\n` (raises on failure)."""
    iw  = subprocess.check_output(["iw", "dev"]).decode()
    reg = subprocess.check_output(["iw", "reg", "get"]).decode()

    m = re.search(
        rf"Interface {re.escape(INTERFACE)}.*?channel (\d+).*?width:\s*(\d+).*?center1:\s*(\d+)",
        iw, re.S
    )
    if not m:
        raise RuntimeError(f"interface {INTERFACE!r} not found in `iw dev` output")
    chan, width_raw, center1 = map(int, m.groups())
    freq = FREQ_BASE + 5 * chan

    width = {20:"HT20",80:"80MHz",160:"160MHz",10:"10MHz",5:"5MHz"}.get(
        width_raw,
        "HT40+" if width_raw == 40 and center1 > freq else
        "HT40-" if width_raw == 40 else "HT20"
    )

    region = re.search(r"country\s+([A-Z]{2}):", reg).group(1)
    return f"channel={chan};width={width};region={region}\n"

# ───────── regex helpers (unchanged) ─────────
_RX_ANT_RE = re.compile(
    r"^(?P<freq>\d+):(?P<mcs>\d+):(?P<bw>\d+)\s+"
    r"(?P<ant>[0-9a-fA-F]+)\s+"
    r"(?P<count>\d+):(?P<rssi_min>-?\d+):(?P<rssi_avg>-?\d+):(?P<rssi_max>-?\d+):"
    r"(?P<snr_min>-?\d+):(?P<snr_avg>-?\d+):(?P<snr_max>-?\d+)$"
)
_TX_ANT_RE = re.compile(
    r"^(?P<ant>[0-9a-fA-F]+)\s+"
    r"(?P<p_inj>\d+):(?P<p_drop>\d+):"
    r"(?P<lat_min>\d+):(?P<lat_avg>\d+):(?P<lat_max>\d+)$"
)
_PKT_RE = re.compile(r"^(?P<vals>(?:\d+:){10}\d+)$")

PKT_FIELDS = [
    "count_p_all", "count_b_all", "count_p_dec_err",
    "count_p_session", "count_p_data", "count_p_uniq",
    "count_p_fec_recovered", "count_p_lost", "count_p_bad",
    "count_p_outgoing", "count_b_outgoing",
]

# ───────── in-memory store ─────────
BUFFER = 100
_metrics_lock = threading.Lock()
_metrics: Dict[str, Dict[str, Deque[Dict[str, Any]]]] = defaultdict(
    lambda: {
        "RX_ANT": deque(maxlen=BUFFER),
        "TX_ANT": deque(maxlen=BUFFER),
        "PKT":    deque(maxlen=BUFFER),
        "MAVLINK_STATS":        deque(maxlen=BUFFER),
        "RC_CHANNELS_OVERRIDE": deque(maxlen=BUFFER),
        "RADIO_STATUS":         deque(maxlen=BUFFER),
        "MAVLINK_EXEC":         deque(maxlen=BUFFER),
    }
)

def _flush_batch(batch: Dict[str, Dict[str, List[Dict[str, Any]]]]) -> None:
    if not batch:
        return
    with _metrics_lock:
        for thr, kinds in batch.items():
            dest = _metrics[thr]
            for k, lst in kinds.items():
                if lst:
                    dest[k].extend(lst)
                    lst.clear()

# ─────────── streaming client ───────────
class StreamClient(threading.Thread):
    FLUSH_EVERY = 10          # records
    FLUSH_MS    = 0.05        # seconds

    def __init__(self, host: str, port: int, stream_cmd: str, retry: int = 5):
        super().__init__(daemon=True)
        self.h, self.p, self.cmd, self.r = host, port, stream_cmd, retry
        self._local: Dict[str, Dict[str, List[Dict[str, Any]]]] = defaultdict(
            lambda: defaultdict(list)
        )
        self._since_flush = 0
        self._last_flush_t = time.time()

        # dispatch table
        self._dispatch = {
            "RX_ANT":               self._rx_ant,
            "TX_ANT":               self._tx_ant,
            "PKT":                  self._pkt,
            "MAVLINK_STATS":        self._mstats,
            "RC_CHANNELS_OVERRIDE": self._rc,
            "RADIO_STATUS":         self._radio,
            "MAVLINK_EXEC":         self._exec,
        }

    def run(self):
        while True:
            try:
                self._once()
            except Exception as e:
                print("[Stream]", e)
            time.sleep(self.r)

    def _once(self):
        with socket.create_connection((self.h, self.p), 10) as s:
            f = s.makefile("r", encoding="ascii", newline="\n")
            s.sendall((self.cmd + "\n").encode())
            f.readline()            # banner / ack
            for ln in f:
                if ln.strip():
                    self._parse(ln.rstrip("\n"))

    # ── fast single-dispatch ──
    def _parse(self, line: str):
        try:
            head, kind, rest = line.split(maxsplit=2)
            thread, ts = head.split(':', 1)
        except ValueError:
            return  # malformed

        fn = self._dispatch.get(kind)
        if fn:
            fn(thread, ts, rest)
            self._maybe_flush()

    # ── handlers ──
    def _rx_ant(self, thread: str, ts: str, rest: str):
        m = _RX_ANT_RE.match(rest)
        if not m:
            return
        self._local[thread]["RX_ANT"].append({
            "ts": int(ts),
            "freq": int(m["freq"]),
            "mcs": int(m["mcs"]),
            "bw":  int(m["bw"]),
            "antenna_id": f"0x{int(m['ant'],16):016x}",
            "packet_count": int(m["count"]),
            "rssi": {"min": int(m["rssi_min"]), "avg": int(m["rssi_avg"]), "max": int(m["rssi_max"])},
            "snr":  {"min": int(m["snr_min"]),  "avg": int(m["snr_avg"]),  "max": int(m["snr_max"])},
        })

    def _tx_ant(self, thread: str, ts: str, rest: str):
        m = _TX_ANT_RE.match(rest)
        if not m:
            return
        self._local[thread]["TX_ANT"].append({
            "ts": int(ts),
            "antenna_id": f"0x{int(m['ant'],16):016x}",
            "pkt_injected": int(m["p_inj"]),
            "pkt_dropped":  int(m["p_drop"]),
            "latency": {"min": int(m["lat_min"]), "avg": int(m["lat_avg"]), "max": int(m["lat_max"])},
        })

    def _pkt(self, thread: str, ts: str, rest: str):
        m = _PKT_RE.match(rest)
        if not m:
            return
        vals = list(map(int, m["vals"].split(":")))
        if len(vals) == 11:
            self._local[thread]["PKT"].append(
                {"ts": int(ts), **dict(zip(PKT_FIELDS, vals))}
            )

    def _mstats(self, thread: str, ts: str, rest: str):
        self._local[thread]["MAVLINK_STATS"].append(
            {"ts": int(ts), "stats": list(map(int, rest.split(':')))}
        )

    def _rc(self, thread: str, ts: str, rest: str):
        parts = rest.split(None, 3)
        if len(parts) != 4:
            return
        length, chan_cnt, flag, chs = parts
        self._local[thread]["RC_CHANNELS_OVERRIDE"].append({
            "ts": int(ts),
            "length": int(length),
            "chan_count": int(chan_cnt),
            "flag": int(flag),
            "channels": list(map(int, chs.split(':')))
        })

    def _radio(self, thread: str, ts: str, rest: str):
        q, stats = rest.split(' ', 1)
        self._local[thread]["RADIO_STATUS"].append({
            "ts": int(ts),
            "quality": int(q),
            "stats": list(map(int, stats.split(':')))
        })

    def _exec(self, thread: str, ts: str, rest: str):
        self._local[thread]["MAVLINK_EXEC"].append({
            "ts": int(ts),
            "values": list(map(int, rest.split(':')))
        })

    # ── batching ──
    def _maybe_flush(self):
        self._since_flush += 1
        now = time.time()
        if self._since_flush >= self.FLUSH_EVERY or (now - self._last_flush_t) >= self.FLUSH_MS:
            _flush_batch(self._local)
            self._since_flush = 0
            self._last_flush_t = now

# ───────── flask app ─────────
def create_app(host: str, port: int, ui_path: Optional[pathlib.Path]) -> Flask:
    app = Flask(__name__, static_folder=None)

    # ── metrics ──
    @app.route("/metrics")
    def metrics() -> Response:
        with _metrics_lock:
            data = {t: {k: list(d) for k, d in v.items()} for t, v in _metrics.items()}
        return Response(json.dumps(data, separators=(",", ":")),
                        mimetype="application/json")

    # ── commands list ──
    @app.route("/commands")
    def commands_api() -> Response:
        return Response(json.dumps(list(COMMANDS_MAP.keys()), separators=(",", ":")),
                        mimetype="application/json")

    # ── command exec ──
    @app.route("/exec", methods=["POST"])
    def exec_api() -> Response:
        key = request.json.get("cmd") if request.is_json else None
        if not key:
            abort(400, description="missing 'cmd'")
        if key not in COMMANDS_MAP:
            abort(404, description=f"unknown command {key!r}")
        res = run_command(key)
        return Response(json.dumps(res, separators=(",", ":")),
                        mimetype="application/json")

    # ── wifi channel ──
    @app.route("/wifi_channel")
    def wifi_channel_api() -> Response:
        try:
            blob = current_channel_blob()
        except Exception as exc:
            abort(500, description=str(exc))
        return Response(blob, mimetype="text/plain")

    # ── UI (same logic as before) ──
    if ui_path:
        ui_path = ui_path.expanduser().resolve()
        if not ui_path.is_file():
            raise FileNotFoundError(ui_path)

        @app.route("/")
        def index_file():
            return send_file(ui_path)
    else:
        @app.route("/")
        def index_stub():
            return "<h2>Thread-Manager backend running</h2>"

    return app

# ───────── main ─────────
def main():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--host", default="192.168.2.20", help="Thread-Manager IP")
    p.add_argument("--port", type=int, default=9500, help="Thread-Manager TCP port")
    p.add_argument("--web-port", type=int, default=5000, help="Flask UI port")
    p.add_argument("--stream-cmd", default="list stream all",
                   help='Command sent after connect (e.g. "stream video_agg_rx stdout")')
    p.add_argument("--ui", type=pathlib.Path,
                   help="External HTML/JS file to serve instead of built-in stub")
    a = p.parse_args()

    StreamClient(a.host, a.port, a.stream_cmd).start()
    create_app(a.host, a.port, a.ui).run(host="0.0.0.0", port=a.web_port, threaded=True)

if __name__ == "__main__":
    main()
