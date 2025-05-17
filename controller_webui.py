#!/usr/bin/env python3
"""
Thread-Manager Live Monitor – v3.1-fixed-strip

Same as the v3.1 you supplied, plus:
    • fixed-length 100-sample strip per antenna
    • graphs update in-place (no flicker), toggle button intact
"""
import argparse, re, socket, threading, time
from collections import defaultdict, deque
from typing import Dict, Deque, Any
from flask import Flask, jsonify, Response

##############################################################################
# -- regex & parsing helpers (unchanged) -------------------------------------
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
_PKT_RE = re.compile(r"^(?P<thread>[\w\-]+):(?P<ts>\d+)\s+PKT\s+(?P<vals>(?:\d+:){10}\d+)$")
PKT_FIELDS = [
    "count_p_all", "count_b_all", "count_p_dec_err",
    "count_p_session", "count_p_data", "count_p_uniq",
    "count_p_fec_recovered", "count_p_lost", "count_p_bad",
    "count_p_outgoing", "count_b_outgoing",
]

##############################################################################
# -- in-memory store ---------------------------------------------------------
BUFFER = 200
_metrics_lock = threading.Lock()
_metrics: Dict[str, Dict[str, Deque[Dict[str, Any]]]] = defaultdict(
    lambda: {"RX_ANT": deque(maxlen=BUFFER),
             "TX_ANT": deque(maxlen=BUFFER),
             "PKT":    deque(maxlen=BUFFER)}
)
def _store(thread:str, kind:str, data:Dict[str,Any])->None:
    with _metrics_lock: _metrics[thread][kind].append(data)

##############################################################################
# -- streaming client --------------------------------------------------------
class StreamClient(threading.Thread):
    def __init__(self, host="127.0.0.1", port=9500, retry=5):
        super().__init__(daemon=True); self.h,self.p,self.r=host,port,retry
    def run(self):
        while True:
            try: self._once()
            except Exception as e: print("[Stream]",e)
            time.sleep(self.r)
    def _once(self):
        with socket.create_connection((self.h,self.p),10) as s:
            f=s.makefile("r",encoding="ascii",newline="\n")
            s.sendall(b"list stream all\n"); f.readline()
            for line in f:
                if not line.strip(): continue
                self._parse(line.rstrip("\n"))
    def _parse(self,line:str):
        if (m:=_RX_ANT_RE.match(line)):
            _store(m["thread"],"RX_ANT",{
                "ts":int(m["ts"]),"freq":int(m["freq"]),"mcs":int(m["mcs"]),"bw":int(m["bw"]),
                "antenna_id":f"0x{int(m['ant'],16):016x}","packet_count":int(m["count"]),
                "rssi":{k:int(m[k]) for k in ("rssi_min","rssi_avg","rssi_max")},
                "snr": {k:int(m[k]) for k in ("snr_min","snr_avg","snr_max")}})
            return
        if (m:=_TX_ANT_RE.match(line)):
            _store(m["thread"],"TX_ANT",{
                "ts":int(m["ts"]),"antenna_id":f"0x{int(m['ant'],16):016x}",
                "pkt_injected":int(m["p_inj"]),"pkt_dropped":int(m["p_drop"]),
                "latency":{k:int(m[k]) for k in ("lat_min","lat_avg","lat_max")}})
            return
        if (m:=_PKT_RE.match(line)):
            vals=list(map(int,m["vals"].split(":")))
            if len(vals)==11: _store(m["thread"],"PKT",
                       {"ts":int(m["ts"]),**dict(zip(PKT_FIELDS,vals))})

##############################################################################
# -- Flask UI ----------------------------------------------------------------
def create_app()->Flask:
    app=Flask(__name__)

    @app.route("/metrics")
    def metrics()->Response:
        with _metrics_lock:
            return jsonify({t:{k:list(d) for k,d in v.items()} for t,v in _metrics.items()})

    INDEX_HTML=r"""
<!doctype html><html lang=en><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Thread-Manager</title><script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>:root{--bg:#e8f0ff;--fg:#023;--accent:#1976d2;--card:#fff}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{background:var(--accent);color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.7rem 1rem}
header h1{margin:0;font-size:1rem}button#mode{border:none;background:#fff;color:var(--accent);padding:.35rem .9rem;border-radius:6px;font-weight:600;cursor:pointer}
.tabs{display:flex;position:sticky;top:0;z-index:1;background:var(--bg)}
.tab{flex:1;padding:.55rem 0;text-align:center;font-weight:600;border:none;background:#cde0ff;color:var(--fg);cursor:pointer}
.tab.active{background:#fff;border-bottom:3px solid var(--accent)}
section{padding:1rem}.tab-content{display:none}.tab-content.active{display:block}
.thread{margin:.35rem 0;font-size:1rem;font-weight:600}
.card,.chart{background:var(--card);border-radius:8px;box-shadow:0 2px 4px #0002;margin:.3rem 0;padding:.4rem .6rem}
.card{white-space:pre-wrap;font-family:monospace;font-size:.78rem;overflow-x:auto}
canvas{width:100%!important;height:240px!important}@media(max-width:600px){canvas{height:160px!important}}
</style></head><body>
<header><h1>Thread-Manager Live Metrics</h1><button id=mode>GRAPHS</button></header>
<div class=tabs><button class="tab active" data-tab=rx>RX</button><button class=tab data-tab=tx>TX</button></div>
<section id=rx class="tab-content active"></section><section id=tx class="tab-content"></section>

<script>
/* ---------- UI ---------- */
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{
  document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active',t===b));
  document.querySelectorAll('.tab-content').forEach(s=>s.classList.toggle('active',s.id===b.dataset.tab));
});
let showGraphs=true; const mode=document.getElementById('mode');
mode.onclick=()=>{showGraphs=!showGraphs;mode.textContent=showGraphs?'STATS':'GRAPHS';};

/* ---------- helpers ---------- */
const MAX=100;
const colour=id=>`hsl(${parseInt(id.slice(-4),16)%360} 65% 55%)`;
const charts=new Map();                       // key -> Chart
const wrappers=new Map();                     // key -> wrapper <div>
const ckey=(tab,thr)=>`${tab}:${thr}`;

/* ---------- main loop ---------- */
setInterval(refresh,1000); refresh();
async function refresh(){
  const r=await fetch('/metrics'); if(!r.ok) return;
  const data=await r.json();
  draw('rx',data,'RX_ANT',s=>s.rssi.rssi_avg);
  draw('tx',data,'TX_ANT',s=>s.latency.lat_avg);
}

/* ---------- draw one tab ---------- */
function draw(tabId,data,type,ySel){
  const cont=document.getElementById(tabId);

  /* TEXT MODE */
  if(!showGraphs){
    cont.innerHTML='';
    [...charts.keys()].filter(k=>k.startsWith(tabId)).forEach(k=>{charts.get(k).destroy();charts.delete(k);wrappers.delete(k);});
    let html='';
    for(const [thr,k] of Object.entries(data)){
      if(k[type]?.length){
        html+=`<div class=thread>${thr}</div><div class=card>${type}\n${JSON.stringify(k[type].at(-1),null,2)}</div>`;
        if(k.PKT?.length) html+=`<div class=card>PKT\n${JSON.stringify(k.PKT.at(-1),null,2)}</div>`;
      }
    }
    cont.innerHTML=html||'<p>No data…</p>'; return;
  }

  /* GRAPH MODE */
  for(const [thr,k] of Object.entries(data)){
    if(!k[type]?.length) continue;

    const id=ckey(tabId,thr);
    /* create wrapper + canvas once */
    if(!wrappers.has(id)){
      const div=document.createElement('div');div.className='chart';
      div.innerHTML=`<div class=thread>${thr}</div><canvas></canvas>`;
      cont.appendChild(div); wrappers.set(id,div);
      const ctx=div.querySelector('canvas');
      charts.set(id,new Chart(ctx,{type:'line',
        data:{labels:[...Array(MAX).keys()],datasets:[]},
        options:{animation:false,responsive:true,maintainAspectRatio:false,
                 spanGaps:true,                                /* <── NEW */
                 scales:{x:{type:'linear',min:0,max:MAX-1,ticks:{display:false}}},
                 plugins:{legend:{display:true,position:'bottom'}}}}));
    }
    const chart=charts.get(id);

    /* group by antenna */
    const per={}; for(const s of k[type]){(per[s.antenna_id]??=[]).push(ySel(s));}

    /* sync datasets */
    chart.data.datasets = Object.entries(per).map(([lab,vals])=>{
      let ds = chart.data.datasets.find(d=>d.label===lab);
      if(!ds) ds={label:lab,borderColor:colour(lab),backgroundColor:colour(lab),
                  tension:0,fill:false,data:Array(MAX).fill(null)};
      /* append newest single value */
      ds.data.push(vals.at(-1)); if(ds.data.length>MAX) ds.data.shift();
      return ds;
    });
    chart.update('none');
  }
}
</script></body></html>
"""
    @app.route("/")
    def index(): return INDEX_HTML
    return app

##############################################################################
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--host",default="127.0.0.1"); p.add_argument("--port",type=int,default=9500)
    p.add_argument("--web-port",type=int,default=5000)
    a=p.parse_args()
    StreamClient(a.host,a.port).start()
    create_app().run(host="0.0.0.0",port=a.web_port,threaded=True)

if __name__=="__main__": main()
