#!/usr/bin/env python3
"""
Thread-Manager Live Monitor + One-Shot Commands  (v4.5)

* Default GRAPHS view (100-sample strip, fixed legend order)
* ANTENNA / ERRORS / THROUGHPUT charts per thread
* Chart headers now show *thread – title* (e.g. “tun_agg_rx – ERRORS”)
* Unique colours per chart, integer Y-axes, no markers
* Live Mbps totals, refresh-rate toggle, commands pane
"""
import argparse, socket, threading, time, re
from collections import defaultdict, deque
from typing import Dict, Deque, Any
from flask import Flask, jsonify, request, Response, abort

# ─────────── regex helpers ────────────
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
PKT_FIELDS = [
    "count_p_all","count_b_all","count_p_dec_err",
    "count_p_session","count_p_data","count_p_uniq",
    "count_p_fec_recovered","count_p_lost","count_p_bad",
    "count_p_outgoing","count_b_outgoing",
]

# ─────────── in-memory store ──────────
BUFFER = 200
_metrics_lock = threading.Lock()
_metrics: Dict[str, Dict[str, Deque[Dict[str, Any]]]] = defaultdict(
    lambda: {"RX_ANT": deque(maxlen=BUFFER),
             "TX_ANT": deque(maxlen=BUFFER),
             "PKT":    deque(maxlen=BUFFER)}
)
def _store(thread:str, kind:str, data:Dict[str,Any]) -> None:
    with _metrics_lock:
        _metrics[thread][kind].append(data)

# ─────────── streaming client ─────────
class StreamClient(threading.Thread):
    def __init__(self, host:str, port:int, retry:int=5):
        super().__init__(daemon=True)
        self.h,self.p,self.r = host,port,retry
    def run(self):
        while True:
            try: self._once()
            except Exception as e: print("[Stream]",e)
            time.sleep(self.r)
    def _once(self):
        with socket.create_connection((self.h,self.p),10) as s:
            f=s.makefile("r",encoding="ascii",newline="\n")
            s.sendall(b"list stream all\n"); f.readline()
            for ln in f:
                if ln.strip(): self._parse(ln.rstrip("\n"))
    def _parse(self,line:str):
        if (m:=_RX_ANT_RE.match(line)):
            _store(m["thread"],"RX_ANT",{
                "ts":int(m["ts"]),
                "freq":int(m["freq"]),"mcs":int(m["mcs"]),"bw":int(m["bw"]),
                "antenna_id":f"0x{int(m['ant'],16):016x}",
                "packet_count":int(m["count"]),
                "rssi":{k:int(m[k]) for k in ("rssi_min","rssi_avg","rssi_max")},
                "snr": {k:int(m[k]) for k in ("snr_min","snr_avg","snr_max")}
            }); return
        if (m:=_TX_ANT_RE.match(line)):
            _store(m["thread"],"TX_ANT",{
                "ts":int(m["ts"]),
                "antenna_id":f"0x{int(m['ant'],16):016x}",
                "pkt_injected":int(m["p_inj"]),
                "pkt_dropped": int(m["p_drop"]),
                "latency":{k:int(m[k]) for k in ("lat_min","lat_avg","lat_max")}
            }); return
        if (m:=_PKT_RE.match(line)):
            vals=list(map(int,m["vals"].split(":")))
            if len(vals)==11:
                _store(m["thread"],"PKT",
                       {"ts":int(m["ts"]),**dict(zip(PKT_FIELDS,vals))})

# ── one-shot command helpers ──
def tm_request(host:str,port:int,cmd:str)->str:
    try:
        with socket.create_connection((host,port),5) as s:
            s.sendall((cmd+"\n").encode())
            return s.recv(4096).decode(errors='ignore').strip()
    except Exception as e: return f"ERROR: {e}"
def list_commands(host,port):
    return [l for l in tm_request(host,port,"list commands").splitlines() if l]

# ───────────── flask app ─────────────
def create_app(host:str, port:int) -> Flask:
    app=Flask(__name__)

    @app.route("/metrics")
    def metrics() -> Response:
        with _metrics_lock:
            return jsonify({t:{k:list(d) for k,d in v.items()} for t,v in _metrics.items()})

    @app.route("/commands")
    def commands_api() -> Response:
        return jsonify(list_commands(host,port))

    @app.route("/exec", methods=["POST"])
    def exec_api() -> Response:
        cmd=request.json.get("cmd")
        if not cmd: abort(400)
        return jsonify({"output": tm_request(host,port,f"exec {cmd}")})

    # ---------------- HTML / JS ----------------
    INDEX_HTML = r"""
<!doctype html><html lang=en><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Thread-Manager</title><script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>:root{--bg:#e8f0ff;--fg:#023;--accent:#1976d2;--card:#fff}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{background:var(--accent);color:#fff;display:flex;flex-wrap:wrap;gap:.5rem;align-items:center;padding:.7rem 1rem}
header h1{margin:0;font-size:1rem}button.btn{border:none;background:#fff;color:var(--accent);padding:.35rem .9rem;border-radius:6px;font-weight:600;cursor:pointer}
.statsline{font-size:.9rem;width:100%;margin-top:.2rem}
.tabs{display:flex;position:sticky;top:0;z-index:1;background:var(--bg)}
.tab{flex:1;padding:.55rem 0;text-align:center;font-weight:600;border:none;background:#cde0ff;color:var(--fg);cursor:pointer}
.tab.active{background:#fff;border-bottom:3px solid var(--accent)}
section{padding:1rem}.tab-content{display:none}.tab-content.active{display:block}
.thread{margin:.35rem 0;font-size:.95rem;font-weight:600}
.chart-title{font-size:.8rem;color:#555;margin:.1rem 0 .2rem}
.card,.chart{background:var(--card);border-radius:8px;box-shadow:0 2px 4px #0002;margin:.35rem 0;padding:.4rem .6rem}
.card{white-space:pre-wrap;font-family:monospace;font-size:.78rem;overflow-x:auto}
canvas{width:100%!important;height:220px!important}@media(max-width:600px){canvas{height:150px!important}}
.cmd-btn{width:100%;margin:.25rem 0;padding:.5rem;border:none;border-radius:6px;background:#1976d2;color:#fff;font-weight:600;cursor:pointer}
#cmd-out{background:#000;color:#0f0;font-family:monospace;font-size:.8rem;padding:.5rem;border-radius:6px;min-height:1.2rem;margin-top:.5rem;white-space:pre-wrap}
</style></head><body>
<header><h1>Thread-Manager</h1>
  <button id=mode  class=btn>STATS</button>
  <button id=refresh class=btn>REF 1000ms</button>
  <button id=cmdToggle class=btn>COMMANDS</button>
  <div id=bytesline class=statsline></div>
</header>

<div id=metrics>
  <div class=tabs><button class="tab active" data-tab=rx>RX</button><button class=tab data-tab=tx>TX</button></div>
  <section id=rx class="tab-content active"></section><section id=tx class="tab-content"></section>
</div>
<section id=commands style="display:none"></section>

<script>
/* ---------- helpers & constants ---------- */
const MAX=100;
const pool=['#1976d2','#ef5350','#66bb6a','#ffa726','#ab47bc','#26c6da','#ff7043','#8d6e63','#42a5f5','#d4e157'];
const colour=i=>pool[i%pool.length];
const charts=new Map(), wrappers=new Map();
const ckey=(tab,thr,sfx)=>`${tab}:${thr}:${sfx}`;
const ERR_KEYS=['bad','lost','rec','dec'], THR_KEYS=['all','out','uniq'];

/* ---------- view & refresh --------------- */
let mode='graphs', view='metrics';
const ints=[200,400,600,800,1000]; let idx=4; let timer;
const modeBtn=document.getElementById('mode'), refBtn=document.getElementById('refresh'), bytesLine=document.getElementById('bytesline');
modeBtn.onclick=()=>{if(view==='commands')toggleCmd(false);mode=mode==='stats'?'graphs':'stats';modeBtn.textContent=mode==='stats'?'GRAPHS':'STATS';refresh();}
refBtn.onclick=()=>{idx=(idx+1)%ints.length;refBtn.textContent=`REF ${ints[idx]}ms`;sched();}
document.getElementById('cmdToggle').onclick=()=>toggleCmd(view!=='commands');
function toggleCmd(en){if(en){view='commands';metrics.style.display='none';commands.style.display='';if(!commands.hasChildNodes())loadCmd();}else{view='metrics';metrics.style.display='';commands.style.display='none';}}
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active',t===b));document.querySelectorAll('.tab-content').forEach(s=>s.classList.toggle('active',s.id===b.dataset.tab));});
function sched(){clearTimeout(timer);if(view==='metrics')refresh();timer=setTimeout(sched,ints[idx]);}sched();

/* ---------- metrics update --------------- */
async function refresh(){
  const r=await fetch('/metrics'); if(!r.ok)return; const data=await r.json();
  let inB=0,outB=0;Object.values(data).forEach(t=>{const p=t.PKT.at(-1);if(p){inB+=p.count_b_all;outB+=p.count_b_outgoing;}});
  bytesLine.textContent=`Total in ${(inB*8e-6).toFixed(2)} Mbit/time | out ${(outB*8e-6).toFixed(2)} Mbit/time`;
  drawTab('rx',data,'RX_ANT',s=>s.rssi.rssi_avg); drawTab('tx',data,'TX_ANT',s=>s.latency.lat_avg);
}
function drawTab(tabId,data,antKey,ySel){
  const cont=document.getElementById(tabId);
  if(mode==='stats'){
    cont.innerHTML='';[...charts.keys()].filter(k=>k.startsWith(tabId)).forEach(k=>{charts.get(k).destroy();charts.delete(k);wrappers.delete(k);});
    let html='';for(const[thr,kinds]of Object.entries(data)){if(!kinds[antKey]?.length)continue;
      html+=`<div class=thread>${thr}</div><div class=card>${antKey}\n${JSON.stringify(kinds[antKey].at(-1),null,2)}</div>`;
      if(kinds.PKT?.length)html+=`<div class=card>PKT\n${JSON.stringify(kinds.PKT.at(-1),null,2)}</div>`;}
    cont.innerHTML=html||'<p>No data…</p>';return;}
  cont.querySelectorAll('.thread,.card').forEach(el=>el.remove());
  for(const[thr,kinds]of Object.entries(data)){
    if(!kinds[antKey]?.length)continue;
    renderStrip(tabId,thr,'ant',`${thr} – ANTENNA`,kinds[antKey],s=>s.antenna_id,v=>v.at(-1),ySel);
    if(kinds.PKT?.length){
      renderStrip(tabId,thr,'err',`${thr} – ERRORS`,[kinds.PKT.at(-1)],()=>1,p=>({bad:p.count_p_bad,lost:p.count_p_lost,rec:p.count_p_fec_recovered,dec:p.count_p_dec_err}),null,{min:0},ERR_KEYS);
      renderStrip(tabId,thr,'thr',`${thr} – THROUGHPUT`,[kinds.PKT.at(-1)],()=>1,p=>({all:p.count_p_all,out:p.count_p_outgoing,uniq:p.count_p_uniq}),null,{min:0},THR_KEYS);}}
}
/* generic strip ------------------------ */
function renderStrip(tab,thr,sfx,title,samples,keyFn,valFn,ySel,yExtra={},fixedOrder=null){
  const id=ckey(tab,thr,sfx);let wrap=wrappers.get(id);
  if(!wrap){
    wrap=document.createElement('div');wrap.className='chart';
    wrap.innerHTML=`<div class=chart-title>${title}</div><canvas></canvas>`;
    document.getElementById(tab).appendChild(wrap);
    const ctx=wrap.querySelector('canvas');
    charts.set(id,new Chart(ctx,{type:'line',data:{labels:[...Array(MAX).keys()],datasets:[]},
      options:{animation:false,responsive:true,maintainAspectRatio:false,spanGaps:true,
               elements:{point:{radius:0}},
               scales:{x:{type:'linear',min:0,max:MAX-1,ticks:{display:false}},
                       y:{beginAtZero:true,ticks:{precision:0,callback:v=>v},...yExtra}},
               plugins:{legend:{display:true,position:'bottom'}}}}));
    wrappers.set(id,wrap);}
  const chart=charts.get(id);
  let dSets=[], index=0;
  if(sfx==='ant'){
    const groups={};samples.forEach(s=>(groups[keyFn(s)]??=[]).push(ySel(s)));
    Object.keys(groups).sort().forEach(lab=>{let ds=chart.data.datasets.find(d=>d.label===lab);
      if(!ds)ds={label:lab,borderColor:colour(index),backgroundColor:colour(index),tension:0,fill:false,data:Array(MAX).fill(null)};
      ds.data.push(valFn(groups[lab]));if(ds.data.length>MAX)ds.data.shift();dSets.push(ds);index++;});
  }else{
    const obj=valFn(samples.at(-1));(fixedOrder||Object.keys(obj).sort()).forEach(lab=>{let ds=chart.data.datasets.find(d=>d.label===lab);
      if(!ds)ds={label:lab,borderColor:colour(index),backgroundColor:colour(index),tension:0,fill:false,data:Array(MAX).fill(null)};
      ds.data.push(obj[lab]);if(ds.data.length>MAX)ds.data.shift();dSets.push(ds);index++;});
  }
  chart.data.datasets=dSets;chart.update('none');
}

/* ---------- COMMANDS ------------- */
async function loadCmd(){const sec=document.getElementById('commands');sec.innerHTML='<p>Loading…</p>';
  try{const cmds=await(await fetch('/commands')).json();let h='';cmds.forEach(c=>h+=`<button class=cmd-btn data-cmd="${c}">${c}</button>`);h+='<div id=cmd-out></div>';sec.innerHTML=h;sec.querySelectorAll('.cmd-btn').forEach(b=>b.onclick=exec);}
  catch(e){sec.innerHTML=`<p style="color:red">ERROR ${e}</p>`;}}
async function exec(e){const b=e.target,cmd=b.dataset.cmd;b.disabled=true;const out=document.getElementById('cmd-out');out.textContent='…';
  try{const j=await(await fetch('/exec',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd})})).json();out.textContent=j.output;}
  catch(err){out.textContent='ERROR '+err;}b.disabled=false;}
</script></body></html>
"""
    @app.route("/")
    def index():
        return INDEX_HTML
    return app

# ───────── main ─────────
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--host",default="192.168.2.20")
    p.add_argument("--port",type=int,default=9500)
    p.add_argument("--web-port",type=int,default=5000)
    a=p.parse_args()
    StreamClient(a.host,a.port).start()
    create_app(a.host,a.port).run(host="0.0.0.0",port=a.web_port,threaded=True)

if __name__=="__main__":
    main()
