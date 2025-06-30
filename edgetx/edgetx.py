#!/usr/bin/env python3
"""
EdgeTX → RC_CHANNELS_OVERRIDE bridge + Wi-Fi link-health G-VAR
=============================================================

Adds HTTP calls for the new Lua “SET,WIFI,<ch>” and “EXEC,<idx>” lines:

* `SET,WIFI,157`  →  POST /exec {"cmd":"edgetx_wifi_157"}
* `EXEC,2`        →  POST /exec {"cmd":"edgetx_exec_2"}

The target URL is configurable with **--exec-url** (default
`http://127.0.0.1/exec`).  When **--debug** is given we log every
outbound request and any error/response text.

All previous behaviour (serial parsing, RC translation, link-penalty
logic, etc.) is unchanged.

NEW CLI flag
------------
    --exec-url URL      (default http://127.0.0.1/exec)
"""

from __future__ import annotations
import argparse, json, signal, subprocess, sys, threading, time, re
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import serial        # type: ignore
except ModuleNotFoundError:
    sys.stderr.write("pip install pyserial\n"); sys.exit(1)

try:
    import requests      # optional, nicer than urllib
except ModuleNotFoundError:
    requests = None      # we’ll fall back to urllib if absent

import urllib.request   # always available in stdlib

# ───────── output scaling (edit here) ───────────────────────────────────
OUT_MIN, OUT_MAX = 988, 2012                 # <-- set e.g. 1012, 1988
# -----------------------------------------------------------------------

PWM_MIN, PWM_MAX = OUT_MIN, OUT_MAX
PWM_RANGE        = PWM_MAX - PWM_MIN
HALF_RANGE       = PWM_RANGE / 2              # used for channel mapping

GV_MIN, GV_MAX   = -1024, 1024

RSSI_MIN, RSSI_MAX = -100, -30                # dBm → PWM base
RSSI_RANGE         = RSSI_MAX - RSSI_MIN

MAX_PENALTY       = 750                       # µs
SERIAL_RETRY_SEC  = 3
WFBRX_RETRY_SEC   = 3

SCRIPT = Path("/usr/bin/channels.sh")
r_pkt, r_rxant = re.compile(r"\bPKT\b"), re.compile(r"\bRX_ANT\b")

# ───────── helpers ──────────────────────────────────────────────────────
def _clamp(v:int, lo:int, hi:int)->int: return max(lo, min(hi, v))

def rssi_to_pwm(rssi:int)->int:
    frac = (_clamp(rssi, RSSI_MIN, RSSI_MAX) - RSSI_MIN) / RSSI_RANGE
    return int(round(PWM_MIN + frac*PWM_RANGE))

def pwm_to_gv(pwm:int)->int:
    pwm = _clamp(pwm, PWM_MIN, PWM_MAX)
    return int(round((pwm - 1500) * 1024 / HALF_RANGE))

def chan_raw_to_pwm(raw:int)->int:
    """EdgeTX channel –1024…+1024 ⇒ OUT_MIN…OUT_MAX µs (symmetric)"""
    return int(round(1500 + _clamp(raw, -1024, 1024) * HALF_RANGE / 1024))

signal.signal(signal.SIGPIPE, signal.SIG_DFL)

# ------------- Wi-Fi reader thread (unchanged) --------------------------
class WfbReader(threading.Thread):
    def __init__(self, sniffer:'EdgeTXSniffer', *,
                 disable_penalty:bool, debug:bool)->None:
        super().__init__(daemon=True)
        self.sniffer, self.disable_penalty, self.debug = sniffer, disable_penalty, debug
    @staticmethod
    def _penalty(lost:int, fec:int, ratio:float)->tuple[int,int,int,int]:
        p_lost = (lost*200) if lost<=3 else 600+(lost-3)*50
        p_lost = _clamp(p_lost, 0, 500)
        p_fec  = _clamp((fec*5) if fec<=10 else 50+(fec-10)*10 if fec<=25
                        else 200+(fec-25)*15, 0, 200)
        if   ratio<1.5: p_div=int(round((1.5-ratio)/0.5*100))
        elif ratio<1.8: p_div=int(round((1.8-ratio)/0.3*50))
        elif ratio<2.8: p_div=int(round((2.8-ratio)/1.0*20))
        else:           p_div=0
        p_total=_clamp(p_lost+p_fec+p_div,0,MAX_PENALTY)
        return p_total,p_lost,p_fec,p_div
    def run(self)->None:
        while True:
            try:
                proc=subprocess.Popen(["nc","127.0.0.1","9500"],
                                       stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                       text=True)
                assert proc.stdin
                proc.stdin.write("wfb_rx\n"); proc.stdin.flush()
                best:Optional[int]=None
                for line in proc.stdout:               # type: ignore[arg-type]
                    if r_rxant.search(line):
                        try: avg=int(line.split()[4].split(":")[2])
                        except (IndexError,ValueError): continue
                        if best is None or avg>best: best=avg
                    elif r_pkt.search(line):
                        tok=next((t for t in line.split()
                                  if t[0].isdigit() and ':' in t),"")
                        try: nums=[int(x) for x in tok.split(':')]
                        except ValueError: nums=[]
                        if len(nums)<8: best=None; continue
                        total_p, uniq_p, fec_cnt, lost_cnt = nums[0], nums[5], nums[6], nums[7]
                        ratio = total_p/uniq_p if uniq_p else 0.0
                        if best is not None:
                            pwm_base=rssi_to_pwm(best)
                            if self.disable_penalty:
                                pwm_final=pwm_base
                                p_total=p_lost=p_fec=p_div=0
                            else:
                                p_total,p_lost,p_fec,p_div = \
                                    self._penalty(lost_cnt,fec_cnt,ratio)
                                pwm_final=_clamp(pwm_base-p_total, PWM_MIN, PWM_MAX)
                            gv_val=_clamp(pwm_to_gv(pwm_final), GV_MIN, GV_MAX)
                            if self.debug:
                                print(f"[DBG] RSSI={best:<4} base={pwm_base} "
                                      f"lost={lost_cnt}({p_lost}) "
                                      f"fec={fec_cnt}({p_fec}) "
                                      f"div={ratio:.2f}({p_div}) "
                                      f"pen={p_total} link={pwm_final} gv={gv_val}",
                                      flush=True)
                            self.sniffer.send_gv(gv_val)
                        best=None
            except Exception as exc:                   # pylint: disable=broad-except
                if self.debug: print(f"[WARN] WiFi reader err: {exc}", flush=True)
            try:
                if proc and proc.poll() is None: proc.kill()
            except Exception: pass
            if self.debug:
                print(f"[WARN] WiFi stream lost – retry in {WFBRX_RETRY_SEC}s",
                      flush=True)
            time.sleep(WFBRX_RETRY_SEC)

# ------------- main sniffer ---------------------------------------------
class EdgeTXSniffer:                             # pylint: disable=too-many-instance-attributes
    _POLL_SLEEP=0.01
    def __init__(self,*,port:str,baud:int,period_ms:int,debug:bool,
                 command_parse:bool,min_change:int,persist_ms:int,pause_ms:int,
                 channels_filter:Optional[set[int]],disable_penalty:bool,
                 exec_url:str)->None:
        self.port,self.baud,self.debug=port,baud,debug
        self.exec_url=exec_url.rstrip('/')       # POST target
        self.requests_available=bool(requests)
        self.ser:Optional[serial.Serial]=None
        self.serial_lock=threading.Lock()
        self._connect_serial()
        self.period_ms=max(1,period_ms); self.period_sec=self.period_ms/1000
        self._start=time.time(); self._next=self._start+self.period_sec
        self.latest_pwm=[1500]*16; self.rc_cnt=0
        self.cmd_parse,self.min_change,self.persist_ms,self.pause_ms = \
            command_parse,min_change,persist_ms,pause_ms
        self.allowed=channels_filter
        self.pending:[Optional[Dict[str,Any]]]=[None]*16
        self.last:[Optional[int]]=[None]*16
        self.next_allowed_ms=0
        self.exec_idx=0; self.idx_lock=threading.Lock()
        if self.debug:
            print(f"[INFO] {self.port}@{self.baud} agg={self.period_ms}ms "
                  f"→ POST {self.exec_url}",flush=True)
        WfbReader(self, disable_penalty=disable_penalty, debug=debug).start()

    # ─── Serial helpers ────────────────────────────────────────────────
    def _connect_serial(self)->None:
        while True:
            try: self.ser=serial.Serial(self.port,self.baud,timeout=0.1); break
            except serial.SerialException as e:
                if self.debug: print(f"[WARN] open {e} – retry {SERIAL_RETRY_SEC}s",
                                     flush=True)
                time.sleep(SERIAL_RETRY_SEC)
        if self.debug: print("[INFO] serial connected",flush=True)

    def _reset_serial(self)->None:
        with self.serial_lock:
            try: self.ser.close()
            except Exception: pass
            self.ser=None
        if self.debug: print("[WARN] serial lost – reconnect …",flush=True)
        self._connect_serial()

    # ─── Small helper to POST commands to /exec asynchronously ─────────
    def _post_cmd(self, cmd:str)->None:
        if self.debug: print(f"[>>] POST {self.exec_url} cmd={cmd}", flush=True)

        def worker()->None:
            try:
                if self.requests_available:
                    resp = requests.post(self.exec_url, json={"cmd": cmd}, timeout=2)  # type: ignore
                    if self.debug:
                        print(f"[<<] {resp.status_code} {resp.text[:120]}", flush=True)
                else:
                    data = json.dumps({"cmd": cmd}).encode()
                    req  = urllib.request.Request(self.exec_url, data=data,
                                                  headers={"Content-Type":"application/json"})
                    with urllib.request.urlopen(req, timeout=2) as resp:
                        if self.debug:
                            print(f"[<<] {resp.status} {resp.read(120).decode(errors='ignore')}",
                                  flush=True)
            except Exception as exc:                                  # pylint: disable=broad-except
                if self.debug: print(f"[ERR] POST fail: {exc}", flush=True)
        threading.Thread(target=worker, daemon=True).start()

    # ─── GV sender (unchanged) ─────────────────────────────────────────
    def send_gv(self,val:int)->None:
        with self.serial_lock:
            if self.ser:
                try: self.ser.write(f"GV,0,{val}\n".encode())
                except serial.SerialException: self._reset_serial(); return
        if self.debug: print(f"[>>] GV,0,{val}",flush=True)

    # ─── Main runtime loop ─────────────────────────────────────────────
    def run(self)->None:
        try:
            while True:
                now=time.time()
                line=""
                if self.ser:
                    try: line=self.ser.readline().decode(errors="replace").strip()
                    except serial.SerialException: self._reset_serial()
                if line:
                    if   line.startswith("CH,"):          self._handle_ch(line)
                    elif line.startswith("SET,WIFI,"):    self._handle_wifi(line)
                    elif line.startswith("EXEC,"):        self._handle_exec(line)

                if now>=self._next:
                    if self.rc_cnt: self._print_override(); self.rc_cnt=0
                    self._next+=self.period_sec
                if not line: time.sleep(self._POLL_SLEEP)
        except KeyboardInterrupt:
            if self.debug: print("[INFO] terminated",flush=True)

    # ─── Serial line handlers ──────────────────────────────────────────
    def _handle_wifi(self,line:str)->None:
        # format: SET,WIFI,<channel>
        try:
            ch=int(line.split(",")[2])
            self._post_cmd(f"edgetx_wifi_{ch}")
        except (IndexError,ValueError):
            if self.debug: print(f"[WARN] bad SET line: {line}", flush=True)

    def _handle_exec(self,line:str)->None:
        # format: EXEC,<index>
        try:
            idx=int(line.split(",")[1])
            self._post_cmd(f"edgetx_exec_{idx}")
        except (IndexError,ValueError):
            if self.debug: print(f"[WARN] bad EXEC line: {line}", flush=True)

    def _handle_ch(self,line:str)->None:
        parts=line.split(",")
        if len(parts)!=17: return
        try: raws=[int(x) for x in parts[1:]]
        except ValueError: return
        self.latest_pwm=[chan_raw_to_pwm(v) for v in raws]   # OUT_MIN…OUT_MAX
        self.rc_cnt+=1
        if self.cmd_parse: self._eval_channels(self.latest_pwm)

    # ─── Remaining original helpers (unchanged) ───────────────────────
    def _print_override(self)->None:
        ts=int((time.time()-self._start)*1000)
        print(f"{ts} RC_CHANNELS_OVERRIDE {self.rc_cnt} 0 0 "
              f"{':'.join(map(str,self.latest_pwm))}",flush=True)
    def _eval_channels(self,pwm:List[int])->None:
        now=self._now_ms()
        for idx,val in enumerate(pwm):
            ch=idx+1
            if self.allowed and ch not in self.allowed: continue
            prev=self.last[idx]
            if prev is None: self.last[idx]=val; continue
            diff=abs(val-prev); cand=self.pending[idx]
            if cand:
                if diff<self.min_change: self.pending[idx]=None; continue
                if now-cand["start"]>=self.persist_ms:
                    if self._try_exec(ch,prev,val,now):
                        self.last[idx]=val; self.pending[idx]=None
                continue
            if diff>=self.min_change: self.pending[idx]={"start":now}
    def _try_exec(self,ch:int,old:int,new:int,now_ms:int)->bool:
        if now_ms<self.next_allowed_ms or not SCRIPT.exists(): return False
        with self.idx_lock: self.exec_idx+=1; idx=self.exec_idx
        def worker()->None:
            start=time.time()
            try:
                res=subprocess.run([str(SCRIPT),str(ch),str(new)],
                                   capture_output=True)
                rt=int(round((time.time()-start)*1000))
                if res.returncode!=0 and self.debug:
                    print(f"[WARN] channels.sh err: "
                          f"{res.stderr.decode(errors='ignore')}",flush=True)
            except Exception as exc:
                if self.debug: print(f"[WARN] exec fail {exc}",flush=True)
                return
            ts=self._now_ms()
            print(f"{ts} MAVLINK_EXEC {idx}:{ch}:{old}:{new}:{rt}",flush=True)
        threading.Thread(target=worker,daemon=True).start()
        self.next_allowed_ms=now_ms+self.pause_ms; return True
    def _now_ms(self)->int: return int((time.time()-self._start)*1000)

# ───────── CLI ──────────────────────────────────────────────────────────
def _parse_filter(txt:Optional[str])->Optional[set[int]]:
    if not txt: return None
    try: s={int(x) for x in txt.split(',') if x.strip()}
    except ValueError: sys.stderr.write("bad --channels\n"); sys.exit(1)
    if any(not 1<=c<=16 for c in s): sys.stderr.write("channels 1-16\n"); sys.exit(1)
    return s

def _args()->argparse.Namespace:
    p=argparse.ArgumentParser("EdgeTX RC bridge + Wi-Fi GVAR health")
    p.add_argument("--port",default="/dev/ttyACM0")
    p.add_argument("--baud",type=int,default=115200)
    p.add_argument("--period",type=int,default=1000,metavar="MS")
    p.add_argument("--debug",action="store_true")
    p.add_argument("--command-parse",action="store_true")
    p.add_argument("--min-change",type=int,default=100)
    p.add_argument("--persist",type=int,default=500,metavar="MS")
    p.add_argument("--pause",type=int,default=500,metavar="MS")
    p.add_argument("--channels")
    p.add_argument("--disable-penalty",action="store_true")
    p.add_argument("--exec-url",default="http://127.0.0.1/exec",
                   help="POST target for SET/WIFI/EXEC (default %(default)s)")
    return p.parse_args()

def main()->None:
    a=_args()
    EdgeTXSniffer(port=a.port,baud=a.baud,period_ms=a.period,debug=a.debug,
                  command_parse=a.command_parse,min_change=a.min_change,
                  persist_ms=a.persist,pause_ms=a.pause,
                  channels_filter=_parse_filter(a.channels),
                  disable_penalty=a.disable_penalty,
                  exec_url=a.exec_url).run()

if __name__=="__main__":
    main()
