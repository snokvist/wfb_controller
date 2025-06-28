#!/usr/bin/env python3
"""
EdgeTX → RC_CHANNELS_OVERRIDE bridge + WiFi-health G-VAR
with automatic 3-second serial reconnect.
"""

from __future__ import annotations
import argparse, signal, subprocess, sys, threading, time, re
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import serial  # type: ignore
except ModuleNotFoundError:
    sys.stderr.write("pip install pyserial\n"); sys.exit(1)

# ────────── helpers & constants ────────────────────────────────────────────
MIN_EDGETX, MAX_EDGETX = -1024, 1024
MIN_PWM,   MAX_PWM     =  1000,  2000
_EDGETX_RANGE, _PWM_RANGE = 2048, 1000

RSSI_MIN_DBM, RSSI_MAX_DBM = -100, -30
RSSI_RANGE = RSSI_MAX_DBM - RSSI_MIN_DBM

SCRIPT = Path("/usr/bin/channels.sh")

r_pkt, r_rxant = re.compile(r"\bPKT\b"), re.compile(r"\bRX_ANT\b")

def _clamp(v:int,lo:int,hi:int)->int: return max(lo,min(hi,v))
def edgetx_to_pwm(v:int)->int:
    return int(round((_clamp(v,MIN_EDGETX,MAX_EDGETX)-MIN_EDGETX)
                     *_PWM_RANGE/_EDGETX_RANGE + MIN_PWM))
def rssi_to_gvar(rssi:int)->int:
    frac=(_clamp(rssi,RSSI_MIN_DBM,RSSI_MAX_DBM)-RSSI_MIN_DBM)/RSSI_RANGE
    return int(round(frac*2048-1024))

signal.signal(signal.SIGPIPE, signal.SIG_DFL)

# ────────── WiFi reader thread ─────────────────────────────────────────────
class WfbReader(threading.Thread):
    def __init__(self,sniffer:'EdgeTXSniffer')->None:
        super().__init__(daemon=True); self.sniffer=sniffer
    def run(self)->None:
        proc:Optional[subprocess.Popen[str]]=None
        try:
            proc=subprocess.Popen(["nc","127.0.0.1","9500"],
                                  stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                  text=True)
            assert proc.stdin; proc.stdin.write("wfb_rx\n"); proc.stdin.flush()
            best:Optional[int]=None
            while True:
                if proc.stdout is None: break
                line=proc.stdout.readline()
                if not line: break
                if r_rxant.search(line):
                    try: avg=int(line.split()[4].split(":")[2])
                    except (IndexError,ValueError): continue
                    if best is None or avg>best: best=avg
                elif r_pkt.search(line):
                    if best is not None: self.sniffer.send_gv(rssi_to_gvar(best))
                    best=None
        finally:
            if proc and proc.poll() is None: proc.kill()

# ────────── main sniffer ──────────────────────────────────────────────────
class EdgeTXSniffer:                                  # pylint: disable=too-many-instance-attributes
    _POLL_SLEEP=0.01
    _RETRY_SEC = 3
    def __init__(self,*,port:str,baud:int,period_ms:int,
                 command_parse:bool,min_change:int,persist_ms:int,
                 pause_ms:int,channels_filter:Optional[set[int]])->None:
        self.port, self.baud = port, baud
        self.ser:Optional[serial.Serial]=None
        self.serial_lock=threading.Lock()
        self._connect_serial()                       # blocks until OK

        self.period_ms=max(1,period_ms); self.period_sec=self.period_ms/1000
        self._start=time.time(); self._next=self._start+self.period_sec

        self.latest_pwm=[1500]*16; self.rc_cnt=0
        self.cmd_parse, self.min_change, self.persist_ms, self.pause_ms = \
            command_parse, min_change, persist_ms, pause_ms
        self.allowed=channels_filter
        self.pending:[Optional[Dict[str,Any]]]=[None]*16
        self.last:[Optional[int]]=[None]*16
        self.next_allowed_ms=0; self.exec_idx=0
        self.idx_lock=threading.Lock()

        print(f"[INFO] {self.port}@{self.baud} agg={self.period_ms}ms "
              f"cmd_parse={self.cmd_parse}", flush=True)

        WfbReader(self).start()

    # ───── serial (re)connect helpers ─────
    def _connect_serial(self)->None:
        while True:
            try:
                self.ser=serial.Serial(self.port,self.baud,timeout=0.1)
                print(f"[INFO] Serial connected", flush=True)
                return
            except serial.SerialException as exc:
                print(f"[WARN] Serial open failed: {exc} – retrying in "
                      f"{self._RETRY_SEC}s", flush=True)
                time.sleep(self._RETRY_SEC)

    def _reset_serial(self)->None:
        with self.serial_lock:
            if self.ser:
                try: self.ser.close()
                except Exception: pass                   # already dead
            self.ser=None
        print(f"[WARN] Serial lost – reconnecting …", flush=True)
        self._connect_serial()

    # ───── GV sender (from WiFi thread) ─────
    def send_gv(self,val:int)->None:
        with self.serial_lock:
            if self.ser is None: return
            try:
                self.ser.write(f"GV,0,{val}\n".encode())
                print(f"[>>] GV,0,{val} (from RSSI)", flush=True)
            except serial.SerialException:
                self._reset_serial()

    # ───── main loop ─────
    def run(self)->None:                               # pylint: disable=too-many-branches
        try:
            while True:
                now=time.time()

                # 1) read from serial (with auto-reconnect)
                line=""
                if self.ser:
                    try:
                        line=self.ser.readline().decode(errors="replace").strip()
                    except serial.SerialException:
                        self._reset_serial()
                if line.startswith("CH,"):
                    self._handle_ch(line)

                # 2) periodic RC_CHANNELS_OVERRIDE
                if now>=self._next:
                    if self.rc_cnt:
                        self._print_override(); self.rc_cnt=0
                    self._next+=self.period_sec

                if not line:
                    time.sleep(self._POLL_SLEEP)
        except KeyboardInterrupt:
            print("\n[INFO] Terminated", flush=True)

    # ───── helpers (unchanged, but always flush) ─────
    def _handle_ch(self,line:str)->None:
        parts=line.split(","); 
        if len(parts)!=17: return
        try: raws=[int(x) for x in parts[1:]]
        except ValueError: return
        self.latest_pwm=[edgetx_to_pwm(v) for v in raws]; self.rc_cnt+=1
        if self.cmd_parse: self._eval_channels(self.latest_pwm)

    def _print_override(self)->None:
        ts=self._now_ms()
        print(f"{ts} RC_CHANNELS_OVERRIDE {self.rc_cnt} 0 0 "
              f"{':'.join(map(str,self.latest_pwm))}", flush=True)

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
        if now_ms<self.next_allowed_ms: return False
        if not SCRIPT.exists():
            sys.stderr.write(f"{SCRIPT} missing\n"); return False
        with self.idx_lock: self.exec_idx+=1; idx=self.exec_idx
        def work()->None:
            start=time.time()
            try:
                res=subprocess.run([str(SCRIPT),str(ch),str(new)],capture_output=True)
                rt=int(round((time.time()-start)*1000))
                if res.returncode!=0:
                    sys.stderr.write(f"channels.sh err ch{ch}: "
                                     f"{res.stderr.decode(errors='ignore')}\n")
            except Exception as exc:
                sys.stderr.write(f"exec fail {exc}\n"); return
            ts=int((time.time()-self._start)*1000)
            print(f"{ts} MAVLINK_EXEC {idx}:{ch}:{old}:{new}:{rt}", flush=True)
        threading.Thread(target=work,daemon=True).start()
        self.next_allowed_ms=now_ms+self.pause_ms; return True

    def _now_ms(self)->int: return int((time.time()-self._start)*1000)

# ────────── CLI plumbing ───────────────────────────────────────────
def _parse_filter(txt:Optional[str])->Optional[set[int]]:
    if not txt: return None
    try: s={int(x) for x in txt.split(',') if x.strip()}
    except ValueError: sys.stderr.write("bad --channels\n"); sys.exit(1)
    if any(not 1<=c<=16 for c in s): sys.stderr.write("channels 1-16\n"); sys.exit(1)
    return s
def _args()->argparse.Namespace:
    p=argparse.ArgumentParser("EdgeTX RC bridge + WiFi GVAR + auto-reconnect")
    p.add_argument("--port",default="/dev/ttyACM0"); p.add_argument("--baud",type=int,default=115200)
    p.add_argument("--period",type=int,default=1000); p.add_argument("--command-parse",action="store_true")
    p.add_argument("--min-change",type=int,default=100); p.add_argument("--persist",type=int,default=500)
    p.add_argument("--pause",type=int,default=500); p.add_argument("--channels")
    return p.parse_args()
def main()->None:
    a=_args()
    EdgeTXSniffer(port=a.port,baud=a.baud,period_ms=a.period,
                  command_parse=a.command_parse,min_change=a.min_change,
                  persist_ms=a.persist,pause_ms=a.pause,
                  channels_filter=_parse_filter(a.channels)).run()
if __name__=="__main__":
    main()
