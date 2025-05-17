#!/usr/bin/env python3
"""
Thread Manager + One-Shot Command Executor
-----------------------------------------
∙ All previous functionality remains (threads, streaming, help, debug,
  graceful Ctrl-C).
∙ NEW: `[commands]` section for fire-and-forget shell commands.

TCP additions
-------------
list commands            – list command names
exec <name|index>        – run once, return exit code + any output
"""

from __future__ import annotations
import argparse, base64, binascii, configparser, os, shlex, signal, socket
import socketserver, subprocess, sys, threading, time
from pathlib import Path
from typing import Dict, Set, Tuple, List

DEBUG_LINE_TEMPLATE = "RUN:{run} RST:{rst} TX:{tx} DROP:{drop}"
MAX_CONNECTIONS, IDLE_TIMEOUT = 10, 60
RESTART_DELAY, MISSING_RECHECK = 5, 30
STOP_EVENT = threading.Event()
DEBUG_ENABLED = False

def dlog(msg:str):
    if DEBUG_ENABLED: sys.stderr.write(f"[SHUT] {msg}\n"); sys.stderr.flush()

HELP_TEXT = """\
list                         – show thread names
list status                  – status of all threads
list info                    – command line of all threads
list stream all              – stream all stdout+stderr
list commands                – list one-shot commands
status <thread|index>        – status of one thread
info <thread|index>          – command line of one thread
stream <thread|index> (stdout|stderr|all)
exec   <command|index>       – run one-shot command
help                         – this message
"""

# ──────────────────────────── ManagedProcess (unchanged) ────────────────────
class ManagedProcess:
    def __init__(self,name:str,cmd:str,debug:bool):
        self.name,self.cmd,self.debug=name,cmd,debug
        self.proc:subprocess.Popen|None=None; self.restarts=0
        self.alive=False; self.missing_exec=False; self._last_missing=0.0
        self._lock=threading.Lock()
    def start(self):
        with self._lock:
            if self.missing_exec and time.time()-self._last_missing<MISSING_RECHECK: return
            try:
                self.proc=subprocess.Popen(shlex.split(self.cmd),
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
            except FileNotFoundError as e:
                self.alive=False; self.missing_exec=True; self._last_missing=time.time()
                print(f"[MISSING] {self.name}: {e}",file=sys.stderr); return
            except Exception as e:
                self.alive=False; print(f"[ERROR] Could not start {self.name}: {e}",file=sys.stderr); return
            self.alive=True; self.missing_exec=False
            if self.debug: print(f"[START] {self.name} pid={self.proc.pid}\n        cmd: {self.cmd}",file=sys.stderr,flush=True)
            threading.Thread(target=self._pump,args=(self.proc.stdout,"stdout"),daemon=True).start()
            threading.Thread(target=self._pump,args=(self.proc.stderr,"stderr"),daemon=True).start()
    def stop(self,kill=False):
        with self._lock:
            if not self.proc: return
            status="TERM OK"
            if self.proc.poll() is None:
                try: self.proc.terminate(); self.proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    if kill: self.proc.kill(); status="KILL"
            dlog(f"    • {self.name} pid {self.proc.pid} -> {status}")
            for p in (self.proc.stdout,self.proc.stderr):
                try: p.close()
                except Exception: pass
            self.alive=False
    def _pump(self,pipe,stream):
        for line in iter(pipe.readline,""): Broadcaster.publish(self.name,stream,line)
        pipe.close()
    def is_running(self): return self.alive and self.proc and self.proc.poll() is None
    def status(self):     return "missing" if self.missing_exec else ("running" if self.is_running() else "error")

# ─────────────────────────────── Broadcaster ────────────────────────────────
class Broadcaster:
    _listeners: Dict[socket.socket, Set[Tuple[str, str]] | str] = {}
    _sent_bytes = _dropped_bytes = 0
    _lock = threading.Lock()

    @classmethod
    def add_listener(cls, sock, subs):
        with cls._lock:
            cls._listeners[sock] = subs

    @classmethod
    def remove_listener(cls, sock):
        with cls._lock:
            cls._listeners.pop(sock, None)

    @classmethod
    def publish(cls, name: str, stream: str, data: str):
        payload = f"{name}:{data}".encode()
        delivered, dead = 0, []
        with cls._lock:
            for sock, subs in cls._listeners.items():
                if subs == "ALL" or (isinstance(subs, set) and (name, stream) in subs):
                    try:
                        sock.sendall(payload); delivered += 1
                    except Exception:
                        dead.append(sock)
            for d in dead:
                cls._listeners.pop(d, None)

        if delivered:
            cls._sent_bytes += delivered * len(payload)
        else:
            cls._dropped_bytes += len(payload)

    @classmethod
    def sent_bytes(cls):
        with cls._lock:
            return cls._sent_bytes

    @classmethod
    def dropped_bytes(cls):
        with cls._lock:
            return cls._dropped_bytes


# ───────────────────────────── CommandSet (NEW) ─────────────────────────────
class CommandSet:
    def __init__(self,cmds:List[Tuple[str,str]]):
        self.order=[n for n,_ in cmds]
        self.map=dict(cmds)
    def list_names(self): return self.order
    def name_from_token(self,tok:str):
        return self.order[int(tok)-1] if tok.isdigit() and 1<=int(tok)<=len(self.order) else tok
    def get(self,name:str): return self.map.get(name)

# ───────────────────────────── ThreadManager (unchanged) ────────────────────
class ThreadManager:
    def __init__(self,procs:Dict[str,ManagedProcess],order:List[str]):
        self.procs,self.order=procs,order; self._mon=threading.Thread(target=self._monitor,daemon=True)
    def start_all(self): [p.start() for p in self.procs.values()]; self._mon.start()
    def stop_all(self):  [p.stop(kill=True) for p in self.procs.values()]
    def _monitor(self):
        while not STOP_EVENT.is_set():
            time.sleep(1)
            for p in self.procs.values():
                if STOP_EVENT.is_set(): break
                if not p.is_running() and not p.missing_exec:
                    time.sleep(RESTART_DELAY); p.restarts+=1; p.start()
                elif p.missing_exec and time.time()-p._last_missing>=MISSING_RECHECK:
                    p.start()
    # helpers
    def list_names(self): return self.order
    def name_from_token(self,tok): return self.order[int(tok)-1] if tok.isdigit() and 1<=int(tok)<=len(self.order) else tok
    def status(self,n=None): return {k:p.status() for k,p in self.procs.items()} if n is None else {n:self.procs.get(n).status() if n in self.procs else "unknown"}
    def info(self,n=None):   return {k:p.cmd   for k,p in self.procs.items()} if n is None else {n:self.procs.get(n).cmd    if n in self.procs else "unknown"}
    def restarts_total(self):return sum(p.restarts for p in self.procs.values())
    def running_count(self): return sum(1 for p in self.procs.values() if p.is_running())

# ──────────────────────────── TCP handler  (updates) ────────────────────────
class CommandHandler(socketserver.BaseRequestHandler):
    def handle(self):
        sock=self.request
        if len(self.server.connections)>=MAX_CONNECTIONS:
            sock.sendall(b"ERR Too many connections\n"); return
        self.server.connections.add(sock); sock.settimeout(IDLE_TIMEOUT)
        subs=None
        try:
            while not STOP_EVENT.is_set():
                try: raw=sock.recv(1024)
                except socket.timeout:
                    if subs: continue
                    break
                if not raw: break
                if subs and raw in {b"\n",b"\r\n",b"\r"}:
                    Broadcaster.remove_listener(sock); subs=None; sock.sendall(b"STREAM stopped\n"); continue
                line=raw.decode().strip()
                if not line: continue
                parts=line.split(); cmd=parts[0].lower()

                if cmd=="help": sock.sendall(HELP_TEXT.encode()); continue

                if cmd=="list":
                    if len(parts)==1:
                        sock.sendall("\n".join(TM.list_names()).encode()+b"\n")
                    elif parts[1]=="status":
                        sock.sendall(("\n".join(f"{k}: {v}" for k,v in TM.status().items())+"\n").encode())
                    elif parts[1]=="info":
                        sock.sendall(("\n".join(f"{k}: {v}" for k,v in TM.info().items())+"\n").encode())
                    elif parts[1]=="commands":
                        sock.sendall("\n".join(CS.list_names()).encode()+b"\n")
                    elif len(parts)>2 and parts[1]=="stream" and parts[2]=="all":
                        subs="ALL"; Broadcaster.add_listener(sock,"ALL"); sock.sendall(b"STREAM ALL\n")
                    else: sock.sendall(b"ERR Unknown list subcommand\n")
                    continue

                if cmd in {"status","info"} and len(parts)==2:
                    name=TM.name_from_token(parts[1])
                    func=TM.status if cmd=="status" else TM.info
                    k,v=next(iter(func(name).items())); sock.sendall(f"{k}: {v}\n".encode()); continue

                if cmd=="stream" and len(parts)==3:
                    token,spec=parts[1],parts[2]
                    if spec not in {"stdout","stderr","all"}:
                        token,spec = parts[2],parts[1]
                    if spec not in {"stdout","stderr","all"}:
                        sock.sendall(b"ERR stream <thread|index> <stdout|stderr|all>\n"); continue
                    name=TM.name_from_token(token)
                    if name not in TM.procs: sock.sendall(b"ERR No such thread\n"); continue
                    subs={(name,"stdout"),(name,"stderr")} if spec=="all" else {(name,spec)}
                    Broadcaster.add_listener(sock,subs)
                    sock.sendall(f"STREAM {name} {spec}\n".encode()); continue

                if cmd=="exec" and len(parts)==2:
                    name=CS.name_from_token(parts[1])
                    cmline=CS.get(name)
                    if cmline is None: sock.sendall(b"ERR No such command\n"); continue
                    try:
                        res=subprocess.run(cmline,shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=10)
                        output=res.stdout.rstrip()
                        sock.sendall((output+"\n").encode() if output else b"")
                        sock.sendall(f"EXIT {res.returncode}\n".encode())
                    except Exception as e:
                        sock.sendall(f"ERR exec failed: {e}\n".encode())
                    continue

                sock.sendall(b"ERR Unknown command\n")
        finally:
            Broadcaster.remove_listener(sock)
            self.server.connections.discard(sock)
            try: sock.shutdown(socket.SHUT_RDWR)
            except Exception: pass
            sock.close()

# ───────────────────────────── TCP server class ─────────────────────────────
class ThreadedTCPServer(socketserver.ThreadingMixIn,socketserver.TCPServer):
    daemon_threads=True; allow_reuse_address=True
    def __init__(self,*a,**kw): super().__init__(*a,**kw); self.connections:Set[socket.socket]=set()

# ───────────────────────────── debug ticker ────────────────────────────────
def _debug_loop():
    while not STOP_EVENT.is_set():
        sys.stderr.write(DEBUG_LINE_TEMPLATE.format(
            run=TM.running_count(), rst=TM.restarts_total(),
            tx=Broadcaster.sent_bytes(), drop=Broadcaster.dropped_bytes())+"\n")
        sys.stderr.flush(); time.sleep(1)

# ───────────────────────────── config helpers ──────────────────────────────
def ensure_key(cfg,key:Path)->Path:
    if key.exists(): return key
    raw=base64.b64decode(cfg["settings"]["private_key_b64"].strip())
    key.parent.mkdir(parents=True,exist_ok=True); key.write_bytes(raw); key.chmod(0o600); return key

def load(cfg,key:Path,debug:bool):
    if "threads" not in cfg: sys.exit("[threads] missing")
    order,procs=[],{}
    for name,cmd in cfg["threads"].items():
        order.append(name); procs[name]=ManagedProcess(name,cmd.replace("{priv_key_path}",str(key)),debug)
    # commands may be absent
    cmd_list=[(n,c.replace("{priv_key_path}",str(key))) for n,c in (cfg["commands"].items() if "commands" in cfg else [])]
    return procs,order,cmd_list

# ─────────────────────────────────── main ───────────────────────────────────
def main():
    global DEBUG_ENABLED, TM, CS
    if os.geteuid()!=0: sys.exit("Must be run as root (sudo).")

    pa=argparse.ArgumentParser(); pa.add_argument("--config",required=True); pa.add_argument("--key-path",required=True)
    pa.add_argument("--debug",action="store_true"); pa.add_argument("--port",type=int,default=9500)
    args=pa.parse_args(); DEBUG_ENABLED=args.debug

    cfg=configparser.ConfigParser(interpolation=None)
    if not cfg.read(args.config): sys.exit(f"Cannot read {args.config}")
    key_path=ensure_key(cfg,Path(args.key_path))

    procs,order,cmd_list=load(cfg,key_path,args.debug)
    TM=ThreadManager(procs,order); TM.start_all()
    CS=CommandSet(cmd_list)

    if args.debug: threading.Thread(target=_debug_loop,daemon=True).start()
    server=ThreadedTCPServer(("0.0.0.0",args.port),CommandHandler)

    def shutdown(sig,frm):
        dlog("signal received, shutting down …")
        if STOP_EVENT.is_set(): return
        STOP_EVENT.set()
        for s in list(server.connections):
            try: s.shutdown(socket.SHUT_RDWR)
            except Exception: pass
            s.close()
        threading.Thread(target=server.shutdown,daemon=True).start()
        TM.stop_all()
        dlog("bye!"); time.sleep(0.5); os._exit(0)

    signal.signal(signal.SIGINT,shutdown); signal.signal(signal.SIGTERM,shutdown)
    try: server.serve_forever()
    except KeyboardInterrupt: shutdown(signal.SIGINT,None)

if __name__=="__main__": main()
