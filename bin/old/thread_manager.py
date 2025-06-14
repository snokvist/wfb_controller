#!/usr/bin/env python3
"""
Async Thread Manager & Command Server  – Python 3.9-compatible
=============================================================

• Pure-asyncio core (zero Python threads for I/O)
• Auto-restart, reload, placeholder substitution – same features as original
• Comma-separated & wildcard stream subscriptions:
      stream foo,bar stdout
      stream worker* all
      stream * stderr
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import binascii
import configparser
import fnmatch
import logging
import os
import shlex
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Set, Tuple

# ───────────────────────── constants ─────────────────────────
MAX_CONNECTIONS   = 30
IDLE_TIMEOUT      = 60          # seconds client may stay idle (if not streaming)
RESTART_DELAY     = 5           # delay before restart attempt
MISSING_RECHECK   = 30          # retry period for missing executables
QUEUE_HIGH_WATER  = 64          # max payloads buffered per client

HELP_TEXT = """\
THREAD QUERIES
list                      – names of all threads
status [thread|idx]       – status of all / one thread
info   [thread|idx]       – command line of all / one thread
stream <thread[,..]|*> <stdout|stderr|all>
list stream all           – stream every thread’s stdout+stderr
list settings             – show placeholder → value map

ONE-SHOT SHELL COMMANDS
list commands             – list programmable one-shot commands
exec <name|index>         – run one-shot command

THREAD CONTROL
start   <thread|idx|all>  – start stopped thread(s)
stop    <thread|idx|all>  – stop running thread(s)
restart <thread|idx|all>  – stop+start thread(s)
reload                    – re-read config, rebuild whole thread set

MISC
help                      – this help text
"""

LOG_FMT = "%(asctime)s %(levelname)s %(name)s %(message)s"
logger  = logging.getLogger("manager")

# ───────────────────── placeholder helpers ──────────────────────
def _escape_braces(s: str) -> str:
    return s.replace("{", "{{").replace("}", "}}")

def substitute(text: str, mapping: Mapping[str, str]) -> str:
    esc = {k: _escape_braces(v) for k, v in mapping.items()}
    try:
        out = text.format_map(esc)
    except KeyError as e:
        raise KeyError(f"Unknown placeholder {e.args[0]} in '{text}'") from None
    return out.replace("{{", "{").replace("}}", "}")

# ───────────────────── key material helper ─────────────────────
def ensure_keys(cfg: configparser.ConfigParser) -> None:
    if "settings" not in cfg:
        return
    settings = cfg["settings"]
    keys: Dict[str, Dict[str, str]] = {}
    for k, v in settings.items():
        if not k.startswith("key_"):
            continue
        parts = k.split("_", 2)          # key_<name>_rest
        if len(parts) != 3:
            continue
        _, name, rest = parts
        keys.setdefault(name, {})[rest] = v.strip()

    for name, ent in keys.items():
        # secret
        if ent.get("secret_b64") and ent.get("secret_path"):
            try:
                raw = base64.b64decode(ent["secret_b64"])
            except binascii.Error as e:
                sys.exit(f"Cannot decode base64 for key_{name}_secret_b64: {e}")
            p = Path(ent["secret_path"])
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(raw)
            p.chmod(0o600)
        # public
        if ent.get("public_b64") and ent.get("public_path"):
            try:
                raw = base64.b64decode(ent["public_b64"])
            except binascii.Error as e:
                sys.exit(f"Cannot decode base64 for key_{name}_public_b64: {e}")
            p = Path(ent["public_path"])
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(raw)
            p.chmod(0o644)

# ───────────────────── broadcaster ──────────────────────────────
class AsyncBroadcaster:
    """Fan-out stdout/stderr lines to subscribed clients."""

    def __init__(self) -> None:
        self._subs   : Dict[asyncio.StreamWriter, Set[Tuple[str,str]]|str] = {}
        self._queues : Dict[asyncio.StreamWriter, asyncio.Queue[bytes]]    = {}
        self._lock   = asyncio.Lock()
        self._sent   = 0
        self._dropped= 0

    async def add(self, w: asyncio.StreamWriter, spec) -> None:
        async with self._lock:
            self._subs[w] = spec
            q: asyncio.Queue[bytes] = asyncio.Queue(maxsize=QUEUE_HIGH_WATER)
            self._queues[w] = q
            asyncio.create_task(self._writer_loop(w, q))

    async def remove(self, w: asyncio.StreamWriter) -> None:
        async with self._lock:
            self._subs.pop(w, None)
            q = self._queues.pop(w, None)
            if q:
                q.put_nowait(b"__CLOSE__")

    async def publish(self, name: str, stream: str, line: str) -> None:
        payload = f"{name}:{line}".encode()
        async with self._lock:
            for w, spec in self._subs.items():
                want = spec == "ALL" or (isinstance(spec,set) and (name, stream) in spec)
                if not want:
                    continue
                q = self._queues[w]
                if q.full():
                    self._dropped += len(payload)
                    continue
                q.put_nowait(payload)
                self._sent += len(payload)

    async def _writer_loop(self, w: asyncio.StreamWriter, q: asyncio.Queue[bytes]):
        try:
            while True:
                chunk = await q.get()
                if chunk == b"__CLOSE__":
                    break
                try:
                    w.write(chunk)
                    await w.drain()
                except (ConnectionResetError,OSError):
                    break
        finally:
            try:
                w.close()
                await w.wait_closed()
            except Exception:
                pass

BCAST = AsyncBroadcaster()

# ───────────────────── managed process ──────────────────────────
class AsyncManagedProcess:
    def __init__(self, name: str, cmd: str, debug: bool) -> None:
        self.name, self.cmd, self.debug = name, cmd, debug
        self.proc       : Optional[asyncio.subprocess.Process] = None
        self.alive      = False
        self.manual_stop= False
        self.missing    = False
        self.last_miss  = 0.0
        self.restarts   = 0
        self._tasks     : List[asyncio.Task] = []

    async def start(self, *, force=False) -> None:
        if self.alive and not force:
            return
        if self.manual_stop and not force:
            return
        argv = shlex.split(self.cmd)
        try:
            # Python 3.9: no "text" parameter
            self.proc = await asyncio.create_subprocess_exec(
                *argv,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        except FileNotFoundError as e:
            self.alive = False
            self.missing = True
            self.last_miss = time.monotonic()
            logger.error("[MISSING] %s: %s", self.name, e)
            return

        self.alive   = True
        self.missing = False
        if self.debug:
            logger.info("START %s pid=%d", self.name, self.proc.pid)
        for reader, sname in ((self.proc.stdout,"stdout"),
                              (self.proc.stderr,"stderr")):
            self._tasks.append(asyncio.create_task(self._pump(reader,sname)))
        self._tasks.append(asyncio.create_task(self._waiter()))

    async def stop(self) -> None:
        self.manual_stop = True
        if not self.proc or not self.alive:
            return
        if self.proc.returncode is None:
            self.proc.terminate()
            try:
                await asyncio.wait_for(self.proc.wait(), timeout=3)
            except asyncio.TimeoutError:
                self.proc.kill()
                await self.proc.wait()
        self.alive = False
        for t in self._tasks:
            t.cancel()

    async def _pump(self, reader: asyncio.StreamReader, stream: str):
        try:
            async for line in reader:
                # decode bytes → str (replace errors)
                await BCAST.publish(self.name, stream, line.decode(errors="replace"))
        except asyncio.CancelledError:
            pass

    async def _waiter(self):
        if self.proc:
            await self.proc.wait()
        self.alive = False

    def status(self) -> str:
        if self.manual_stop: return "stopped"
        if self.missing:     return "missing"
        return "running" if self.alive else "error"

# ───────────────────── process manager ──────────────────────────
class ProcessManager:
    def __init__(self, procs: Dict[str,AsyncManagedProcess]):
        self.procs = procs
        self._mon_task: Optional[asyncio.Task] = None

    async def start_all(self):
        for p in self.procs.values():
            p.manual_stop = False
            await p.start(force=True)
        if not self._mon_task or self._mon_task.done():
            self._mon_task = asyncio.create_task(self._monitor())

    async def stop_all(self):
        for p in self.procs.values():
            await p.stop()

    async def shutdown(self):
        if self._mon_task:
            self._mon_task.cancel()
        await self.stop_all()

    async def _monitor(self):
        try:
            while True:
                await asyncio.sleep(1)
                for p in self.procs.values():
                    if p.manual_stop:
                        continue
                    if not p.alive and not p.missing:
                        await asyncio.sleep(RESTART_DELAY)
                        p.restarts += 1
                        await p.start()
                    elif p.missing and time.monotonic()-p.last_miss >= MISSING_RECHECK:
                        await p.start()
        except asyncio.CancelledError:
            pass

    # convenience ----------------------------------------------------
    def list_names(self): return list(self.procs.keys())

    def name_from_token(self, tok: str) -> str:
        order = self.list_names()
        if tok.isdigit() and 1<=int(tok)<=len(order):
            return order[int(tok)-1]
        return tok

    def status(self, n: Optional[str]=None):
        if n is None:
            return {k:p.status() for k,p in self.procs.items()}
        p = self.procs.get(n)
        return {n: p.status() if p else "unknown"}

    def info(self, n: Optional[str]=None):
        if n is None:
            return {k:p.cmd for k,p in self.procs.items()}
        p = self.procs.get(n)
        return {n: p.cmd if p else "unknown"}

    async def restart_thread(self, name:str):
        p = self.procs.get(name)
        if not p: return
        await p.stop()
        await asyncio.sleep(RESTART_DELAY)
        p.manual_stop=False
        await p.start()

    async def restart_all(self):
        for n in self.list_names():
            await self.restart_thread(n)

    async def stop_thread(self, name:str):
        p=self.procs.get(name)
        if p: await p.stop()

    async def start_thread(self, name:str):
        p=self.procs.get(name)
        if p:
            p.manual_stop=False
            await p.start(force=True)

    def restarts_total(self): return sum(p.restarts for p in self.procs.values())

# ───────────────────── one-shot command set ────────────────────────
class CommandSet:
    def __init__(self, cmds: List[Tuple[str,str]]):
        self.order=[n for n,_ in cmds]
        self.map  =dict(cmds)

    def list_names(self): return self.order
    def name_from_token(self,tok:str)->str:
        if tok.isdigit() and 1<=int(tok)<=len(self.order):
            return self.order[int(tok)-1]
        return tok
    def get(self,name:str): return self.map.get(name)

# ───────────────────── config loader ───────────────────────────────
async def load_config(path:str, debug:bool):
    cfg=configparser.ConfigParser(interpolation=None)
    if not cfg.read(path):
        sys.exit(f"Cannot read {path}")

    ensure_keys(cfg)
    settings = {k.strip():v.strip() for k,v in cfg["settings"].items()} if "settings" in cfg else {}

    if "threads" not in cfg:
        sys.exit("[threads] section missing in profile")

    procs: Dict[str,AsyncManagedProcess]={}
    for name,cmd in cfg["threads"].items():
        procs[name]=AsyncManagedProcess(name, substitute(cmd,settings), debug)

    cmd_list: List[Tuple[str,str]]=[]
    if "commands" in cfg:
        for name,cmd in cfg["commands"].items():
            cmd_list.append((name,substitute(cmd,settings)))

    pm=ProcessManager(procs)
    cs=CommandSet(cmd_list)
    settings_list=list(settings.items())
    return pm,cs,settings_list

# ───────────────────── client session ──────────────────────────────
class ClientSession:
    def __init__(self, rd:asyncio.StreamReader, wr:asyncio.StreamWriter,
                 pm:ProcessManager, cs:CommandSet, settings_list):
        self.r=rd ; self.w=wr
        self.pm=pm ; self.cs=cs ; self.settings=settings_list
        self.subscribed=False

    async def run(self):
        try:
            while True:
                try:
                    raw=await asyncio.wait_for(self.r.readline(), timeout=IDLE_TIMEOUT)
                except asyncio.TimeoutError:
                    if self.subscribed: continue
                    break
                if not raw: break
                line=raw.decode().strip()
                if not line: continue
                await self.dispatch(line)
        finally:
            await BCAST.remove(self.w)
            self.w.close()
            await self.w.wait_closed()

    async def send(self,data:str):
        self.w.write(data.encode())
        await self.w.drain()

    # ───────── command dispatch ─────────
    async def dispatch(self,line:str):
        parts=line.split()
        cmd=parts[0].lower()

        if cmd=="help":
            await self.send(HELP_TEXT); return
        if cmd=="list":
            await self._cmd_list(parts); return
        if cmd in {"status","info"}:
            await self._cmd_statusinfo(cmd,parts); return
        if cmd=="stream":
            await self._cmd_stream(parts); return
        if cmd=="exec":
            await self._cmd_exec(parts); return
        if cmd=="reload":
            await self._cmd_reload(parts); return
        if cmd in {"start","stop","restart"}:
            await self._cmd_lifecycle(cmd,parts); return
        await self.send("ERR Unknown command\n")

    # --- sub-commands ---------------------------------------------------
    async def _cmd_list(self,parts):
        if len(parts)==1:
            await self.send("\n".join(self.pm.list_names())+"\n"); return
        topic=parts[1]
        if topic=="status":
            d=self.pm.status(); await self.send("\n".join(f"{k}: {v}" for k,v in d.items())+"\n")
        elif topic=="info":
            d=self.pm.info();   await self.send("\n".join(f"{k}: {v}" for k,v in d.items())+"\n")
        elif topic=="commands":
            await self.send("\n".join(self.cs.list_names())+"\n")
        elif topic=="settings":
            await self.send("\n".join(f"{k}: {v}" for k,v in self.settings)+"\n")
        elif topic=="stream" and len(parts)>2 and parts[2]=="all":
            await BCAST.add(self.w,"ALL"); self.subscribed=True
            await self.send("STREAM ALL\n")
        else:
            await self.send("ERR Unknown list subcommand\n")

    async def _cmd_statusinfo(self,which,parts):
        if len(parts)==1:
            func=self.pm.status if which=="status" else self.pm.info
            d=func(); await self.send("\n".join(f"{k}: {v}" for k,v in d.items())+"\n"); return
        name=self.pm.name_from_token(parts[1])
        func=self.pm.status if which=="status" else self.pm.info
        d=func(name); k,v=next(iter(d.items()))
        await self.send(f"{k}: {v}\n")

    async def _cmd_stream(self,parts):
        if len(parts)!=3:
            await self.send("ERR stream <targets> <stdout|stderr|all>\n"); return
        tgt_str, spec = parts[1], parts[2]
        if spec not in {"stdout","stderr","all"}:
            tgt_str, spec = parts[2], parts[1]
            if spec not in {"stdout","stderr","all"}:
                await self.send("ERR stream <targets> <stdout|stderr|all>\n"); return

        # build subscription set
        if tgt_str=="*":
            subs="ALL"
        else:
            names=self.pm.list_names()
            wanted:set[Tuple[str,str]]=set()
            for pattern in tgt_str.split(","):
                pattern=pattern.strip()
                if not pattern: continue
                if pattern=="*":
                    subs="ALL"; break
                if any(x in pattern for x in "*?[]"):
                    matched=[n for n in names if fnmatch.fnmatch(n,pattern)]
                else:
                    matched=[self.pm.name_from_token(pattern)]
                for n in matched:
                    if n not in self.pm.procs: continue
                    if spec=="all":
                        wanted.update({(n,"stdout"),(n,"stderr")})
                    else:
                        wanted.add((n,spec))
            else:
                subs=wanted
        await BCAST.add(self.w, subs)
        self.subscribed=True
        await self.send(f"STREAM {tgt_str} {spec}\n")

    async def _cmd_exec(self,parts):
        if len(parts)!=2:
            await self.send("ERR exec <name|index>\n"); return
        name=self.cs.name_from_token(parts[1])
        cmd=self.cs.get(name)
        if cmd is None:
            await self.send("ERR No such command\n"); return
        try:
            # Python 3.9 – no text=True
            p=await asyncio.create_subprocess_shell(cmd,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.STDOUT)
            out,_=await asyncio.wait_for(p.communicate(),timeout=10)
            output=out.decode(errors="replace")
            if output: await self.send(output.rstrip()+"\n")
            await self.send(f"EXIT {p.returncode}\n")
        except Exception as e:
            await self.send(f"ERR exec failed: {e}\n")

    async def _cmd_reload(self,_):
        try:
            await reload_config()
            await self.send("RELOADED\n")
        except Exception as e:
            await self.send(f"ERR reload failed: {e}\n")

    async def _cmd_lifecycle(self,cmd,parts):
        if len(parts)!=2:
            await self.send(f"ERR {cmd} <thread|idx|all>\n"); return
        tgt=parts[1].lower()
        if tgt=="all":
            coro=getattr(self.pm,f"{cmd}_all")
            await coro(); await self.send(f"{cmd.upper()} ALL\n"); return
        name=self.pm.name_from_token(tgt)
        if name not in self.pm.procs:
            await self.send("ERR No such thread\n"); return
        coro=getattr(self.pm,f"{cmd}_thread")
        await coro(name)
        await self.send(f"{cmd.upper()} {name}\n")

# ───────────────────── reload helper ───────────────────────────────
async def reload_config():
    global PM,CS,SETTINGS_LIST
    await PM.shutdown()
    PM,CS,SETTINGS_LIST = await load_config(CONFIG_PATH, DEBUG_ENABLED)
    await PM.start_all()

# ───────────────────── main ─────────────────────────────────────────
PM: ProcessManager
CS: CommandSet
SETTINGS_LIST: List[Tuple[str,str]]
CONFIG_PATH=""
DEBUG_ENABLED=False
CLIENTS: set[asyncio.StreamWriter] = set() 

async def main():
    global PM,CS,SETTINGS_LIST,CONFIG_PATH,DEBUG_ENABLED

    if os.geteuid()!=0:
        sys.exit("Must be run as root (sudo).")

    ap=argparse.ArgumentParser()
    ap.add_argument("--config",required=True,help="Path to profile")
    ap.add_argument("--debug",action="store_true")
    ap.add_argument("--port",type=int,default=9500)
    args=ap.parse_args()

    DEBUG_ENABLED=args.debug
    CONFIG_PATH=args.config
    logging.basicConfig(level=logging.DEBUG if DEBUG_ENABLED else logging.INFO,
                        format=LOG_FMT)

    PM,CS,SETTINGS_LIST = await load_config(CONFIG_PATH, DEBUG_ENABLED)
    await PM.start_all()

    async def on_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        # real-time limit: count *connected* clients, not all asyncio tasks
        if len(CLIENTS) >= MAX_CONNECTIONS:
            writer.write(b"ERR Too many connections\n")
            await writer.drain()
            writer.close()
            await writer.wait_closed()
            return

        CLIENTS.add(writer)
        try:
            session = ClientSession(reader, writer, PM, CS, SETTINGS_LIST)
            await session.run()
        finally:
            CLIENTS.discard(writer)          # always release the slot

    server=await asyncio.start_server(on_client,"0.0.0.0",args.port)
    logger.info("Listening on port %d",args.port)

    stop_evt=asyncio.Event()
    loop=asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_evt.set)

    await stop_evt.wait()
    logger.info("Shutting down …")
    server.close(); await server.wait_closed()
    await PM.shutdown()

if __name__=="__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
