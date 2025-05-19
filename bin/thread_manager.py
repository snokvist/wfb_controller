#!/usr/bin/env python3
"""
Thread Manager + One-Shot Command Executor
==========================================

• Managed threads launched from `[threads]` in the INI profile.
• Fire-and-forget shell commands taken from `[commands]`.
• Placeholder **{tokens}** in both sections are substituted using every key
  from `[settings]`; unknown placeholders raise a *KeyError* at start-up.

─────────────────────────────────────────────────────────────────────────────
NEW MANAGEMENT COMMANDS (TCP)
─────────────────────────────────────────────────────────────────────────────
reload                       – stop **all** threads, reread the profile,  
                               rebuild thread list and commands  
restart  all|thread|index    – stop + start running threads (live config)  
stop     all|thread|index    – stop running thread(s) and mark *manual-stop*  
start    all|thread|index    – (re)start manually-stopped thread(s)

Manually stopped threads are **not** auto-restarted by the watchdog until
`start` or `restart` is issued: no more surprise respawns or double instances.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import configparser
import errno
import os
import shlex
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Set, Tuple

DEBUG_LINE_TEMPLATE = "RUN:{run} RST:{rst} TX:{tx} DROP:{drop}"
MAX_CONNECTIONS, IDLE_TIMEOUT = 10, 60
RESTART_DELAY, MISSING_RECHECK = 5, 30
STOP_EVENT = threading.Event()
DEBUG_ENABLED = False

# Globals replaced on reload ───────────────────────────────────────────
TM: "ThreadManager"
CS: "CommandSet"
SETTINGS_LIST: List[Tuple[str, str]] = []
CONFIG_PATH: str

# ───────────────────────────── helpers ────────────────────────────────
def dlog(msg: str):
    if DEBUG_ENABLED:
        sys.stderr.write(f"[SHUT] {msg}\n")
        sys.stderr.flush()


def _escape_braces(s: str) -> str:  # single-level substitution helper
    return s.replace("{", "{{").replace("}", "}}")


# ───────────────────────── helper: placeholder substitution ──────────
def substitute_placeholders(text: str, mapping: Dict[str, str]) -> str:
    """
    Single-pass replacement of {placeholders} using *mapping* while
    leaving any braces that already live *inside* the mapping values
    untouched.  
    """
    esc = {k: _escape_braces(v) for k, v in mapping.items()}
    try:
        out = text.format_map(esc)
    except KeyError as e:
        raise KeyError(f"Unknown placeholder {e.args[0]} in '{text}'") from None
    return out.replace("{{", "{").replace("}}", "}")


# ─────────────────────────── Key loading ──────────────────────────────
def ensure_keys(cfg: configparser.ConfigParser):
    """
    Find all key_<name>_secret_b64 / _public_b64 and key_<name>_secret_path / _public_path
    entries in [settings], decode the base64 values, and overwrite the files.
    """
    if "settings" not in cfg:
        return

    settings = cfg["settings"]
    # collect per-key entries
    key_entries: Dict[str, Dict[str, str]] = {}
    for k, v in settings.items():
        if not k.startswith("key_"):
            continue
        parts = k.split("_", 2)  # ["key", "<name>", "<rest>"]
        if len(parts) != 3:
            continue
        _, name, rest = parts
        key_entries.setdefault(name, {})[rest] = v.strip()

    for name, ent in key_entries.items():
        # secret key
        secret_b64 = ent.get("secret_b64")
        secret_path = ent.get("secret_path")
        if secret_b64 and secret_path:
            try:
                raw = base64.b64decode(secret_b64)
            except binascii.Error as e:
                sys.exit(f"Cannot decode base64 for key_{name}_secret_b64: {e}")
            p = Path(secret_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(raw)
            p.chmod(0o600)

        # public key
        public_b64 = ent.get("public_b64")
        public_path = ent.get("public_path")
        if public_b64 and public_path:
            try:
                raw = base64.b64decode(public_b64)
            except binascii.Error as e:
                sys.exit(f"Cannot decode base64 for key_{name}_public_b64: {e}")
            p = Path(public_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(raw)
            p.chmod(0o644)


# ───────────────────────── ManagedProcess ─────────────────────────────
class ManagedProcess:
    def __init__(self, name: str, cmd: str, debug: bool):
        self.name, self.cmd, self.debug = name, cmd, debug
        self.proc: subprocess.Popen | None = None
        self.restarts = 0
        self.alive = False
        self.manual_stop = False
        self.missing_exec = False
        self._last_missing = 0.0
        self._lock = threading.Lock()

    def start(self, force: bool = False):
        with self._lock:
            if self.is_running():
                return
            if self.manual_stop and not force:
                return
            try:
                self.proc = subprocess.Popen(
                    shlex.split(self.cmd),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                )
            except FileNotFoundError as e:
                self.alive = False
                self.missing_exec = True
                self._last_missing = time.time()
                print(f"[MISSING] {self.name}: {e}", file=sys.stderr)
                return
            except Exception as e:
                self.alive = False
                print(f"[ERROR] Could not start {self.name}: {e}", file=sys.stderr)
                return
            self.alive = True
            self.missing_exec = False
            if self.debug:
                print(
                    f"[START] {self.name} pid={self.proc.pid}\n        cmd: {self.cmd}",
                    file=sys.stderr,
                    flush=True,
                )
            threading.Thread(
                target=self._pump, args=(self.proc.stdout, "stdout"), daemon=True
            ).start()
            threading.Thread(
                target=self._pump, args=(self.proc.stderr, "stderr"), daemon=True
            ).start()

    def stop(self, kill: bool = False, manual: bool = False):
        with self._lock:
            if manual:
                self.manual_stop = True
            if not self.proc:
                return
            status = "TERM OK"
            if self.proc.poll() is None:
                try:
                    self.proc.terminate()
                    self.proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        self.proc.kill()
                        self.proc.wait(timeout=3)
                        status = "KILL"
                    except subprocess.TimeoutExpired:
                        try:
                            os.kill(self.proc.pid, 9)
                            status = "KILL-9"
                        except Exception:
                            status = "UNSTOPPABLE"
            dlog(f"    • {self.name} pid {self.proc.pid} -> {status}")
            for p in (self.proc.stdout, self.proc.stderr):
                try:
                    p.close()
                except Exception:
                    pass
            self.alive = False

    def _pump(self, pipe, stream):
        for line in iter(pipe.readline, ""):
            Broadcaster.publish(self.name, stream, line)
        pipe.close()

    def is_running(self):
        return self.alive and self.proc and self.proc.poll() is None

    def status(self):
        if self.manual_stop:
            return "stopped"
        if self.missing_exec:
            return "missing"
        return "running" if self.is_running() else "error"


# ─────────────────────────── Broadcaster ──────────────────────────────
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
                        sock.sendall(payload)
                        delivered += 1
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


# ─────────────────────────── CommandSet ───────────────────────────────
class CommandSet:
    def __init__(self, cmds: List[Tuple[str, str]]):
        self.order = [n for n, _ in cmds]
        self.map = dict(cmds)

    def list_names(self):
        return self.order

    def name_from_token(self, tok: str):
        return (
            self.order[int(tok) - 1]
            if tok.isdigit() and 1 <= int(tok) <= len(self.order)
            else tok
        )

    def get(self, name: str):
        return self.map.get(name)


# ─────────────────────────── ThreadManager ────────────────────────────
class ThreadManager:
    def __init__(self, procs: Dict[str, ManagedProcess], order: List[str]):
        self.procs, self.order = procs, order
        self._active = True
        self._mon = threading.Thread(target=self._monitor, daemon=True)

    def start_all(self):
        for p in self.procs.values():
            p.manual_stop = False
            p.start(force=True)
        if not self._mon.is_alive():
            self._mon.start()

    def stop_all(self, manual: bool = False):
        for p in self.procs.values():
            p.stop(kill=True, manual=manual)

    def shutdown(self):
        self._active = False

    def _monitor(self):
        while not STOP_EVENT.is_set() and self._active:
            time.sleep(1)
            for p in self.procs.values():
                if STOP_EVENT.is_set() or not self._active:
                    break
                if p.manual_stop:
                    continue
                if not p.is_running() and not p.missing_exec:
                    time.sleep(RESTART_DELAY)
                    p.restarts += 1
                    p.start()
                elif p.missing_exec and time.time() - p._last_missing >= MISSING_RECHECK:
                    p.start()

    def stop_thread(self, name: str):
        if name in self.procs:
            self.procs[name].stop(kill=True, manual=True)

    def start_thread(self, name: str):
        if name in self.procs:
            self.procs[name].manual_stop = False
            self.procs[name].start(force=True)

    def restart_thread(self, name: str):
        if name in self.procs:
            self.stop_thread(name)
            time.sleep(RESTART_DELAY)
            self.start_thread(name)

    def restart_all(self):
        for n in list(self.order):
            self.restart_thread(n)

    def list_names(self):
        return self.order

    def name_from_token(self, tok):
        return (
            self.order[int(tok) - 1]
            if tok.isdigit() and 1 <= int(tok) <= len(self.order)
            else tok
        )

    def status(self, n=None):
        return (
            {k: p.status() for k, p in self.procs.items()}
            if n is None
            else {n: self.procs.get(n).status() if n in self.procs else "unknown"}
        )

    def info(self, n=None):
        return (
            {k: p.cmd for k, p in self.procs.items()}
            if n is None
            else {n: self.procs.get(n).cmd if n in self.procs else "unknown"}
        )

    def restarts_total(self):
        return sum(p.restarts for p in self.procs.values())

    def running_count(self):
        return sum(1 for p in self.procs.values() if p.is_running())


# ──────────────────────────── TCP HELP text ───────────────────────────
HELP_TEXT = """\
THREAD QUERIES
  list                      – names of all threads
  status [thread|index]     – status of all / one thread
  info   [thread|index]     – command line of all / one thread
  stream <thread|idx> <stdout|stderr|all>
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


# ───────────────────────── config helpers ─────────────────────────────
def load_sections(cfg: configparser.ConfigParser, mapping: Dict[str, str], debug: bool):
    if "threads" not in cfg:
        sys.exit("[threads] section missing in profile")

    order: List[str] = []
    procs: Dict[str, ManagedProcess] = {}
    cmd_list: List[Tuple[str, str]] = []

    for name, cmd in cfg["threads"].items():
        try:
            full_cmd = substitute_placeholders(cmd, mapping)
        except KeyError as e:
            sys.exit(str(e))
        procs[name] = ManagedProcess(name, full_cmd, debug)
        order.append(name)

    if "commands" in cfg:
        for name, cmd in cfg["commands"].items():
            try:
                full_cmd = substitute_placeholders(cmd, mapping)
            except KeyError as e:
                sys.exit(str(e))
            cmd_list.append((name, full_cmd))

    return procs, order, cmd_list


def init_from_config(config_path: str, debug: bool, first_run: bool = False):
    cfg = configparser.ConfigParser(interpolation=None)
    if not cfg.read(config_path):
        sys.exit(f"Cannot read {config_path}")

    # process and write out all keys from [settings]
    ensure_keys(cfg)

    # build placeholder mapping from settings
    settings_map = (
        {k.strip(): v.strip() for k, v in cfg["settings"].items()}
        if "settings" in cfg
        else {}
    )

    procs, order, cmd_list = load_sections(cfg, settings_map, debug)
    new_TM = ThreadManager(procs, order)
    new_CS = CommandSet(cmd_list)
    new_SETTINGS = list(settings_map.items())

    return new_TM, new_CS, new_SETTINGS


# ─────────────────────────── CommandHandler ───────────────────────────
class CommandHandler(socketserver.BaseRequestHandler):
    def handle(self):
        sock = self.request
        if len(self.server.connections) >= MAX_CONNECTIONS:
            sock.sendall(b"ERR Too many connections\n")
            return

        self.server.connections.add(sock)
        sock.settimeout(IDLE_TIMEOUT)
        subs = None

        try:
            while not STOP_EVENT.is_set():
                try:
                    raw = sock.recv(1024)
                except socket.timeout:
                    if subs:
                        continue
                    break
                except OSError as e:
                    if e.errno == errno.EBADF:
                        return
                    raise

                if not raw:
                    break

                if subs and raw in {b"\n", b"\r\n", b"\r"}:
                    Broadcaster.remove_listener(sock)
                    subs = None
                    sock.sendall(b"STREAM stopped\n")
                    continue

                line = raw.decode().strip()
                if not line:
                    continue
                parts = line.split()
                cmd = parts[0].lower()

                # help -------------------------------------------------
                if cmd == "help":
                    sock.sendall(HELP_TEXT.encode())
                    continue

                # list -------------------------------------------------
                if cmd == "list":
                    if len(parts) == 1:
                        sock.sendall(("\n".join(TM.list_names()) + "\n").encode())
                    elif parts[1] == "status":
                        sock.sendall(
                            (
                                "\n".join(f"{k}: {v}" for k, v in TM.status().items())
                                + "\n"
                            ).encode()
                        )
                    elif parts[1] == "info":
                        sock.sendall(
                            (
                                "\n".join(f"{k}: {v}" for k, v in TM.info().items())
                                + "\n"
                            ).encode()
                        )
                    elif parts[1] == "commands":
                        sock.sendall(("\n".join(CS.list_names()) + "\n").encode())
                    elif parts[1] == "settings":
                        sock.sendall(
                            ("\n".join(f"{k}: {v}" for k, v in SETTINGS_LIST) + "\n").encode()
                        )
                    elif len(parts) > 2 and parts[1] == "stream" and parts[2] == "all":
                        subs = "ALL"
                        Broadcaster.add_listener(sock, "ALL")
                        sock.sendall(b"STREAM ALL\n")
                    else:
                        sock.sendall(b"ERR Unknown list subcommand\n")
                    continue

                # status / info ---------------------------------------
                if cmd in {"status", "info"}:
                    if len(parts) == 1:
                        func = TM.status if cmd == "status" else TM.info
                        sock.sendall(
                            ("\n".join(f"{k}: {v}" for k, v in func().items()) + "\n").encode()
                        )
                    elif len(parts) == 2:
                        name = TM.name_from_token(parts[1])
                        func = TM.status if cmd == "status" else TM.info
                        k, v = next(iter(func(name).items()))
                        sock.sendall(f"{k}: {v}\n".encode())
                    else:
                        sock.sendall(f"ERR {cmd} takes zero or one argument\n".encode())
                    continue

                # stream ----------------------------------------------
                if cmd == "stream" and len(parts) == 3:
                    token, spec = parts[1], parts[2]
                    if spec not in {"stdout", "stderr", "all"}:
                        token, spec = parts[2], parts[1]
                    if spec not in {"stdout", "stderr", "all"}:
                        sock.sendall(b"ERR stream <thread|index> <stdout|stderr|all>\n")
                        continue
                    name = TM.name_from_token(token)
                    if name not in TM.procs:
                        sock.sendall(b"ERR No such thread\n")
                        continue
                    subs = (
                        {(name, "stdout"), (name, "stderr")}
                        if spec == "all"
                        else {(name, spec)}
                    )
                    Broadcaster.add_listener(sock, subs)
                    sock.sendall(f"STREAM {name} {spec}\n".encode())
                    continue

                # exec -------------------------------------------------
                if cmd == "exec" and len(parts) == 2:
                    name = CS.name_from_token(parts[1])
                    cmline = CS.get(name)
                    if cmline is None:
                        sock.sendall(b"ERR No such command\n")
                        continue
                    try:
                        res = subprocess.run(
                            cmline,
                            shell=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            timeout=10,
                        )
                        output = res.stdout.rstrip()
                        if output:
                            sock.sendall((output + "\n").encode())
                        sock.sendall(f"EXIT {res.returncode}\n".encode())
                    except Exception as e:
                        sock.sendall(f"ERR exec failed: {e}\n".encode())
                    continue

                # reload ----------------------------------------------
                if cmd == "reload" and len(parts) == 1:
                    try:
                        reload_config()
                        sock.sendall(b"RELOADED\n")
                    except Exception as e:
                        sock.sendall(f"ERR reload failed: {e}\n".encode())
                    continue

                # restart ---------------------------------------------
                if cmd == "restart" and len(parts) == 2:
                    target = parts[1].lower()
                    if target == "all":
                        TM.restart_all()
                        sock.sendall(b"RESTART ALL\n")
                    else:
                        name = TM.name_from_token(target)
                        if name not in TM.procs:
                            sock.sendall(b"ERR No such thread\n")
                        else:
                            TM.restart_thread(name)
                            sock.sendall(f"RESTART {name}\n".encode())
                    continue

                # stop -------------------------------------------------
                if cmd == "stop" and len(parts) == 2:
                    target = parts[1].lower()
                    if target == "all":
                        TM.stop_all(manual=True)
                        sock.sendall(b"STOP ALL\n")
                    else:
                        name = TM.name_from_token(target)
                        if name not in TM.procs:
                            sock.sendall(b"ERR No such thread\n")
                        else:
                            TM.stop_thread(name)
                            sock.sendall(f"STOP {name}\n".encode())
                    continue

                # start -----------------------------------------------
                if cmd == "start" and len(parts) == 2:
                    target = parts[1].lower()
                    if target == "all":
                        TM.start_all()
                        sock.sendall(b"START ALL\n")
                    else:
                        name = TM.name_from_token(target)
                        if name not in TM.procs:
                            sock.sendall(b"ERR No such thread\n")
                        else:
                            TM.start_thread(name)
                            sock.sendall(f"START {name}\n".encode())
                    continue

                # unknown ---------------------------------------------
                sock.sendall(b"ERR Unknown command\n")
        finally:
            Broadcaster.remove_listener(sock)
            self.server.connections.discard(sock)
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            sock.close()


# ─────────────────────────── TCP Server class ─────────────────────────
class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.connections: Set[socket.socket] = set()


# ───────────────────────── debug ticker ───────────────────────────────
def _debug_loop():
    while not STOP_EVENT.is_set():
        sys.stderr.write(
            DEBUG_LINE_TEMPLATE.format(
                run=TM.running_count(),
                rst=TM.restarts_total(),
                tx=Broadcaster.sent_bytes(),
                drop=Broadcaster.dropped_bytes(),
            )
            + "\n"
        )
        sys.stderr.flush()
        time.sleep(1)


# ────────────────────────── reload helper ─────────────────────────────
def reload_config():
    """Stop running threads, reread config, rebuild TM/CS/settings."""
    global TM, CS, SETTINGS_LIST

    TM.shutdown()
    TM.stop_all()
    new_TM, new_CS, new_SETTINGS = init_from_config(CONFIG_PATH, DEBUG_ENABLED)
    TM, CS, SETTINGS_LIST = new_TM, new_CS, new_SETTINGS
    TM.start_all()


# ────────────────────────────────── main ──────────────────────────────
def main():
    global DEBUG_ENABLED, TM, CS, SETTINGS_LIST, CONFIG_PATH

    if os.geteuid() != 0:
        sys.exit("Must be run as root (sudo).")

    pa = argparse.ArgumentParser()
    pa.add_argument("--config", required=True, help="Path to profile")
    pa.add_argument("--debug", action="store_true")
    pa.add_argument("--port", type=int, default=9500)
    args = pa.parse_args()

    DEBUG_ENABLED = args.debug
    CONFIG_PATH = args.config

    TM, CS, SETTINGS_LIST = init_from_config(CONFIG_PATH, DEBUG_ENABLED, first_run=True)
    TM.start_all()

    if DEBUG_ENABLED:
        threading.Thread(target=_debug_loop, daemon=True).start()

    server = ThreadedTCPServer(("0.0.0.0", args.port), CommandHandler)

    def shutdown(sig, frm):
        dlog("signal received, shutting down …")
        if STOP_EVENT.is_set():
            return
        STOP_EVENT.set()
        for s in list(server.connections):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            s.close()
        threading.Thread(target=server.shutdown, daemon=True).start()
        TM.shutdown()
        TM.stop_all()
        dlog("bye!")
        time.sleep(0.5)
        os._exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        shutdown(signal.SIGINT, None)


if __name__ == "__main__":
    main()
