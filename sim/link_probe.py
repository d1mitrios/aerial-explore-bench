#!/usr/bin/env python3
"""One-shot TCP probe of the PX4 -> Pegasus link (Pegasus listens on 0.0.0.0:4560 on
Windows, PX4 in WSL2 connects out to the host's vswitch address).

Listens on 0.0.0.0:<port> the way Pegasus does (pymavlink tcpin, backlog 1), but without
SO_REUSEADDR, so a stale listener on the port is reported instead of silently shared, and
waits for one connection. Run it under Isaac's own python (python.bat): the firewalls
(ESET Endpoint Security, Windows Defender Firewall) then judge the same program that serves
PX4 in a flight. Prints one status line per step, flushed:

  LISTENING 0.0.0.0:<port> pid=<n> exe=<python>
  ACCEPTED <peer ip>:<peer port>          (exit 0: the link is open)
  TIMEOUT no connection in <t> s          (exit 1)
  PORT_IN_USE port=<port> <error>         (exit 3: another process holds the port)

  <isaac>\\python.bat -u sim\\link_probe.py [--port 4560] [--timeout 60]

missions/batch_aerial.sh starts it through sim/link_probe.ps1 (the pre-flight, --link-check,
and after a run that got no PX4 heartbeat) and connects from WSL with bash's /dev/tcp.
Standard library only.
"""
import argparse
import os
import socket
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=4560)
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for the connection")
    ap.add_argument("--bind", default="0.0.0.0", help="address to listen on (Pegasus: 0.0.0.0)")
    a = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((a.bind, a.port))
        s.listen(1)
    except OSError as exc:
        print(f"PORT_IN_USE port={a.port} {exc}", flush=True)
        return 3
    print(f"LISTENING {a.bind}:{a.port} pid={os.getpid()} exe={sys.executable}", flush=True)
    s.settimeout(a.timeout)
    try:
        conn, peer = s.accept()
    except socket.timeout:
        print(f"TIMEOUT no connection in {a.timeout:g} s", flush=True)
        return 1
    print(f"ACCEPTED {peer[0]}:{peer[1]}", flush=True)
    conn.close()
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
