#!/usr/bin/env python3
"""Talks to the PacketProbe firmware over its USB serial port.

  probe_log.py live -o run.csv [--seconds N]   stream CSV until Ctrl+C / N seconds
  probe_log.py arm                             deferred mode + reset: measure silently
  probe_log.py dump -o run.csv                 fetch the results kept since 'arm'
  probe_log.py cmd status                      any firmware command, prints the reply

A measurement without USB traffic: arm, run sender + receiver, dump.
Lines starting with '#' are status messages and go to stderr, CSV goes to the file.
No dependencies (no pyserial): termios, so Linux and macOS only.
"""

import argparse
import glob
import os
import select
import signal
import sys
import termios
import time
import tty

HEADER = "index,start_ns,end_ns,gpio_ns,frame_ns,latency_ns,edges"


class Probe:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)  # no echo, no line editing, no CR/LF translation
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.buf = b""

    def send(self, command):
        os.write(self.fd, (command + "\n").encode())

    def lines(self, timeout, total=None):
        """Yields complete lines until no data arrives for @timeout seconds,
        or @total seconds passed (live mode never goes quiet)."""
        deadline = time.monotonic() + total if total else None
        while deadline is None or time.monotonic() < deadline:
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                yield line.decode(errors="replace").rstrip("\r")
            ready, _, _ = select.select([self.fd], [], [], timeout)
            if not ready:
                return
            try:
                chunk = os.read(self.fd, 65536)
            except BlockingIOError:
                continue
            if not chunk:
                raise OSError("probe disconnected")
            self.buf += chunk


def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no probe found (/dev/ttyACM*, /dev/cu.usbmodem*), use --port")
    return ports[0]


def status(line):
    print(line, file=sys.stderr, flush=True)


def is_row(line):
    return line[:1].isdigit()


def cmd_live(probe, args):
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))

    probe.send("live")
    probe.send("reset")
    rows = 0
    started = False  # rows before '# reset' belong to an earlier run
    deadline = time.monotonic() + args.seconds if args.seconds else None
    with open(args.output, "w") as out:
        out.write(HEADER + "\n")
        while not stop and (deadline is None or time.monotonic() < deadline):
            for line in probe.lines(0.2):
                if line.startswith("# reset"):
                    started = True
                if line.startswith("#"):
                    status(line)
                elif started and is_row(line):
                    out.write(line + "\n")
                    rows += 1
                if stop or (deadline and time.monotonic() >= deadline):
                    break
    probe.send("status")
    for line in probe.lines(0.5, total=2):
        if line.startswith("# live"):
            status(line)
    status(f"# {rows} rows -> {args.output}")


def cmd_arm(probe, _args):
    probe.send("deferred")
    probe.send("reset")
    for line in probe.lines(0.5, total=3):
        if line.startswith("# mode") or line.startswith("# reset"):
            status(line)


def cmd_dump(probe, args):
    for _ in probe.lines(0.2, total=1):
        pass  # drop whatever was queued before
    probe.send("dump")
    rows = 0
    with open(args.output, "w") as out:
        out.write(HEADER + "\n")
        for line in probe.lines(5.0):
            if line.startswith("# end dump"):
                status(line)
                break
            if is_row(line):
                out.write(line + "\n")
                rows += 1
        else:
            sys.exit("dump: no '# end dump' from the probe (still in live mode?)")
    status(f"# {rows} rows -> {args.output}")


def cmd_cmd(probe, args):
    probe.send(" ".join(args.text))
    for line in probe.lines(args.wait, total=args.wait + 2):
        if not is_row(line) and line != HEADER:
            print(line)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial device (default: first /dev/ttyACM* or /dev/cu.usbmodem*)")
    sub = p.add_subparsers(dest="command", required=True)

    live = sub.add_parser("live", help="stream results to a CSV file")
    live.add_argument("-o", "--output", required=True)
    live.add_argument("--seconds", type=float, help="stop after this long (default: Ctrl+C / SIGTERM)")
    live.set_defaults(func=cmd_live)

    sub.add_parser("arm", help="deferred mode and reset").set_defaults(func=cmd_arm)

    dump = sub.add_parser("dump", help="fetch results kept in deferred mode")
    dump.add_argument("-o", "--output", required=True)
    dump.set_defaults(func=cmd_dump)

    cmd = sub.add_parser("cmd", help="send a firmware command, e.g. 'sim on', 'status', 'help'")
    cmd.add_argument("text", nargs="+")
    cmd.add_argument("--wait", type=float, default=0.5, help="seconds of silence that end the reply")
    cmd.set_defaults(func=cmd_cmd)

    args = p.parse_args()
    args.func(Probe(args.port or find_port()), args)


if __name__ == "__main__":
    main()
