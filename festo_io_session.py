#!/usr/bin/env python3
"""
One step of an interactive I/O test between the RV-2AJ (CR1-571) and the
Festo MPS 500 transfer line (Station 4, connector XPA4). Run once per step;
the person at the cell reports what they saw, which is logged with `note`.

Confirmed map (official communication plan, robot = "Robot with assembly"):
  OUT4 = ERR CODE #0     IN6 = Order Bit0
  OUT5 = ERR CODE #1     IN7 = Order Bit1
  OUT6 = Order active (busy)
  OUT7 = Station ready
  OUT0-OUT3 are not connected and are never touched by this tool.

Serial forms: EXECM_OUT(n)=0/1 (after CNTLON) and VALM_INB(0)/VALM_OUTB(0)
were accepted on this controller on 2026-09-25. Every write is still checked:
a Qe/Qx reply stops the step and is printed and logged exactly, and each
write is read back with VALM_OUTB.

Usage:
  festo_io_session.py inputs                   read IN0-IN7 and OUT0-OUT7 only
  festo_io_session.py assert 7=1 6=1 --hold 8  set the listed outputs, hold,
                                               re-read inputs, then clear OUT4-7
  festo_io_session.py note "what I saw"        log an observation

Every command, reply and observation is appended to festo_io_session.txt.
On exit, including Ctrl-C or an error, OUT4-OUT7 are all set to 0 and the
port is closed.
"""

import argparse
import datetime
import os
import sys
import time

import serial

PORT = "/dev/ttyUSB0"
TIMEOUT_S = 2.0
LINE_OUTPUTS = (4, 5, 6, 7)
OUT_NAMES = {4: "ERR CODE #0", 5: "ERR CODE #1", 6: "Order active (busy)", 7: "Station ready"}
IN_NAMES = {6: "Order Bit0", 7: "Order Bit1"}
ORDERS = {(0, 0): "no order", (0, 1): "Assemble", (1, 0): "Sorting out", (1, 1): "At bus: take part on pallet"}
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "festo_io_session.txt")


def log(text):
    line = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S.%f}"[:-3] + f"  {text}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


class Rejected(Exception):
    pass


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port=port, baudrate=9600, bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO, timeout=0.2)

    def send(self, command):
        self.ser.reset_input_buffer()
        self.ser.write(f"1;1;{command}\r".encode("ascii"))
        self.ser.flush()
        deadline = time.monotonic() + TIMEOUT_S
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self.ser.read(1)
            if chunk:
                buf += chunk
                if chunk == b"\r":
                    break
        reply = buf.decode("ascii", errors="replace").strip()
        log(f"TX 1;1;{command}   RX {reply!r}" if reply else f"TX 1;1;{command}   RX <no reply>")
        return reply

    def expect_ok(self, command):
        reply = self.send(command)
        if not reply.startswith("Qo"):
            raise Rejected(f"{command!r} -> {reply!r}" if reply else f"{command!r} -> no reply")
        return reply

    def read_byte(self, var):
        reply = self.expect_ok(f"VAL{var}(0)")
        return int(reply.rsplit("=", 1)[1])

    def close(self):
        self.ser.close()


def port_users(port):
    target = os.path.realpath(port)
    pids = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == os.getpid():
            continue
        try:
            if any(os.path.realpath(f"/proc/{pid}/fd/{fd}") == target for fd in os.listdir(f"/proc/{pid}/fd")):
                pids.append(int(pid))
        except OSError:
            pass
    return pids


def bits(byte):
    return [(byte >> n) & 1 for n in range(8)]


def describe_inputs(inb):
    b = bits(inb)
    order = ORDERS[(b[7], b[6])]
    return "IN0-7 " + " ".join(f"{n}:{b[n]}" for n in range(8)) + f"   order (IN7,IN6)=({b[7]},{b[6]}) -> {order}"


def describe_outputs(outb):
    b = bits(outb)
    return "OUT4-7 " + "  ".join(f"OUT{n} {OUT_NAMES[n]}={b[n]}" for n in LINE_OUTPUTS) + \
        f"   [OUT0-3 {b[0]}{b[1]}{b[2]}{b[3]}, untouched]"


def snapshot(link, label):
    inb = link.read_byte("M_INB")
    outb = link.read_byte("M_OUTB")
    log(f"{label}: {describe_inputs(inb)}")
    log(f"{label}: {describe_outputs(outb)}")
    return inb, outb


def cmd_inputs(link):
    snapshot(link, "READ")


def cmd_assert(link, pattern, hold):
    touched = True
    link.expect_ok("CNTLON")
    snapshot(link, "BEFORE")
    for n, v in pattern.items():
        link.expect_ok(f"EXECM_OUT({n})={v}")
    outb = link.read_byte("M_OUTB")
    wrong = [n for n, v in pattern.items() if (outb >> n) & 1 != v]
    if wrong:
        raise Rejected(f"read-back mismatch on OUT{wrong}: M_OUTB={outb:08b}")
    log(f"ASSERTED {pattern} -> {describe_outputs(outb)}; holding {hold:g} s")
    last = None
    end = time.monotonic() + hold
    while time.monotonic() < end:
        inb = link.read_byte("M_INB")
        if inb != last:
            log(f"DURING: {describe_inputs(inb)}" + ("" if last is None else "   <-- CHANGED"))
            last = inb
        time.sleep(0.5)
    return touched


def clear_line_outputs(link):
    ok = True
    try:
        link.expect_ok("CNTLON")
    except Exception as e:  # noqa: BLE001 - still try the clears
        log(f"!!! CNTLON before clear failed: {e}")
    for n in LINE_OUTPUTS:
        try:
            link.expect_ok(f"EXECM_OUT({n})=0")
        except Exception as e:  # noqa: BLE001 - keep clearing the others
            ok = False
            log(f"!!! OUT{n} MAY STILL BE SET: {e}")
    try:
        log(f"AFTER CLEAR: {describe_outputs(link.read_byte('M_OUTB'))}")
        log(f"AFTER CLEAR: {describe_inputs(link.read_byte('M_INB'))}")
    except Exception as e:  # noqa: BLE001
        ok = False
        log(f"!!! could not read back after clear: {e}")
    return ok


def parse_pattern(items):
    pattern = {}
    for item in items:
        try:
            n, v = item.split("=")
            n, v = int(n), int(v)
        except ValueError:
            sys.exit(f"bad pattern item {item!r}; use e.g. 7=1")
        if n not in LINE_OUTPUTS:
            sys.exit(f"OUT{n} is not a transfer-line output; only OUT4-OUT7 may be set")
        if v not in (0, 1):
            sys.exit(f"value for OUT{n} must be 0 or 1")
        pattern[n] = v
    if not pattern:
        sys.exit("nothing to assert")
    return pattern


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("inputs")
    a = sub.add_parser("assert")
    a.add_argument("pattern", nargs="+", help="e.g. 7=1 6=1 (outputs 4-7 only)")
    a.add_argument("--hold", type=float, default=8.0, help="seconds to hold (default 8)")
    a.add_argument("--label", default="", help="step name for the log")
    n = sub.add_parser("note")
    n.add_argument("text")
    args = ap.parse_args()

    if args.cmd == "note":
        log(f"OBSERVED: {args.text}")
        return
    pattern = parse_pattern(args.pattern) if args.cmd == "assert" else None
    users = port_users(PORT)
    if users:
        sys.exit(f"{PORT} is open by PID(s) {users} (GUI or ROS stack?). Close it first.")

    log("=" * 70)
    log(f"STEP {args.cmd}" + (f" {pattern} hold {args.hold:g}s" if pattern else "")
        + (f"  [{args.label}]" if getattr(args, "label", "") else ""))
    link = None
    touched = False
    try:
        link = Link(PORT)
        if args.cmd == "inputs":
            cmd_inputs(link)
        else:
            touched = True  # clear even if the first write fails part-way
            cmd_assert(link, pattern, args.hold)
    except serial.SerialException as e:
        log(f"SERIAL ERROR: {e}")
    except KeyboardInterrupt:
        log("INTERRUPTED (Ctrl-C)")
    except Rejected as e:
        log(f"STOPPED: controller rejected {e}")
    finally:
        if link is not None:
            if touched:
                clear_line_outputs(link)
                try:
                    link.send("CNTLOFF")
                except Exception:  # noqa: BLE001
                    pass
            link.close()
            log("port closed")


if __name__ == "__main__":
    main()
