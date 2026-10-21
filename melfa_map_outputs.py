#!/usr/bin/env python3
"""
Map the RV-2AJ / CR1-571 general-purpose outputs 0-7, one at a time, with a
person watching the cell and reporting what each output does.

Serial form tried: EXECM_OUT(n)=1 / EXECM_OUT(n)=0 (MELFA-BASIC IV M_OUT with
the EXEC prefix). Reading with VALM_OUT(n) is confirmed (2026-09-25); the EXEC
write form is NOT confirmed. If the controller rejects it
(Qe/Qx), the exact reply is printed and the script stops; no other form is
tried.

Sequence for each output: read its current value (VALM_OUT), announce the
command, wait for Enter, switch it to the opposite value (normally 0 -> 1),
record what you saw, restore the value it had, wait for you to confirm it
returned, then Enter to move on. Only one output is ever changed at a time. On
exit, including Ctrl-C, every output this run changed is restored to the value
it had before and the port is closed.

Usage:  python3 melfa_map_outputs.py [--port /dev/ttyUSB0] [--first 0 --last 7]
Stop the ROS stack first; the script refuses to run if another program has
the port open.
"""

import argparse
import datetime
import os
import sys
import time

import serial

TIMEOUT_S = 2.0
OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output_map.txt")

SAFETY_TEXT = """
==========================================================================
 SAFETY - read before continuing
==========================================================================
 The outputs being tested may be wired to pneumatic actuators, including
 the Festo HGP-16 parallel gripper, which closes with up to 160 N, and to
 stoppers on the transfer line. Setting an output can make them move
 without further warning.

 * Keep hands clear of the gripper and the transfer line before any output
   is set, and until it has been set back to 0.
 * Only one output is changed at a time; each is restored before the next.
 * Ctrl-C at any prompt restores every output changed in this run.
 * The robot arm itself is not moved: no servo-on, no motion commands.
==========================================================================
"""


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
        return buf.decode("ascii", errors="replace").strip()

    def close(self):
        self.ser.close()


def is_ok(reply):
    return reply.startswith("Qo")


def other_users(port):
    """PIDs of other processes that have the serial port open."""
    target = os.path.realpath(port)
    pids = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == os.getpid():
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                if os.path.realpath(f"/proc/{pid}/fd/{fd}") == target:
                    pids.append(int(pid))
                    break
        except OSError:
            continue
    return pids


def out_cmd(n, value):
    return f"EXECM_OUT({n})={value}"


def tx(link, command):
    print(f"    -> 1;1;{command}")
    reply = link.send(command)
    print(f"    <- {reply!r}" if reply else "    <- <no reply>")
    return reply


class CommandRejected(Exception):
    pass


def set_output(link, n, value):
    reply = tx(link, out_cmd(n, value))
    if not is_ok(reply):
        raise CommandRejected(
            f"controller rejected '{out_cmd(n, value)}' with reply {reply!r}" if reply
            else f"no reply to '{out_cmd(n, value)}'"
        )


def read_output(link, n):
    """Current value of output n (VALM_OUT(n) -> 'QoKM_OUT(n)=+0'), or None."""
    reply = tx(link, f"VALM_OUT({n})")
    if not is_ok(reply) or "=" not in reply:
        return None
    try:
        return int(reply.rsplit("=", 1)[1])
    except ValueError:
        return None


def write_record(results, form_note):
    lines = [
        f"RV-2AJ / CR1-571 general-purpose output map ({datetime.datetime.now():%Y-%m-%d %H:%M})",
        f"Command form: {form_note}",
        "",
        f"{'Output':<8}{'Test':<8}Observed",
        f"{'-' * 8}{'-' * 60}",
    ]
    lines += [f"{n:<8}{f'{a}->{b}':<8}{text}" for n, a, b, text in results]
    text = "\n".join(lines) + "\n"
    with open(OUTPUT_FILE, "w") as f:
        f.write(text)
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--first", type=int, default=0)
    ap.add_argument("--last", type=int, default=7)
    args = ap.parse_args()

    users = other_users(args.port)
    if users:
        sys.exit(f"{args.port} is already open by PID(s) {users} (the ROS stack?). Stop it first.")

    print(SAFETY_TEXT)
    input("Press Enter once hands are clear and you are ready (Ctrl-C to abort)... ")

    link = None
    changed = {}           # output -> value before this run changed it (restored on exit)
    results = []           # (output, before, test value, observation)
    form_note = "EXECM_OUT(n)=1 / =0 (unverified before this run)"
    try:
        link = Link(args.port)
        print("\nConfirming the link:")
        state = tx(link, "STATE")
        if not is_ok(state):
            sys.exit(f"STATE did not return Qo ({state!r}); check the connection.")
        reply = tx(link, "CNTLON")   # operating rights for EXEC commands; no motion
        if not is_ok(reply):
            sys.exit(f"CNTLON rejected ({reply!r}); cannot send EXEC commands.")

        for n in range(args.first, args.last + 1):
            print("\n" + "-" * 74)
            before = read_output(link, n)
            if before not in (0, 1):
                print(f" STOPPED: could not read the current value of output {n}.")
                break
            test = 1 - before
            print(f" Output {n} is currently {before}. Will send  1;1;{out_cmd(n, test)}")
            if before == 1:
                print(" (It is already ON, so this test switches it OFF and then back ON.)")
            input(f" Press Enter to SET it to {test} (hands clear)... ")
            try:
                set_output(link, n, test)
            except CommandRejected as e:
                print(f"\n STOPPED: {e}.")
                print(" The EXECM_OUT form is not accepted as sent; no other form was tried.")
                form_note = f"REJECTED on output {n}: {e}"
                break
            changed[n] = before
            form_note = "EXECM_OUT(n)=1 / =0 (accepted by this controller)"
            seen = input(f" Output {n} is now {test}. What did you observe? ").strip() or "(nothing observed)"
            results.append((n, before, test, seen))

            print(f" Restoring output {n} to {before}:")
            set_output(link, n, before)
            input(f" Output {n} is back to {before}. Press Enter once it has returned to its prior state... ")
            input(" Press Enter to continue to the next output... ")

    except KeyboardInterrupt:
        print("\n Interrupted.")
    except CommandRejected as e:
        print(f"\n STOPPED: {e}. Setting outputs back to 0.")
    finally:
        if link is not None:
            if changed:
                print("\nRestoring every output changed in this run:")
            for n, before in changed.items():
                try:
                    if not is_ok(tx(link, out_cmd(n, before))):
                        print(f" !!! OUTPUT {n} MAY NOT BE RESTORED to {before}: not acknowledged. Check the cell.")
                except Exception as e:  # noqa: BLE001 - keep resetting the others
                    print(f" !!! OUTPUT {n} MAY NOT BE RESTORED to {before}: {e}")
            try:
                tx(link, "CNTLOFF")
            except Exception:  # noqa: BLE001
                pass
            link.close()
            print("Port closed.")

    if results or "REJECTED" in form_note:
        print("\n" + write_record(results, form_note))
        print(f"Saved to {os.path.abspath(OUTPUT_FILE)}")


if __name__ == "__main__":
    main()
