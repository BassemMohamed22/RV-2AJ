#!/usr/bin/env python3
"""
Interactive, manual test harness for verifying the CR1-571's relative
joint-move command syntax on real hardware.

This is NOT the GUI. It exists specifically because melfa_gui/serial_link.py's
send_step_move() is deliberately left unimplemented: the exact MELFA-BASIC IV
MOV / joint-variable syntax for a *relative* joint move has not been
confirmed against the manual. Use this script to test candidate command
strings by hand -- one axis, one small offset (1-2 degrees), one time,
watching the robot -- before writing that syntax into serial_link.py.

Safety:
  - Nothing moves until you explicitly send CNTLON then SRVON through this
    script's menu, each with an explicit confirmation.
  - You type the exact candidate command string yourself for every motion
    test via 'r' -- this script never guesses or hardcodes a MOV/motion
    command.
  - 's' sends STOP immediately from the main menu at any time.
  - Keep your hand near the controller's physical emergency stop while
    testing motion commands here.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "melfa_gui"))

from serial_link import (  # noqa: E402
    PORT_DEFAULT,
    SerialLink,
    SerialLinkError,
    compute_target_angle,
    is_ok_reply,
    parse_jposf,
)

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]

MENU = """
Commands:
  j  - query JPOSF (current joint angles)
  t  - compute a target angle for one joint (current + offset), for you
       to build a candidate MOV command string by hand
  c  - send CNTLON
  o  - send SRVON (requires typed confirmation)
  f  - send SRVOFF
  r  - send a raw command string you type (e.g. your candidate MOV) and
       show the raw reply
  s  - send STOP immediately
  m  - show this menu again
  q  - quit (sends SRVOFF first if SRVON was sent this session)
"""


def prompt(msg):
    return input(msg).strip()


def show_jposf(link):
    try:
        raw = link.query_jposf()
    except SerialLinkError as e:
        print(f"  ERROR: {e}")
        return None
    print(f"  raw: {raw!r}")
    if not is_ok_reply(raw):
        print("  (non-OK reply)")
        return None
    values = parse_jposf(raw)
    for j in JOINTS:
        v = values.get(j)
        print(f"    {j}: {v if v is not None else 'n/a'}")
    return values


def main():
    port = prompt(f"Port [{PORT_DEFAULT}]: ") or PORT_DEFAULT
    link = SerialLink()
    try:
        link.connect(port)
    except SerialLinkError as e:
        print(f"Could not open {port}: {e}")
        return
    print(f"Connected to {port} (9600 baud, even parity, 2 stop bits).")

    cntlon_done = False
    srvon_done = False

    print(MENU)

    while True:
        cmd = prompt("> ").lower()

        if cmd == "j":
            show_jposf(link)

        elif cmd == "t":
            joint = prompt(f"Joint ({'/'.join(JOINTS)}): ").upper()
            if joint not in JOINTS:
                print("  unknown joint")
                continue
            values = show_jposf(link)
            if not values or values.get(joint) is None:
                print("  could not read current angle for that joint")
                continue
            try:
                offset = float(prompt("Offset degrees (e.g. 1, -1, 5): "))
            except ValueError:
                print("  not a number")
                continue
            target = compute_target_angle(values[joint], offset)
            print(
                f"  current {joint} = {values[joint]:.2f}, offset = {offset:+.2f} "
                f"-> target = {target:.2f}"
            )
            print(
                "  Build your candidate MOV / joint-variable command using this "
                "target and send it with 'r'."
            )

        elif cmd == "c":
            try:
                raw = link.send_cntlon()
            except SerialLinkError as e:
                print(f"  ERROR: {e}")
                continue
            print(f"  reply: {raw!r}")
            cntlon_done = is_ok_reply(raw)

        elif cmd == "o":
            if not cntlon_done:
                print("  send CNTLON ('c') first.")
                continue
            confirm = prompt(
                "  Robot will be able to move once SRVON is sent. Clear the "
                "area and keep clear of the workspace. Type YES to continue: "
            )
            if confirm != "YES":
                print("  aborted.")
                continue
            try:
                raw = link.send_srvon()
            except SerialLinkError as e:
                print(f"  ERROR: {e}")
                continue
            print(f"  reply: {raw!r}")
            srvon_done = is_ok_reply(raw)

        elif cmd == "f":
            try:
                raw = link.send_srvoff()
            except SerialLinkError as e:
                print(f"  ERROR: {e}")
                continue
            print(f"  reply: {raw!r}")
            srvon_done = False

        elif cmd == "r":
            if not srvon_done:
                print(
                    "  SRVON has not been confirmed sent this session. "
                    "Sending a motion command now may do nothing, may "
                    "error, or may behave unexpectedly -- this script will "
                    "not stop you, but you should stop yourself unless "
                    "that's intended."
                )
            raw_cmd = prompt(
                "  Exact command to send (without the '1;1;' prefix or "
                "trailing \\r): "
            )
            if not raw_cmd:
                print("  empty, skipped.")
                continue
            confirm = prompt(f"  About to send: {raw_cmd!r}. Type YES to send: ")
            if confirm != "YES":
                print("  aborted.")
                continue
            try:
                raw = link.send(raw_cmd)
            except SerialLinkError as e:
                print(f"  ERROR: {e}")
                continue
            print(f"  reply: {raw!r} -> {'OK' if is_ok_reply(raw) else 'ERROR/UNEXPECTED'}")

        elif cmd == "s":
            try:
                raw = link.send_stop()
                print(f"  STOP reply: {raw!r}")
            except SerialLinkError as e:
                print(f"  ERROR sending STOP: {e}")

        elif cmd == "m":
            print(MENU)

        elif cmd == "q":
            if srvon_done:
                try:
                    link.send_srvoff()
                except SerialLinkError:
                    pass
            link.disconnect()
            print("Disconnected. Bye.")
            return

        else:
            print("  unknown command.")
            print(MENU)


if __name__ == "__main__":
    main()
