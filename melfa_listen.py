"""
Passive listener for the MELFA CR1 serial port.

Sends nothing. Just opens the port and prints whatever bytes arrive,
so you can verify wiring, baud rate, parity and stop bits without any
risk of commanding the robot.

Usage:
    pip install pyserial
    python3 melfa_listen.py --port /dev/ttyUSB0
    python3 melfa_listen.py --port /dev/ttyUSB0 --scan     # try common settings

Note: the CR1 controller does not talk on its own; it only replies to
commands. So with a PC alone you will normally see nothing. To get
traffic to listen to, either:
  - have COSIROP (or another PC) talk to the controller while you tap the
    line, or
  - run a robot program containing PRINT statements to the open COM
    channel (requires CPRCE11 = 2 data link mode), or
  - use the companion script that actually sends a command.

Stop with Ctrl-C.
"""

import argparse
import sys
import time

import serial

COMMON_SETTINGS = [
    # (parity, stopbits, description)
    (serial.PARITY_EVEN, serial.STOPBITS_TWO, "manual default: even parity, 2 stop bits"),
    (serial.PARITY_NONE, serial.STOPBITS_ONE, "reported working: no parity, 1 stop bit"),
    (serial.PARITY_ODD, serial.STOPBITS_ONE, "odd parity, 1 stop bit"),
    (serial.PARITY_NONE, serial.STOPBITS_TWO, "no parity, 2 stop bits"),
]

COMMON_BAUDS = [9600, 19200, 38400, 4800]


def listen(port: str, baud: int, parity, stopbits, seconds: float | None) -> bytes:
    """Open the port read-only and print anything that arrives."""
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        parity=parity,
        stopbits=stopbits,
        bytesize=serial.EIGHTBITS,
        timeout=0.5,
    )
    collected = bytearray()
    start = time.time()
    try:
        while seconds is None or (time.time() - start) < seconds:
            chunk = ser.read(256)
            if chunk:
                collected.extend(chunk)
                text = chunk.decode("ascii", errors="replace")
                print(f"[{len(chunk):>3} bytes] {text!r}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
    return bytes(collected)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--scan", action="store_true",
                    help="cycle through common baud/parity combinations, 5 s each")
    args = ap.parse_args()

    if not args.scan:
        print(f"Listening on {args.port} at {args.baud} 8-E-2. Ctrl-C to stop.\n")
        listen(args.port, args.baud, serial.PARITY_EVEN, serial.STOPBITS_TWO, None)
        return 0

    print("Scanning common settings, 5 seconds each. Make the controller talk "
          "during this (e.g. run COSIROP or a PRINT program).\n")
    for baud in COMMON_BAUDS:
        for parity, stopbits, desc in COMMON_SETTINGS:
            print(f"--- {baud} baud, {desc} ---")
            data = listen(args.port, baud, parity, stopbits, 5.0)
            if data:
                printable = sum(32 <= b < 127 or b in (10, 13) for b in data)
                ratio = printable / len(data)
                verdict = "looks like clean text" if ratio > 0.9 else "garbled, wrong settings"
                print(f"    -> {len(data)} bytes, {verdict}\n")
            else:
                print("    -> nothing received\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())