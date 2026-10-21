#!/usr/bin/env python3
"""
Read-only serial link verification for a Mitsubishi RV-2AJ / CR1-571 controller.

Sends ONLY non-procedural query commands (STATE, PPOSF, JPOSF). Never sends
CNTLON, SRVON, or any motion/servo command, so the robot cannot move as a
result of running this script.

Protocol: ASCII commands "1;1;<COMMAND>\r". A reply beginning with "Qo"
means OK; "Qe" or "Qx" mean the controller rejected the command.
"""

import sys
import time

try:
    import serial
    from serial import Serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

PORT = "/dev/ttyUSB0"
READ_TIMEOUT = 2.0  # seconds

# Only read-only, non-motion queries. Do NOT add CNTLON/SRVON/motion commands.
QUERY_COMMANDS = ["STATE", "PPOSF", "JPOSF"]

BAUD_RATES = [9600, 19200, 4800]

SERIAL_CONFIGS = [
    {"name": "even parity, 2 stop bits", "parity": serial.PARITY_EVEN, "stopbits": serial.STOPBITS_TWO},
    {"name": "no parity, 1 stop bit", "parity": serial.PARITY_NONE, "stopbits": serial.STOPBITS_ONE},
    {"name": "odd parity, 1 stop bit", "parity": serial.PARITY_ODD, "stopbits": serial.STOPBITS_ONE},
]


def send_command(ser, command):
    """Send one command and read a reply up to '\\r'. Returns the raw bytes read."""
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    msg = f"1;1;{command}\r".encode("ascii")
    ser.write(msg)
    ser.flush()

    deadline = time.time() + READ_TIMEOUT
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(1)
        if not chunk:
            break  # ser.timeout elapsed with nothing available
        buf += chunk
        if buf.endswith(b"\r"):
            break
    return bytes(buf)


def is_clean_ok_reply(raw):
    if not raw:
        return False
    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError:
        return False
    return text.startswith("Qo")


def describe_reply(raw):
    if not raw:
        return "<no reply>"
    try:
        text = raw.decode("ascii", errors="replace")
    except Exception:
        text = repr(raw)
    return repr(text)


def try_combo(baud, parity_cfg):
    try:
        ser = Serial(
            port=PORT,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=parity_cfg["parity"],
            stopbits=parity_cfg["stopbits"],
            timeout=READ_TIMEOUT,
        )
    except serial.SerialException as e:
        return None, str(e)

    try:
        raw = send_command(ser, "STATE")
    finally:
        ser.close()

    return raw, None


def main():
    print(f"Mitsubishi RV-2AJ / CR1-571 read-only serial link verification")
    print(f"Port: {PORT}")
    print(f"Queries used (all read-only): {', '.join(QUERY_COMMANDS)}")
    print("=" * 70)

    working_combo = None
    attempts = []

    print("\nScanning baud rate / parity / stop-bit combinations with STATE...")
    for baud in BAUD_RATES:
        for cfg in SERIAL_CONFIGS:
            label = f"{baud} baud, {cfg['name']}"
            raw, err = try_combo(baud, cfg)
            if err is not None:
                print(f"  [{label}] ERROR opening port: {err}")
                attempts.append((label, "open_error", err))
                # If we can't even open the port, further attempts will fail the
                # same way (port busy/missing/permissions) - report and stop scanning.
                print("\nCould not open the serial port at all. Skipping remaining combinations.")
                print_summary(None, attempts)
                return
            ok = is_clean_ok_reply(raw)
            status = "OK (Qo...)" if ok else "no/garbled reply"
            print(f"  [{label}] reply={describe_reply(raw)} -> {status}")
            attempts.append((label, "ok" if ok else "no_reply", raw))
            if ok and working_combo is None:
                working_combo = (baud, cfg)

    if working_combo is None:
        print_summary(None, attempts)
        return

    baud, cfg = working_combo
    print("\n" + "=" * 70)
    print(f"Using working combination: {baud} baud, {cfg['name']}")
    print("Querying STATE, PPOSF, JPOSF...\n")

    try:
        ser = Serial(
            port=PORT,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=cfg["parity"],
            stopbits=cfg["stopbits"],
            timeout=READ_TIMEOUT,
        )
    except serial.SerialException as e:
        print(f"ERROR: could not reopen port with working settings: {e}")
        print_summary(working_combo, attempts)
        return

    results = {}
    try:
        for cmd in QUERY_COMMANDS:
            raw = send_command(ser, cmd)
            results[cmd] = raw
            print(f"{cmd}: {describe_reply(raw)}")
    finally:
        ser.close()

    print_summary(working_combo, attempts, results)


def print_summary(working_combo, attempts, results=None):
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    if working_combo is None:
        any_open_error = any(a[1] == "open_error" for a in attempts)
        print("No working serial settings found. Nothing replied with a clean 'Qo' response.")
        print("\nThings to check:")
        if any_open_error:
            print(f"  - Could not open {PORT} at all:")
            print("      - Confirm the adapter is plugged in: `ls -l /dev/ttyUSB*`")
            print("      - Check permissions: your user may need to be in the 'dialout' group")
            print("        (`sudo usermod -aG dialout $USER`, then log out/in), or run with sudo")
            print("        temporarily to test.")
            print("      - Make sure no other process (e.g. another script, minicom) has the port open.")
        else:
            print("  - Cable / wiring:")
            print("      - RV-2AJ/CR1 uses a null-modem (crossed TX/RX) RS232 cable; a straight-through")
            print("        cable will not work. Verify TX/RX are crossed and GND is connected.")
            print("      - Check the DB9/DB25 pinout matches the controller's COM1 connector.")
            print("  - Adapter:")
            print("      - Confirm the USB-RS232 adapter enumerates and works with another RS232 device")
            print("        or a loopback test (short TX-RX and confirm bytes echo).")
            print("  - Controller configuration:")
            print("      - Re-check COMDEV is set to RS232C on COM1, and CPRCE11 = 0.")
            print("      - Confirm controller MODE switch is AUTO (Ext.) and teaching pendant is DISABLE,")
            print("        as required for host commands.")
            print("      - Confirm controller default baud/parity/stop-bit settings for COM1 (often")
            print("        parameter CBAUD/CPRCL or similar) match one of the combinations tried.")
            print("  - Try other combinations manually if yours uses non-standard settings.")
    else:
        baud, cfg = working_combo
        print(f"Working settings: {baud} baud, {cfg['name']}, 8 data bits, port {PORT}")
        if results:
            print("\nQuery results:")
            for cmd, raw in results.items():
                ok = is_clean_ok_reply(raw)
                print(f"  {cmd}: {'OK' if ok else 'UNEXPECTED'} -> {describe_reply(raw)}")


if __name__ == "__main__":
    main()
