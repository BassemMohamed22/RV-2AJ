#!/usr/bin/env python3
"""READ-ONLY Modbus TCP map of the DVT camera (192.168.10.242:502).
Uses only pymodbus read_* calls. No write path exists in this file."""
import argparse, json, sys
from pymodbus.client import ModbusTcpClient

IP, PORT = "192.168.10.242", 502
BLOCK = 125  # max registers per FC3/4 request


def read_bits(client, fn, start, end, unit):
    vals = {}
    a = start
    while a <= end:
        n = min(2000, end - a + 1)
        try:
            rr = fn(address=a, count=n, device_id=unit)
        except Exception as e:
            return vals, f"exception {type(e).__name__}: {e} at {a}"
        if rr.isError():
            return vals, f"{rr} (stopped at {a})"
        for i, b in enumerate(rr.bits[:n]):
            vals[a + i] = int(b)
        a += n
    return vals, "ok"


def read_regs(client, fn, start, end, unit):
    vals = {}
    a = start
    while a <= end:
        n = min(BLOCK, end - a + 1)
        try:
            rr = fn(address=a, count=n, device_id=unit)
        except Exception as e:
            return vals, f"exception {type(e).__name__}: {e} at {a}"
        if rr.isError():
            return vals, f"{rr} (stopped at {a})"
        for i, v in enumerate(rr.registers[:n]):
            vals[a + i] = v
        a += n
    return vals, "ok"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unit", type=int, default=1)
    ap.add_argument("--max", type=int, default=255)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    c = ModbusTcpClient(IP, port=PORT, timeout=3)
    if not c.connect():
        print("could not connect to", IP, PORT); sys.exit(1)
    result = {"unit": a.unit}
    jobs = [("coils", c.read_coils, read_bits),
            ("discrete_inputs", c.read_discrete_inputs, read_bits),
            ("holding_registers", c.read_holding_registers, read_regs),
            ("input_registers", c.read_input_registers, read_regs)]
    for name, fn, reader in jobs:
        vals, status = reader(c, fn, 0, a.max, a.unit)
        result[name] = {"status": status, "count": len(vals), "values": vals}
        nz = {k: v for k, v in vals.items() if v}
        print(f"{name:18} unit {a.unit}: {status} | {len(vals)} readable, {len(nz)} non-zero")
        if nz:
            print("   non-zero:", {k: v for k, v in list(nz.items())[:40]})
    c.close()
    if a.out:
        json.dump(result, open(a.out, "w"), indent=1)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
