#!/usr/bin/env python3
"""Log the raw mean brightness of each new camera frame for N seconds (no contrast stretch)."""
import sys, time, set_exposure as se
s = se.conn(); t0 = time.time(); dur = float(sys.argv[1]) if len(sys.argv) > 1 else 120
while time.time() - t0 < dur:
    i, a = se.grab(s)
    bar = "#" * int(a.mean() / 4)
    print(f"{time.strftime('%H:%M:%S')}  image {i}  mean {a.mean():6.1f}  min {a.min():3d} max {a.max():3d}  {bar}", flush=True)
