#!/usr/bin/env python3
"""A/B test of the camera strobe (flash): `#PP <product> I <0|1>` (vendor Ethernet command reference,
Product_Management_Commands: "Set strobe light enable flag"). RAM only. Ends with the strobe ON.
    python3 set_strobe.py            # test 1 -> 0 -> 1 and print the brightness of each
    python3 set_strobe.py 0|1        # just set it"""
import sys, time
import numpy as np, cv2
import set_exposure as se

def frames(s, n=3):
    time.sleep(1.5); out = []
    for _ in range(n):
        i, a = se.grab(s); out.append((i, a))
    return out[1:]  # the first can be an old one

def setstrobe(s, v):
    r = se.talk(s, b"#PP %d I %d" % (se.PRODUCT, v)).decode().replace("\r", "").strip().replace("\n", " ")
    acq = se.acquisition(s)
    open("exposure_changes.log", "a").write(f"{time.strftime('%F %T')} #PP {se.PRODUCT} I {v} -> {r} | acquisition: {acq}\n")
    print(f"#PP {se.PRODUCT} I {v} -> {r} | acquisition: {acq}")

if __name__ == "__main__":
    s = se.conn()
    if len(sys.argv) > 1: setstrobe(s, int(sys.argv[1])); sys.exit()
    print("start:", se.acquisition(s))
    for v in (1, 0, 1):
        setstrobe(s, v)
        fr = frames(s)
        for i, a in fr: print(f"   strobe {v}: image {i} min {a.min()} mean {a.mean():.1f} max {a.max()}")
        cv2.imwrite(f"strobe_{v}_{int(time.time())%1000}.png", fr[-1][1].reshape(160, 214))
