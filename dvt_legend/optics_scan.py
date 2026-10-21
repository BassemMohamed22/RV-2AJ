#!/usr/bin/env python3
"""Exposure/strobe scan: full-res frames, stats and a high-pass 'structure' measure.
Changes product 1 exposure/strobe (RAM only); restores exposure 3000 + strobe 1 at the end."""
import re, time, numpy as np, cv2
import set_exposure as se, set_strobe as ss

def full(s):
    time.sleep(2.0)
    ids = [int(x) for x in re.findall(rb"\$ (\d+)\s+\d+", se.talk(s, b"#IQ"))[1:]]
    for _ in range(2):  # skip one so the frame surely uses the new setting
        time.sleep(1.2); ids = [int(x) for x in re.findall(rb"\$ (\d+)\s+\d+", se.talk(s, b"#IQ"))[1:]]
    r = se.talk(s, b"#IS %d 0 0 639 479 0 1 1 0 0" % max(ids), wait=60)
    pix = [int(x) for l in r.replace(b"\r", b"\n").split(b"\n") if l.startswith(b"# 18") for x in l.split()[3:]]
    return np.array(pix[:320 * 240], np.uint8).reshape(240, 320).astype(float)

def setexp(s, us):
    print(f"  #PP 1 E {us} ->", se.talk(s, b"#PP 1 E %d" % us).decode().split()[1])

s = se.conn()
plan = [(1, 3000), (0, 3000), (0, 15000), (0, 60000), (0, 250000), (1, 300), (1, 800)]
for strobe, us in plan:
    ss.setstrobe(s, strobe); setexp(s, us)
    a = full(s)
    low = cv2.GaussianBlur(a, (0, 0), 15); hp = a - low
    sat = (a >= 250).mean() * 100
    print(f"strobe {strobe} exp {us:6d}: mean {a.mean():6.1f} min {a.min():3.0f} max {a.max():3.0f} sat {sat:4.1f}%  "
          f"structure(hp std) {hp.std():.2f}  low-freq range {low.min():.0f}..{low.max():.0f}")
    st = ((a - np.percentile(a, 1)) / (np.percentile(a, 99) - np.percentile(a, 1) + 1e-6) * 255).clip(0, 255).astype(np.uint8)
    cv2.imwrite(f"scan_s{strobe}_e{us}.png", st)
ss.setstrobe(s, 1); setexp(s, 3000)
print("restored: strobe 1, exposure 3000")
