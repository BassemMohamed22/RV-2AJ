#!/usr/bin/env python3
"""For N seconds, log each frame's brightest smoothed spot (x, y) and how much it stands out.
If the optics form an image, a phone torch moved across the front makes the spot move."""
import sys, time, numpy as np, cv2, set_exposure as se
s = se.conn(); t0 = time.time(); dur = float(sys.argv[1]) if len(sys.argv) > 1 else 120
while time.time() - t0 < dur:
    i, a = se.grab(s); a = a.reshape(160, 214).astype(float)
    sm = cv2.GaussianBlur(a, (0, 0), 6)
    y, x = np.unravel_index(np.argmax(sm), sm.shape)
    print(f"{time.strftime('%H:%M:%S')} image {i} mean {a.mean():6.1f} max {a.max():3.0f} "
          f"spot x={x:3d} y={y:3d} peak/mean {sm.max() / max(sm.mean(), 1):.2f}", flush=True)
