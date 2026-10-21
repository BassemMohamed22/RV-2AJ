#!/usr/bin/env python3
"""Live DVT camera view in a plain OpenCV window (cv2.imshow), updated as fast as the
camera's terminal will give frames. READ-ONLY: only `#IQ` (list saved images) and `#IS`
(send image) on port 3246 — nothing is written to the camera.

The camera's terminal is slow (an ASCII pixel dump over the link), so this refreshes
every ~3-20s depending on resolution, not real video frame rates. That is the camera's
limit, not this script's.

    python3 live_opencv.py              # half-res (214x160), ~3s/refresh
    python3 live_opencv.py --full       # full-res (640x480), ~20s/refresh

Window keys: S = save current frame as PNG, Q / Esc = quit.
"""
import argparse
import re
import socket
import time

import cv2
import numpy as np

IP = "192.168.10.242"


def conn():
    s = socket.create_connection((IP, 3246), timeout=8)
    s.settimeout(4)
    try:
        s.recv(100)  # banner
    except socket.timeout:
        pass
    return s


def talk(s, cmd, wait=60):
    s.sendall(cmd + b"\r")
    out = b""
    s.settimeout(wait)
    while True:
        d = s.recv(262144)
        if not d:
            raise ConnectionError("camera closed the connection")
        out += d
        if re.search(rb"%\s*\d+\s*\r\n[?!]\s*$", out):
            return out


def grab(s, skip):
    ids = [int(x) for x in re.findall(rb"\$ (\d+)\s+\d+", talk(s, b"#IQ"))[1:]]
    if not ids:
        return None, None, None
    new = max(ids)
    w = -(-640 // (skip + 1))
    h = -(-480 // (skip + 1))
    r = talk(s, b"#IS %d 0 0 639 479 0 %d %d 0 0" % (new, skip, skip))
    pix = []
    for line in r.replace(b"\r", b"\n").split(b"\n"):
        if line.startswith(b"# 18"):
            pix += [int(x) for x in line.split()[3:]]
    if len(pix) < w * h:
        return None, None, None
    return new, np.array(pix[: w * h], np.uint8).reshape(h, w), (w, h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", action="store_true", help="640x480 instead of 214x160")
    ap.add_argument("--scale", type=int, default=3, help="upscale factor for visibility")
    ap.add_argument("--reconnect-after", type=int, default=5, help="reconnect the socket every N frames")
    args = ap.parse_args()
    skip = 0 if args.full else 2

    win = "DVT 510MR (read-only)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    s = conn()
    n = 0
    print("Q or Esc to quit, S to save the current frame. Ctrl+C in the terminal also works.")
    try:
        while True:
            t0 = time.time()
            try:
                img_id, frame, size = grab(s, skip)
            except (ConnectionError, OSError, socket.timeout) as e:
                print(f"connection problem ({e}); reconnecting...")
                try:
                    s.close()
                except Exception:
                    pass
                time.sleep(1)
                s = conn()
                continue
            n += 1
            if args.reconnect_after and n % args.reconnect_after == 0:
                s.close()
                s = conn()
            if frame is None:
                print("short/incomplete frame, retrying")
                continue
            dt = time.time() - t0
            lo, hi = np.percentile(frame, [1, 99.5])
            stretched = np.clip((frame.astype(float) - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
            big = cv2.resize(stretched, (size[0] * args.scale, size[1] * args.scale),
                              interpolation=cv2.INTER_NEAREST if skip else cv2.INTER_LINEAR)
            big = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
            sat = 100.0 * float((frame >= 250).mean())
            label = (f"image {img_id}  {time.strftime('%H:%M:%S')}  {dt:.1f}s/frame  "
                     f"min {frame.min()} mean {frame.mean():.0f} max {frame.max()}  sat {sat:.0f}%")
            cv2.putText(big, label, (8, big.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(big, "Q/Esc quit  S save  (read-only)", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow(win, big)
            print(label)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                path = time.strftime("dvt_%Y%m%d_%H%M%S.png")
                cv2.imwrite(path, frame)
                print("saved", path)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
