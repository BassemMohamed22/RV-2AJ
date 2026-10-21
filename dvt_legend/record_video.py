#!/usr/bin/env python3
"""Record a short video from the DVT camera's current live images. READ-ONLY: only the
documented `#IQ` (list saved images) and `#IS` (send image) terminal commands are used,
on port 3246 — nothing is written to the camera (no exposure/strobe/product changes).

The camera's own terminal only delivers ~1 frame every few seconds (ASCII pixel dump over
a slow link), so this is a time-lapse, not real-time video: N frames are pulled back to
back as fast as the camera will give them, then assembled into an .mp4 at a chosen
playback rate.

    python3 record_video.py                  # 12 frames, half-res (214x160), -> dvt_video.mp4
    python3 record_video.py --n 20 --full    # 20 frames, full-res (640x480, slower per frame)
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
    """One frame at (skip+1)x downsample, e.g. skip=2 -> 214x160, skip=0 -> 640x480."""
    ids = [int(x) for x in re.findall(rb"\$ (\d+)\s+\d+", talk(s, b"#IQ"))[1:]]
    if not ids:
        raise RuntimeError("camera has no saved images yet")
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
    frame = np.array(pix[: w * h], np.uint8).reshape(h, w)
    return new, frame, (w, h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=12, help="number of frames to capture")
    ap.add_argument("--full", action="store_true", help="640x480 instead of 214x160 (much slower per frame)")
    ap.add_argument("--fps", type=float, default=2.0, help="playback frames/sec in the output video")
    ap.add_argument("--out", default="dvt_video.mp4")
    ap.add_argument("--scale", type=int, default=3, help="upscale factor for visibility")
    args = ap.parse_args()

    skip = 0 if args.full else 2
    s = conn()
    frames = []
    t0 = time.time()
    for i in range(args.n):
        tA = time.time()
        img_id, frame, size = grab(s, skip)
        if frame is None:
            print(f"frame {i}: short/incomplete transfer, skipped")
            continue
        dt = time.time() - tA
        lo, hi = np.percentile(frame, [1, 99.5])
        stretched = np.clip((frame.astype(float) - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
        big = cv2.resize(stretched, (size[0] * args.scale, size[1] * args.scale),
                          interpolation=cv2.INTER_NEAREST if skip else cv2.INTER_LINEAR)
        big = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
        sat = 100.0 * float((frame >= 250).mean())
        label = f"image {img_id}  {time.strftime('%H:%M:%S')}  {dt:.1f}s/frame  sat {sat:.0f}%"
        cv2.putText(big, label, (8, big.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        frames.append(big)
        print(f"frame {i + 1}/{args.n}: id {img_id}, {dt:.1f}s, min {frame.min()} mean {frame.mean():.1f} "
              f"max {frame.max()} saturated {sat:.0f}%")
    s.close()
    if not frames:
        raise SystemExit("no frames captured")
    h, w = frames[0].shape[:2]
    vw = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
    for f in frames:
        vw.write(f)
    vw.release()
    print(f"\nwrote {args.out}: {len(frames)} frames at {args.fps} fps "
          f"({len(frames) / args.fps:.1f} s playback), captured over {time.time() - t0:.0f} s real time")


if __name__ == "__main__":
    main()
