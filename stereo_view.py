#!/usr/bin/env python3
"""
"Stereo Vision 2" USB stereo camera (two UVC cameras, 0ac8:8801 + 0ac8:8802,
1280x720 MJPG 30 fps each): live view, depth heatmap, object detection with
real distances.

Two modes, chosen automatically:
  - METRIC (stereo_calib.yaml exists): rectified with the calibration, depth in
    mm, YOLOv8 objects with their X/Y/Z in mm (left-camera frame: X right,
    Y down, Z forward), mouse-over gives the distance of any pixel.
  - RELATIVE (no calibration): rectified from feature matches, the heatmap
    shows near/far only.

    python3 stereo_view.py --board             # write the checkerboard to print (stereo_board.png)
    python3 stereo_view.py --calibrate --square 25.0
                                               # calibration window; --square = MEASURED square size in mm
    python3 stereo_view.py                     # live viewer
    python3 stereo_view.py --snapshot          # one frame -> stereo_snapshot.jpg + objects printed, no window

Calibration window: hold the printed board (flat, on a board/clipboard) so
BOTH views see all of it; pairs are captured automatically when the board is
still and in a new place. Vary distance (30-150 cm), tilt it, cover the corners
of the image. After 20+ pairs press C. Keys: SPACE capture now, C calibrate, Q quit.
Viewer keys: R re-rectify (relative mode), S swap left/right, D detection on/off, Q quit.

Run from a normal terminal (VS Code's snap terminal breaks PyQt).
"""

import argparse
import glob
import os
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB = os.path.join(HERE, "stereo_calib.yaml")
YOLO_WEIGHTS = os.path.expanduser("~/ros2_ws/yolov8n.pt")
HAND_MODEL = os.path.expanduser("~/ros2_ws/src/hand_gesture_drone_control/models/hand_landmarker.task")
SCALE_FILE = os.path.join(HERE, "stereo_scale.json")
# Without a calibration, distance = FB / disparity (full-res px). FB = focal(px) x baseline(mm).
# Guess until measured: ~90 deg wide lens at 1280 px (focal ~640 px) x ~60 mm lens spacing.
FB_GUESS = 640.0 * 60.0
PALM = [0, 5, 9, 13, 17]   # wrist + finger bases: the palm centre is their mean
W, H = 1280, 720      # capture size of each camera
PW, PH = 640, 360     # processing size for the disparity
BOARD = (9, 6)        # inner corners of the checkerboard (10 x 7 squares)
NEAR_MM, FAR_MM = 250.0, 2500.0   # heatmap colour range in metric mode


def find_cameras():
    """The two Stereo Vision capture nodes (V4L2 index 0), ordered by USB port."""
    cams = []
    for d in glob.glob("/sys/class/video4linux/video*"):
        try:
            name = open(os.path.join(d, "name")).read()
            index = open(os.path.join(d, "index")).read().strip()
        except OSError:
            continue
        if "Stereo Vision" in name and index == "0":
            port = os.path.realpath(os.path.join(d, "device")).split("/")[-2]
            cams.append((port, "/dev/" + os.path.basename(d)))
    cams.sort()
    return [dev for _, dev in cams]


class Grabber(threading.Thread):
    """Keeps the newest frame of one camera."""

    def __init__(self, dev):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {dev}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.dev = dev
        self.frame = None
        self.stamp = 0.0
        self.count = 0
        self.running = True
        self.lock = threading.Lock()

    def run(self):
        while self.running:
            ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.frame, self.stamp = f, time.time()
                    self.count += 1
            else:
                time.sleep(0.01)
        self.cap.release()

    def get(self):
        with self.lock:
            return self.frame, self.stamp


def open_pair(swap):
    devs = find_cameras()
    if len(devs) < 2:
        sys.exit(f"need two Stereo Vision cameras, found {devs}")
    if swap:
        devs = devs[::-1]
    grabbers = [Grabber(d) for d in devs]
    for g in grabbers:
        g.start()
    return devs, grabbers


def get_pair(grabbers):
    (fa, ta), (fb, tb) = grabbers[0].get(), grabbers[1].get()
    return fa, fb, abs(ta - tb)


# -- depth -------------------------------------------------------------------------------------

class Stereo:
    def __init__(self, max_disp=128):
        self.max_disp = max_disp
        self.H1 = self.H2 = None
        self.maps = None
        self.Q = None
        self.info = "no rectification yet"
        block = 5
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=max_disp, blockSize=block,
            P1=8 * 3 * block * block, P2=32 * 3 * block * block,
            disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        # Edge-preserving smoothing (WLS) fills the holes; needs opencv-contrib.
        try:
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.sgbm)
            self.wls = cv2.ximgproc.createDisparityWLSFilter(self.sgbm)
            self.wls.setLambda(8000.0)
            self.wls.setSigmaColor(1.5)
        except AttributeError:
            self.wls = None

    @property
    def metric(self):
        return self.maps is not None

    def load_calibration(self, path):
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        K1, D1, K2, D2 = (fs.getNode(n).mat() for n in ("K1", "D1", "K2", "D2"))
        R, T = fs.getNode("R").mat(), fs.getNode("T").mat()
        size = tuple(int(v) for v in fs.getNode("image_size").mat().ravel())
        rms = fs.getNode("rms").real()
        fs.release()
        # Rectify straight to the processing size.
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, size, R, T, alpha=0, newImageSize=(PW, PH))
        self.cam = (K1, D1, R1, P1, K2, D2, R2, P2)
        self.maps = (cv2.initUndistortRectifyMap(K1, D1, R1, P1, (PW, PH), cv2.CV_16SC2),
                     cv2.initUndistortRectifyMap(K2, D2, R2, P2, (PW, PH), cv2.CV_16SC2))
        self.Q = Q
        self.focal = P1[0, 0]
        self.baseline = float(np.linalg.norm(T))
        self.info = (f"METRIC: baseline {self.baseline:.1f} mm, focal {self.focal:.0f} px @ {PW} wide, "
                     f"calibration RMS {rms:.2f} px, nearest measurable {self.focal * self.baseline / self.max_disp:.0f} mm")

    def rectify_from(self, left, right):
        """Uncalibrated rectification from SIFT matches between the two views."""
        gl = cv2.cvtColor(cv2.resize(left, (PW, PH)), cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(cv2.resize(right, (PW, PH)), cv2.COLOR_BGR2GRAY)
        sift = cv2.SIFT_create(4000)
        kl, dl = sift.detectAndCompute(gl, None)
        kr, dr = sift.detectAndCompute(gr, None)
        if dl is None or dr is None or len(kl) < 30 or len(kr) < 30:
            self.info = "too little texture to rectify"
            return False
        matches = cv2.BFMatcher().knnMatch(dl, dr, k=2)
        good = [m for m, n in (p for p in matches if len(p) == 2) if m.distance < 0.75 * n.distance]
        if len(good) < 30:
            self.info = f"only {len(good)} matches"
            return False
        pl = np.float32([kl[m.queryIdx].pt for m in good])
        pr = np.float32([kr[m.trainIdx].pt for m in good])
        F, mask = cv2.findFundamentalMat(pl, pr, cv2.FM_RANSAC, 1.0, 0.99)
        if F is None or F.shape != (3, 3):
            self.info = "fundamental matrix failed"
            return False
        inl = mask.ravel().astype(bool)
        ok, H1, H2 = cv2.stereoRectifyUncalibrated(pl[inl], pr[inl], F, (PW, PH))
        if not ok:
            self.info = "rectification failed"
            return False
        self.H1, self.H2 = H1, H2
        self.info = f"RELATIVE (not calibrated): {inl.sum()} inlier matches of {len(good)}"
        return True

    def rectify(self, left, right):
        if self.metric:
            (m1a, m1b), (m2a, m2b) = self.maps
            return (cv2.remap(left, m1a, m1b, cv2.INTER_LINEAR), cv2.remap(right, m2a, m2b, cv2.INTER_LINEAR))
        l, r = cv2.resize(left, (PW, PH)), cv2.resize(right, (PW, PH))
        if self.H1 is not None:
            l = cv2.warpPerspective(l, self.H1, (PW, PH))
            r = cv2.warpPerspective(r, self.H2, (PW, PH))
        return l, r

    def compute(self, left, right):
        """-> rectified left, rectified right, disparity (px, 0 = none), xyz (mm, metric only)."""
        l, r = self.rectify(left, right)
        gl, gr = cv2.cvtColor(l, cv2.COLOR_BGR2GRAY), cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
        d = self.sgbm.compute(gl, gr)
        if self.wls is not None:
            d = self.wls.filter(d, gl, disparity_map_right=self.right_matcher.compute(gr, gl))
        d = d.astype(np.float32) / 16.0
        d[:, :self.max_disp] = 0      # no right-image partner at the left edge
        if not self.metric and self.H1 is not None:
            d[cv2.warpPerspective(np.full((PH, PW), 255, np.uint8), self.H1, (PW, PH)) < 255] = 0
        d[d < 0.5] = 0
        xyz = cv2.reprojectImageTo3D(d, self.Q) if self.metric else None
        return l, r, d, xyz

    def heatmap(self, d, xyz):
        valid = d > 0
        heat = np.zeros((PH, PW, 3), np.uint8)
        if not valid.any():
            return heat
        if xyz is not None:   # metric: fixed scale, red = NEAR_MM, blue = FAR_MM
            z = xyz[..., 2]
            norm = np.clip((FAR_MM - z) / (FAR_MM - NEAR_MM) * 255, 0, 255)
        else:
            lo, hi = np.percentile(d[valid], [2, 98])
            norm = np.clip((d - lo) / max(hi - lo, 1e-3) * 255, 0, 255)
        heat = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_TURBO)
        heat[~valid] = 0
        return heat


def colorbar(width):
    """Scale under the metric heatmap."""
    bar = np.zeros((34, width, 3), np.uint8)
    grad = cv2.applyColorMap(np.linspace(255, 0, width).astype(np.uint8).reshape(1, -1), cv2.COLORMAP_TURBO)
    bar[:14] = cv2.resize(grad, (width, 14))
    for mm in (250, 500, 1000, 1500, 2000, 2500):
        x = int((mm - NEAR_MM) / (FAR_MM - NEAR_MM) * (width - 1))
        cv2.line(bar, (x, 10), (x, 18), (255, 255, 255), 1)
        cv2.putText(bar, f"{mm / 1000:g} m", (max(0, min(x - 14, width - 42)), 31),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return bar


# -- detection ---------------------------------------------------------------------------------

class Detector:
    def __init__(self):
        from ultralytics import YOLO
        self.model = YOLO(YOLO_WEIGHTS)
        self.names = self.model.names

    def detect(self, img, xyz, conf=0.35):
        """Objects with the median X/Y/Z (mm) of the middle half of their box."""
        out = []
        res = self.model.predict(img, imgsz=640, conf=conf, verbose=False)[0]
        for box, c, k in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy(),
                             res.boxes.cls.cpu().numpy().astype(int)):
            x1, y1, x2, y2 = box
            obj = {"name": self.names[k], "conf": float(c), "box": (int(x1), int(y1), int(x2), int(y2)), "xyz": None}
            if xyz is not None:
                cx, cy, hw, hh = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) / 4, (y2 - y1) / 4
                patch = xyz[int(cy - hh):int(cy + hh) + 1, int(cx - hw):int(cx + hw) + 1].reshape(-1, 3)
                patch = patch[np.isfinite(patch).all(1) & (patch[:, 2] > 0) & (patch[:, 2] < 10000)]
                if len(patch) > 20:
                    p = np.median(patch, axis=0)
                    obj["xyz"] = tuple(float(v) for v in p)
                    obj["width_mm"] = float((x2 - x1) * p[2] / self._focal)
                    obj["height_mm"] = float((y2 - y1) * p[2] / self._focal)
            out.append(obj)
        return out


class PalmFinder:
    """MediaPipe hand landmarks in BOTH raw full-size frames; the palm centre is
    triangulated directly (rectified point disparity), so it works even where
    the dense depth has holes."""

    def __init__(self):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        self.mp = mp
        self.hl = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=HAND_MODEL), num_hands=2,
            min_hand_detection_confidence=0.3, min_hand_presence_confidence=0.3))
        self.rot = 0   # 0 or 180: the image rotation that last found a hand

    def _hands(self, frame):
        for rot in (self.rot, 180 - self.rot):
            img = frame if rot == 0 else cv2.rotate(frame, cv2.ROTATE_180)
            r = self.hl.detect(self.mp.Image(image_format=self.mp.ImageFormat.SRGB,
                                             data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
            if r.hand_landmarks:
                self.rot = rot
                h, w = frame.shape[:2]
                out = []
                for lm in r.hand_landmarks:
                    pts = np.array([[p.x * w, p.y * h] for p in lm], np.float32)
                    if rot == 180:
                        pts = np.array([w - 1, h - 1], np.float32) - pts
                    out.append(pts)
                return out
        return []

    def find(self, left, right, st):
        """-> list of palms: centre (left raw px), width px, disparity px, xyz mm or None."""
        hl, hr = self._hands(left), self._hands(right)
        palms = []
        for a in hl:
            ca = a[PALM].mean(0)
            # partner in the right image: the hand at about the same height
            best = min(hr, key=lambda b: abs(b[PALM].mean(0)[1] - ca[1]), default=None)
            p = {"centre": ca, "width_px": float(np.linalg.norm(a[5] - a[17])), "disp": None, "xyz": None,
                 "pts": a}
            if best is not None:
                cb = best[PALM].mean(0)
                if st.metric:
                    K1, D1, R1, P1, K2, D2, R2, P2 = st.cam
                    s = PW / W   # P1/P2 are for the processing size
                    ra = cv2.undistortPoints(a[PALM].reshape(-1, 1, 2), K1, D1, R=R1, P=P1).reshape(-1, 2) / s
                    rb = cv2.undistortPoints(best[PALM].reshape(-1, 1, 2), K2, D2, R=R2, P=P2).reshape(-1, 2) / s
                    d = float(np.median(ra[:, 0] - rb[:, 0])) * s
                    p["disp"] = d / s
                    if d > 0:
                        u, v = ra.mean(0) * s
                        X = st.Q @ np.array([u, v, d, 1.0])
                        p["xyz"] = tuple(float(c) for c in X[:3] / X[3])
                else:
                    p["disp"] = float(ca[0] - cb[0])
            palms.append(p)
        return palms


def draw_palms(img, palms, scale, fb=None):
    for p in palms:
        for x, y in p["pts"][PALM] * scale:
            cv2.circle(img, (int(x), int(y)), 3, (255, 0, 255), -1)
        x, y = (p["centre"] * scale).astype(int)
        cv2.circle(img, (x, y), 8, (255, 0, 255), 2)
        if p["xyz"]:
            label = f"palm {np.linalg.norm(p['xyz']) / 10:.1f} cm"
        elif p["disp"] is not None and p["disp"] > 1 and fb is not None:
            label = f"palm ~{fb[0] / p['disp'] / 10:.1f} cm" + ("" if "one-point" in fb[1] else " (guess)")
        elif p["disp"] is not None:
            label = f"palm shift {p['disp']:.0f}px"
        else:
            label = "palm (left only)"
        cv2.putText(img, label, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2, cv2.LINE_AA)


def load_fb():
    """-> (FB, source text). One-point scale from the M key, else the guess."""
    try:
        import json
        with open(SCALE_FILE) as f:
            d = json.load(f)
        return float(d["fb"]), f"one-point scale ({d['distance_mm']:.0f} mm on {d['date']})"
    except (OSError, ValueError, KeyError):
        return FB_GUESS, "GUESS (typical lens values, not measured)"


def save_fb(fb, distance_mm, disparity):
    import json
    with open(SCALE_FILE, "w") as f:
        json.dump({"fb": fb, "distance_mm": distance_mm, "disparity_px": disparity,
                   "date": time.strftime("%Y-%m-%d %H:%M")}, f, indent=1)


def describe_palm(p, fb=None):
    if p["xyz"]:
        x, y, z = p["xyz"]
        return (f"PALM  distance {np.linalg.norm(p['xyz']):.0f} mm  X {x:+.0f}  Y {y:+.0f}  Z {z:.0f} mm"
                f"  (disparity {p['disp']:.1f} px)")
    if p["disp"] is not None:
        if fb is not None and p["disp"] > 1:
            fbv, src = fb
            return (f"PALM  distance ~{fbv / p['disp']:.0f} mm  ({fbv / p['disp'] / 10:.1f} cm)  "
                    f"from {p['disp']:.1f} px shift, scale: {src}")
        return f"PALM  seen in both lenses, disparity {p['disp']:.1f} px (raw)"
    return "PALM  seen in the left lens only"


def draw_objects(img, objs):
    for o in objs:
        x1, y1, x2, y2 = o["box"]
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = o["name"]
        if o["xyz"] is not None:
            label += f" {o['xyz'][2] / 1000:.2f} m"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, max(0, y1 - th - 6)), (x1 + tw + 4, max(th + 6, y1)), (0, 255, 0), -1)
        cv2.putText(img, label, (x1 + 2, max(th + 2, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def describe(o):
    s = f"{o['name']:<12} {o['conf']:.2f}"
    if o["xyz"] is not None:
        x, y, z = o["xyz"]
        s += (f"  distance {np.linalg.norm(o['xyz']) / 1000:.2f} m  X {x:+6.0f}  Y {y:+6.0f}  Z {z:6.0f} mm"
              f"  size ~{o['width_mm']:.0f} x {o['height_mm']:.0f} mm")
    return s


def make_stereo(args):
    st = Stereo(args.max_disp)
    if os.path.exists(args.calib):
        st.load_calibration(args.calib)
    return st


def make_detector(st, enabled):
    if not enabled:
        return None
    try:
        det = Detector()
        det._focal = st.focal if st.metric else 1.0
        return det
    except Exception as e:  # noqa: BLE001
        print("object detection off:", e)
        return None


# -- board + calibration -----------------------------------------------------------------------

def write_board(args):
    """Checkerboard for A4 landscape at 300 dpi, `--square` mm squares."""
    dpi = 300
    px = int(round(args.square / 25.4 * dpi))
    cols, rows = BOARD[0] + 1, BOARD[1] + 1
    margin = int(round(10 / 25.4 * dpi))
    page = np.full((int(210 / 25.4 * dpi), int(297 / 25.4 * dpi)), 255, np.uint8)
    if cols * px + 2 * margin > page.shape[1] or rows * px + 2 * margin > page.shape[0]:
        sys.exit(f"{args.square} mm squares do not fit on A4; use --square 25 or smaller")
    ox, oy = (page.shape[1] - cols * px) // 2, (page.shape[0] - rows * px) // 2
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                page[oy + r * px:oy + (r + 1) * px, ox + c * px:ox + (c + 1) * px] = 0
    cv2.putText(page, f"{BOARD[0]}x{BOARD[1]} inner corners, {args.square:g} mm squares - print at 100% (no fit to page), "
                "then MEASURE a square", (margin, page.shape[0] - margin // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
    from PIL import Image
    img = Image.fromarray(page)
    img.save(os.path.join(HERE, "stereo_board.png"), dpi=(dpi, dpi))
    img.save(os.path.join(HERE, "stereo_board.pdf"), resolution=dpi)
    print("wrote stereo_board.png and stereo_board.pdf (A4 landscape). Print at 100 % / actual size.")


def find_board(gray, fast):
    if fast:
        ok, c = cv2.findChessboardCorners(gray, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
                                          | cv2.CALIB_CB_FAST_CHECK)
        return c if ok else None
    ok, c = cv2.findChessboardCornersSB(gray, BOARD, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return c if ok else None


def calibrate(pairs, square, size):
    objp = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * square
    obj = [objp] * len(pairs)
    cl = [p[0] for p in pairs]
    cr = [p[1] for p in pairs]
    rms1, K1, D1, _, _ = cv2.calibrateCamera(obj, cl, size, None, None)
    rms2, K2, D2, _, _ = cv2.calibrateCamera(obj, cr, size, None, None)
    crit = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 200, 1e-6)
    rms, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(obj, cl, cr, K1, D1, K2, D2, size,
                                                         criteria=crit, flags=cv2.CALIB_FIX_INTRINSIC)
    return rms, rms1, rms2, K1, D1, K2, D2, R, T


def save_calibration(path, rms, K1, D1, K2, D2, R, T, devs, square, n):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_size", np.array([W, H], np.float64))
    for k, v in (("K1", K1), ("D1", D1), ("K2", K2), ("D2", D2), ("R", R), ("T", T)):
        fs.write(k, v)
    fs.write("rms", float(rms))
    fs.write("square_mm", float(square))
    fs.write("pairs", int(n))
    fs.write("left_device", devs[0])
    fs.write("right_device", devs[1])
    fs.write("date", time.strftime("%Y-%m-%d %H:%M"))
    fs.release()


def calib_gui(args):
    from PyQt5 import QtCore, QtWidgets

    devs, grabbers = open_pair(args.swap)
    pairs, last = [], {"t": 0.0, "corners": None, "still": None, "still_t": 0.0}
    app = QtWidgets.QApplication(sys.argv)
    win, labels, status = viewer_window(QtWidgets, "RV-2AJ stereo calibration", ["Left", "Right"])

    def capture(force=False):
        fa, fb, dt = get_pair(grabbers)
        if fa is None or fb is None or dt > 0.03:
            return "frames not in sync, hold still"
        gl, gr = cv2.cvtColor(fa, cv2.COLOR_BGR2GRAY), cv2.cvtColor(fb, cv2.COLOR_BGR2GRAY)
        cl, cr = find_board(gl, False), find_board(gr, False)
        if cl is None or cr is None:
            return "board not found in both views at full resolution"
        pairs.append((cl, cr))
        cv2.imwrite(os.path.join(args.pairs_dir, f"pair{len(pairs):02d}_L.png"), fa) if args.pairs_dir else None
        cv2.imwrite(os.path.join(args.pairs_dir, f"pair{len(pairs):02d}_R.png"), fb) if args.pairs_dir else None
        last["t"], last["corners"] = time.time(), cl.reshape(-1, 2).mean(0)
        return f"captured pair {len(pairs)}"

    msg = ["Hold the board so BOTH views see all of it."]

    def tick():
        fa, fb, dt = get_pair(grabbers)
        if fa is None or fb is None:
            return
        views = []
        found = []
        for f in (fa, fb):
            small = cv2.resize(f, (PW, PH))
            c = find_board(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), True)
            if c is not None:
                cv2.drawChessboardCorners(small, BOARD, c, True)
            found.append(c)
            views.append(small)
        now = time.time()
        if found[0] is not None and found[1] is not None:
            centre = found[0].reshape(-1, 2).mean(0) * 2
            if last["still"] is None or np.linalg.norm(centre - last["still"]) > 6:
                last["still"], last["still_t"] = centre, now
            new_place = last["corners"] is None or np.linalg.norm(centre - last["corners"]) > 60
            if new_place and now - last["still_t"] > 0.6 and now - last["t"] > 1.0:
                msg[0] = capture()
        else:
            last["still"] = None
        for lab, img in zip(labels, views):
            set_image(lab, img)
        state = "board seen in BOTH views" if all(c is not None for c in found) else \
            "board seen in: " + (", ".join(n for n, c in zip(("left", "right"), found) if c is not None) or "neither")
        status.setText(f"{state} | pairs: {len(pairs)} (need 20+, 25-30 is better) | {msg[0]}\n"
                       "Move it to a new place / distance / tilt and hold still ~1 s. SPACE capture, C calibrate, Q quit.")

    def do_calibrate():
        if len(pairs) < 10:
            msg[0] = f"only {len(pairs)} pairs, need at least 10 (20+ recommended)"
            return
        status.setText("calibrating ...")
        app.processEvents()
        rms, rms1, rms2, K1, D1, K2, D2, R, T = calibrate(pairs, args.square, (W, H))
        save_calibration(args.calib, rms, K1, D1, K2, D2, R, T, devs, args.square, len(pairs))
        msg[0] = (f"SAVED {os.path.basename(args.calib)}: stereo RMS {rms:.2f} px (left {rms1:.2f}, right {rms2:.2f}), "
                  f"baseline {np.linalg.norm(T):.1f} mm. Below 0.5 px is good; above 1 px recalibrate. Q to quit.")
        print(msg[0])

    def on_key(k):
        if k == QtCore.Qt.Key_Space:
            msg[0] = capture(True)
        elif k == QtCore.Qt.Key_C:
            do_calibrate()

    run_window(app, win, tick, on_key, grabbers)


# -- viewer ------------------------------------------------------------------------------------

def viewer_window(QtWidgets, title, titles):
    from PyQt5 import QtCore
    win = QtWidgets.QWidget()
    win.setWindowTitle(title)
    grid = QtWidgets.QGridLayout(win)
    labels = []
    for i, t in enumerate(titles):
        r, c, span = (0, i, 1) if i < 2 else (2, 0, 2)
        grid.addWidget(QtWidgets.QLabel(t), r, c, 1, span)
        img = QtWidgets.QLabel()
        img.setMinimumSize(PW // 2, PH // 2)
        img.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        img.setAlignment(QtCore.Qt.AlignCenter)
        img.setMouseTracking(True)
        grid.addWidget(img, r + 1, c, 1, span)
        labels.append(img)
    grid.setRowStretch(1, 1)
    if len(titles) > 2:
        grid.setRowStretch(3, 2)
    status = QtWidgets.QLabel()
    status.setWordWrap(True)
    status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    grid.addWidget(status, 4, 0, 1, 2)
    win.resize(1300, 950)
    return win, labels, status


def set_image(label, img):
    from PyQt5 import QtCore, QtGui
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    pix = QtGui.QPixmap.fromImage(QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy())
    label.setPixmap(pix.scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
    label.img_size = (w, h)


def label_to_image(label, pos):
    """Mouse position on a QLabel -> pixel in the image shown in it."""
    pix = label.pixmap()
    if pix is None or not hasattr(label, "img_size"):
        return None
    ox = (label.width() - pix.width()) / 2
    oy = (label.height() - pix.height()) / 2
    x = (pos.x() - ox) * label.img_size[0] / pix.width()
    y = (pos.y() - oy) * label.img_size[1] / pix.height()
    if 0 <= x < label.img_size[0] and 0 <= y < label.img_size[1]:
        return int(x), int(y)
    return None


def run_window(app, win, tick, on_key, grabbers, on_mouse=None):
    from PyQt5 import QtCore

    class Events(QtCore.QObject):
        def eventFilter(self, obj, ev):
            if ev.type() == QtCore.QEvent.KeyPress:
                if ev.key() in (QtCore.Qt.Key_Q, QtCore.Qt.Key_Escape):
                    win.close()
                else:
                    on_key(ev.key())
            elif ev.type() == QtCore.QEvent.MouseMove and on_mouse is not None:
                on_mouse(obj, ev.pos())
            return False

    events = Events()
    app.installEventFilter(events)
    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(40)
    win.show()
    rc = app.exec_()
    for g in grabbers:
        g.running = False
    time.sleep(0.2)
    sys.exit(rc)


def gui(args):
    from PyQt5 import QtCore, QtWidgets

    devs, grabbers = open_pair(args.swap)
    st = make_stereo(args)
    det = make_detector(st, not args.no_detect)
    palm = PalmFinder() if not args.no_palm else None
    fb_scale = [load_fb()]
    recent = []   # last palm disparities, for the M key
    app = QtWidgets.QApplication(sys.argv)
    heat_title = ("Depth heatmap (red = near, blue = far, black = no match)" if st.metric else
                  "Relative depth heatmap - NOT calibrated, run --calibrate for millimetres")
    win, labels, status = viewer_window(QtWidgets, "RV-2AJ stereo camera",
                                        ["Left (rectified) + objects", "Right (rectified)", heat_title])
    state = {"t": time.time(), "n": [0, 0], "fps": [0.0, 0.0], "proc": 0.0, "xyz": None, "d": None,
             "objs": [], "detect": det is not None, "hover": ""}

    def tick():
        fa, fb, dt = get_pair(grabbers)
        if fa is None or fb is None:
            status.setText("waiting for frames ...")
            return
        if not st.metric and st.H1 is None:
            st.rectify_from(fa, fb)
        t0 = time.time()
        l, r, d, xyz = st.compute(fa, fb)
        heat = st.heatmap(d, xyz)
        # Uncalibrated, the rectified view is warped: show and detect on the plain left image.
        view = l if st.metric else cv2.resize(fa, (PW, PH))
        objs = det.detect(view, xyz) if det is not None and state["detect"] else []
        palms = palm.find(fa, fb, st) if palm is not None else []
        if palms and palms[0]["disp"] is not None and not st.metric:
            recent.append(palms[0]["disp"])
            del recent[:-10]
        state.update(xyz=xyz, d=d, objs=objs)
        state["proc"] = 0.8 * state["proc"] + 0.2 * (time.time() - t0)
        now = time.time()
        if now - state["t"] > 1.0:
            for i, g in enumerate(grabbers):
                state["fps"][i] = (g.count - state["n"][i]) / (now - state["t"])
                state["n"][i] = g.count
            state["t"] = now
        shown = view.copy()
        draw_objects(shown, objs)
        draw_palms(shown, palms, PW / W, fb_scale[0])
        heat_shown = heat.copy()
        if st.metric:
            draw_objects(heat_shown, objs)
        if st.metric:
            heat_shown = np.vstack([heat_shown, colorbar(PW)])
        for lab, img in zip(labels, (shown, r, heat_shown)):
            set_image(lab, img)
        lines = [f"L={devs[0]} R={devs[1]} | camera fps {state['fps'][0]:.0f}/{state['fps'][1]:.0f} | "
                 f"sync {dt * 1000:.0f} ms | processing {state['proc'] * 1000:.0f} ms | {st.info}"]
        valid = d > 0
        if st.metric and valid.any():
            z = xyz[..., 2][valid]
            lines.append(f"depth over {100 * valid.mean():.0f}% of pixels: nearest {np.percentile(z, 2):.0f} mm, "
                         f"median {np.median(z):.0f} mm, farthest {np.percentile(z, 98):.0f} mm")
        lines += [describe_palm(p, fb_scale[0]) for p in palms]
        lines += [describe(o) for o in objs[:8]]
        if state["hover"]:
            lines.append(state["hover"])
        if not st.metric:
            lines.append("Palm distance: hold the palm at a distance you MEASURED and press M to fix the scale.")
        lines.append("Mouse over the left image or heatmap = distance there | M set scale, D detection on/off, S swap, R re-rectify, Q quit")
        status.setText("\n".join(lines))

    def on_mouse(obj, pos):
        if obj not in (labels[0], labels[2]):
            return
        p = label_to_image(obj, pos)
        if p is None or p[1] >= PH or state["d"] is None:
            return
        x, y = p
        if state["d"][y, x] <= 0:
            state["hover"] = f"pixel ({x},{y}): no depth here"
        elif state["xyz"] is not None:
            X, Y, Z = state["xyz"][y, x]
            state["hover"] = f"pixel ({x},{y}): X {X:+.0f}  Y {Y:+.0f}  Z {Z:.0f} mm  (distance {np.sqrt(X*X + Y*Y + Z*Z) / 1000:.3f} m)"
        else:
            state["hover"] = f"pixel ({x},{y}): disparity {state['d'][y, x]:.1f} px (relative)"

    def on_key(k):
        if k == QtCore.Qt.Key_R and not st.metric:
            st.H1 = None
        elif k == QtCore.Qt.Key_D:
            state["detect"] = not state["detect"]
        elif k == QtCore.Qt.Key_M and not st.metric:
            if len(recent) < 3:
                state["hover"] = "M: show the palm to BOTH lenses first"
                return
            disp = float(np.median(recent))
            cm, ok = QtWidgets.QInputDialog.getDouble(win, "Set the distance scale",
                                                      f"Palm shift now {disp:.1f} px.\nReal distance from the lenses to the palm (cm):",
                                                      50.0, 5.0, 500.0, 1)
            if ok:
                save_fb(disp * cm * 10.0, cm * 10.0, disp)
                fb_scale[0] = load_fb()
                state["hover"] = f"scale saved: FB = {disp * cm * 10:.0f} (stereo_scale.json)"
        elif k == QtCore.Qt.Key_S:
            grabbers.reverse()
            devs.reverse()
            st.H1 = None

    run_window(app, win, tick, on_key, grabbers, on_mouse)


def snapshot(args):
    devs, grabbers = open_pair(args.swap)
    time.sleep(1.5)
    fa, fb, dt = get_pair(grabbers)
    for g in grabbers:
        g.running = False
    print("stereo cameras:", devs, f"sync {dt * 1000:.0f} ms")
    st = make_stereo(args)
    if not st.metric:
        st.rectify_from(fa, fb)
    print(st.info)
    l, r, d, xyz = st.compute(fa, fb)
    heat = st.heatmap(d, xyz)
    det = make_detector(st, not args.no_detect)
    objs = det.detect(l, xyz) if det else []
    palms = PalmFinder().find(fa, fb, st) if not args.no_palm else []
    for p in palms:
        print("  " + describe_palm(p, load_fb()))
    draw_palms(l, palms, PW / W, load_fb())
    valid = d > 0
    if st.metric and valid.any():
        z = xyz[..., 2][valid]
        print(f"depth over {100 * valid.mean():.0f}% of pixels: nearest {np.percentile(z, 2):.0f} mm, "
              f"median {np.median(z):.0f} mm, farthest {np.percentile(z, 98):.0f} mm")
    print(f"{len(objs)} objects:")
    for o in objs:
        print("  " + describe(o))
    draw_objects(l, objs)
    draw_objects(heat, objs)
    cv2.imwrite(args.out, np.hstack([l, r, heat]))
    print("wrote", args.out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", action="store_true", help="one frame, no window")
    ap.add_argument("--calibrate", action="store_true", help="checkerboard stereo calibration window")
    ap.add_argument("--board", action="store_true", help="write the printable checkerboard")
    ap.add_argument("--square", type=float, default=25.0, help="checkerboard square size in mm (MEASURE the print)")
    ap.add_argument("--calib", default=CALIB)
    ap.add_argument("--pairs_dir", default="", help="also save the captured calibration images here")
    ap.add_argument("--swap", action="store_true", help="treat the second camera as the left one")
    ap.add_argument("--max_disp", type=int, default=128, help="disparity search range (multiple of 16); larger = nearer objects")
    ap.add_argument("--no_palm", action="store_true", help="no MediaPipe palm tracking")
    ap.add_argument("--no_detect", action="store_true", help="no YOLO object detection")
    ap.add_argument("--out", default="stereo_snapshot.jpg")
    a = ap.parse_args()
    if a.board:
        write_board(a)
    elif a.calibrate:
        calib_gui(a)
    elif a.snapshot:
        snapshot(a)
    else:
        gui(a)
