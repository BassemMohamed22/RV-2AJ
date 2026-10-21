#!/usr/bin/env python3
"""Live view of the DVT 510MR (192.168.10.242). READ-ONLY.
Images: system terminal port 3246, documented commands `#IQ` (list saved images) and
`#IS <id> x1 y1 x2 y2 0 rowskip colskip 0 0` (send image). Results: VDX stream port 7500 (receive only).
Keys: 1 = full 640x480 (~22 s/frame), 2 = 320x240 (~5.5 s), 3 = 213x160 (~2.5 s), A = auto-contrast, S = save PNG, Q = quit.
Run from a normal terminal (VS Code's snap terminal breaks PyQt)."""
import re, socket, sys, threading, time
import cv2, numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

IP = "192.168.10.242"
state = {"skip": 2, "frame": None, "id": None, "t": 0.0, "dt": 0.0, "result": "?", "cycle": None,
         "err": "", "auto": True, "n": 0, "run": True}

def conn(port):
    s = socket.create_connection((IP, port), timeout=6); s.settimeout(3)
    return s

def talk(s, cmd, wait=40):
    s.sendall(cmd + b"\r"); out = b""; s.settimeout(wait)
    while True:
        d = s.recv(262144)
        if not d: raise ConnectionError("camera closed")
        out += d
        if re.search(rb"%\s*\d+\s*\r\n[?!]\s*$", out): return out

def fetch_loop():
    while state["run"]:
        try:
            s = conn(3246)
            try: s.recv(100)
            except socket.timeout: pass
            while state["run"]:
                t0 = time.time()
                ids = [int(x) for x in re.findall(rb"\$ (\d+)\s+\d+", talk(s, b"#IQ"))[1:]]
                if not ids: time.sleep(1); continue
                sk = state["skip"]; new = max(ids)
                r = talk(s, b"#IS %d 0 0 639 479 0 %d %d 0 0" % (new, sk, sk))
                if not r.rstrip().endswith(b"?"): state["err"] = "IS refused (image gone), retrying"; continue
                w, h = -(-640 // (sk + 1)), -(-480 // (sk + 1))
                pix = []
                for line in r.replace(b"\r", b"\n").split(b"\n"):
                    if line.startswith(b"# 18"): pix += [int(x) for x in line.split()[3:]]
                if len(pix) < w * h: state["err"] = f"short frame {len(pix)}/{w*h}"; continue
                state.update(frame=np.array(pix[:w * h], np.uint8).reshape(h, w), id=new, t=time.time(),
                             dt=time.time() - t0, err="", n=state["n"] + 1)
        except Exception as e:
            state["err"] = f"{type(e).__name__}: {e}"; time.sleep(2)

def result_loop():
    names = ("PASS", "WARN", "FAIL", "WRONGCODE")
    while state["run"]:
        try:
            s = conn(7500); s.settimeout(5); buf = b""
            while state["run"]:
                buf += s.recv(4096)
                while len(buf) >= 64:
                    t, buf = buf[:64], buf[64:]
                    state["result"] = "+".join(n for b, n in enumerate(names) if t[0] >> b & 1) or "none"
                    state["cycle"] = t[2]
        except Exception: time.sleep(2)

app = QtWidgets.QApplication(sys.argv)
win = QtWidgets.QWidget(); win.setWindowTitle("DVT 510MR live view (read-only)")
lay = QtWidgets.QVBoxLayout(win)
img = QtWidgets.QLabel(); img.setMinimumSize(640, 480); img.setAlignment(QtCore.Qt.AlignCenter)
img.setStyleSheet("background:#111")
info = QtWidgets.QLabel(); info.setWordWrap(True)
lay.addWidget(img, 1); lay.addWidget(info)
shown = {"n": -1}

def tick():
    f = state["frame"]
    if f is not None and shown["n"] != state["n"]:
        shown["n"] = state["n"]
        sat = float((f >= 250).mean()) * 100
        v = f
        if state["auto"] and f.max() > f.min():
            lo, hi = np.percentile(f, [1, 99.5]); v = np.clip((f.astype(float) - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
        v = cv2.resize(v, (640, 480), interpolation=cv2.INTER_NEAREST if f.shape[1] < 320 else cv2.INTER_LINEAR)
        qi = QtGui.QImage(v.data, 640, 480, 640, QtGui.QImage.Format_Grayscale8).copy()
        img.setPixmap(QtGui.QPixmap.fromImage(qi))
        state["sat"] = sat; state["rng"] = (int(f.min()), int(f.max()))
    age = time.time() - state["t"] if state["t"] else 0
    sat = state.get("sat", 0)
    warn = "   *** SENSOR SATURATED: nearly all pixels at 255, nothing to see (lens lit/too bright) ***" if sat > 95 else ""
    info.setText(f"image id {state['id']} | {state['frame'].shape[1] if state['frame'] is not None else '-'}x"
                 f"{state['frame'].shape[0] if state['frame'] is not None else '-'} | {state['dt']:.1f} s/frame | age {age:.0f} s | "
                 f"pixel range {state.get('rng','-')} | saturated {sat:.0f}%{warn}\n"
                 f"inspection result (port 7500): {state['result']}  cycle {state['cycle']}   {state['err']}\n"
                 f"keys: 1 full-res (slow)  2 half  3 small (fast)  A auto-contrast {'ON' if state['auto'] else 'off'}  S save  Q quit")

class Keys(QtCore.QObject):
    def eventFilter(self, o, e):
        if e.type() == QtCore.QEvent.KeyPress:
            k = e.key()
            if k == QtCore.Qt.Key_Q: win.close()
            elif k == QtCore.Qt.Key_1: state["skip"] = 0
            elif k == QtCore.Qt.Key_2: state["skip"] = 1
            elif k == QtCore.Qt.Key_3: state["skip"] = 2
            elif k == QtCore.Qt.Key_A: state["auto"] = not state["auto"]; shown["n"] = -1
            elif k == QtCore.Qt.Key_S and state["frame"] is not None:
                p = time.strftime("dvt_%H%M%S.png"); cv2.imwrite(p, state["frame"]); print("saved", p)
        return False
keys = Keys(); app.installEventFilter(keys)
timer = QtCore.QTimer(); timer.timeout.connect(tick); timer.start(250)
threading.Thread(target=fetch_loop, daemon=True).start()
threading.Thread(target=result_loop, daemon=True).start()
win.resize(700, 640); win.show()
rc = app.exec_(); state["run"] = False; sys.exit(rc)
