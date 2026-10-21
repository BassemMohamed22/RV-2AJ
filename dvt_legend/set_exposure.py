#!/usr/bin/env python3
"""Set the exposure time of DVT product 1 (RAM only, nothing saved to flash) and report whether
the image is still saturated. Documented command: `#PP <product> E <microseconds>`
(vendor Ethernet command reference, Product_Management_Commands).

    python3 set_exposure.py 1000      # try 1 ms
    python3 set_exposure.py 300
    python3 set_exposure.py 8300      # RESTORE the original value (it was 8300 us)

Every change is appended to exposure_changes.log. A camera power cycle drops the change."""
import re, socket, sys, time
import numpy as np, cv2

IP, PRODUCT = "192.168.10.242", 1


def conn():
    s = socket.create_connection((IP, 3246), timeout=6); s.settimeout(3)
    try: s.recv(100)
    except socket.timeout: pass
    return s


def talk(s, cmd, wait=40):
    s.sendall(cmd + b"\r"); out = b""; s.settimeout(wait)
    while True:
        d = s.recv(262144)
        if not d: raise ConnectionError("camera closed")
        out += d
        if re.search(rb"%\s*\d+\s*\r\n[?!]\s*$", out): return out


def acquisition(s):
    """The product's acquisition line from `#PQ <id>`: the third `$` line (exposure, antiblooming,
    illumination, delay, antiblooming level, gain, window ...)."""
    lines = re.findall(r"^\$\s*(.*?)\s*$", talk(s, b"#PQ %d" % PRODUCT).decode("latin-1").replace("\r", ""), re.M)
    return lines[2] if len(lines) > 2 else f"(unexpected reply: {lines})"


def grab(s):
    time.sleep(1.5)  # let a frame taken with the new exposure reach the saved queue
    ids = [int(x) for x in re.findall(rb"\$ (\d+)\s+\d+", talk(s, b"#IQ"))[1:]]
    new = max(ids)
    r = talk(s, b"#IS %d 0 0 639 479 0 2 2 0 0" % new)
    pix = []
    for line in r.replace(b"\r", b"\n").split(b"\n"):
        if line.startswith(b"# 18"): pix += [int(x) for x in line.split()[3:]]
    return new, np.array(pix[:214 * 160], np.uint8)


if __name__ == "__main__":
    us = int(sys.argv[1])
    s = conn()
    before = acquisition(s)
    r = talk(s, b"#PP %d E %d" % (PRODUCT, us)).decode().replace("\r", "").strip().replace("\n", " ")
    after = acquisition(s)
    with open("exposure_changes.log", "a") as f:
        f.write(f"{time.strftime('%F %T')} #PP {PRODUCT} E {us} -> {r} | before: {before} | after: {after}\n")
    print(f"#PP {PRODUCT} E {us} -> {r}\nbefore: {before}\nafter:  {after}")
    for _ in range(3):
        n, a = grab(s)
        print(f"image {n}: min {a.min()} mean {a.mean():.1f} max {a.max()} saturated {100 * np.mean(a >= 250):.0f}%")
    cv2.imwrite(f"exposure_{us}.png", a.reshape(160, 214))
    print(f"saved exposure_{us}.png (214x160)")
