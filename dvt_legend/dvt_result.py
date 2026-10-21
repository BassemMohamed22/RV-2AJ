#!/usr/bin/env python3
"""Read DVT inspection results from the VDX output stream (port 7500). RECEIVE ONLY: sends nothing.
Documented layout: SR (Script Reference) text/0098_017eeb30.txt:5396. Needs the camera at 192.168.10.242."""
import socket, sys, time
IP, PORT = "192.168.10.242", 7500
s = socket.create_connection((IP, PORT), timeout=5)
n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
buf = b""
while n > 0:
    buf += s.recv(4096)
    while len(buf) >= 64 and n > 0:
        t, buf = buf[:64], buf[64:]
        flags = [name for bit, name in enumerate(("PASS", "WARN", "FAIL", "WRONG_CODE")) if t[0] >> bit & 1]
        print(f"{time.strftime('%H:%M:%S')}  cycle {t[2]:3d}  result {'+'.join(flags) or 'none'}  error_code {t[1]}  user_bits {t[3]:08b}")
        n -= 1
