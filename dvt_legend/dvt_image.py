#!/usr/bin/env python3
"""Fetch the newest image from a DVT SmartImage Sensor over the system terminal (port 3246).
READ-ONLY: sends only the documented `Is` command ("Send the newest image as a set of packets",
vendor Ethernet command reference, Image Management Commands) in binary mode.
Packet layout: Image_Data_Format_and_Sequence / Binary_Packet_Format (big endian)."""
import socket, struct, sys, zlib

IP, PORT = "192.168.10.242", 3246

def recv_exact(s, n):
    b = b""
    while len(b) < n:
        d = s.recv(n - len(b))
        if not d: raise ConnectionError("closed")
        b += d
    return b

def read_packet(s):
    t = recv_exact(s, 1)[0]
    ln = struct.unpack(">H", recv_exact(s, 2))[0]
    return t, recv_exact(s, ln)

def fetch(s, verbose=False):
    """-> (width, height, bytes) of the newest image."""
    s.sendall(b"\xAA" + struct.pack(">H", 2) + b"Is")
    hdr, rows, total = None, [], b""
    while True:
        t, data = read_packet(s)
        if verbose: print(f"  frame type 0x{t:02X} len {len(data)} first {data[:12].hex(' ')}")
        if t in (0xA9,):                    # end reply
            if verbose: print("  end reply", data.hex(" "))
            break
        if t == 0xA8 and data:
            pt = data[0]
            if pt == 16: hdr = data
            elif pt == 18: total += data[5:]
            elif pt == 17 or pt == 21:
                if pt == 21: pass
    if hdr is None: raise RuntimeError("no image header received")
    # <16:U8><ID:I32><x1,y1,x2,y2:I16><fmt:U8><plane:U16><0:U16><rowskip:U8><colskip:U8>...
    iid, x1, y1, x2, y2, fmt, plane, _, rs, cs = struct.unpack_from(">iHHHHBHHBB", hdr, 1)
    return hdr, total

if __name__ == "__main__":
    s = socket.create_connection((IP, PORT), timeout=8)
    s.settimeout(8)
    try: s.recv(100)                       # banner
    except socket.timeout: pass
    hdr, data = fetch(s, verbose=True)
    print("header", hdr.hex(" "))
    print("image bytes", len(data))
    open("last_image.raw", "wb").write(data)
