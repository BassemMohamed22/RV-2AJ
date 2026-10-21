#!/usr/bin/env python3
"""Carve the raw-deflate streams out of a Wise installer overlay (no execution of the installer)."""
import sys, zlib, os, struct
d = open(sys.argv[1], "rb").read()
outdir = sys.argv[2]; os.makedirs(outdir, exist_ok=True)
pos = 14848 + 177
MAGIC = [(b"MZ", "exe"), (b"ITSF", "chm"), (b"%PDF", "pdf"), (b"PK\x03\x04", "zip"), (b"MSCF", "cab"), (b"{\\rtf", "rtf"),
         (b"BM", "bmp"), (b"\x28\x00\x00\x00", "bmp"), (b"GIF8", "gif"), (b"\x89PNG", "png"), (b"\xff\xd8", "jpg"),
         (b"<html", "htm"), (b"<HTML", "htm"), (b"<!DOC", "htm"), (b"<?xml", "xml"), (b"\xd0\xcf\x11\xe0", "ole"),
         (b"\x00\x00\x01\x00", "ico"), (b"RIFF", "riff")]
def ext(b):
    for m, e in MAGIC:
        if b.startswith(m): return e
    t = b[:200]
    if t and all(32 <= c < 127 or c in (9, 10, 13) for c in t): return "txt"
    return "bin"
n = 0; fails = 0
while pos < len(d) - 8 and fails < 200000:
    ok = False
    for skip in range(0, 96):
        s = pos + skip
        try:
            o = zlib.decompressobj(-15); out = o.decompress(d[s:s + 4_000_000])
            if o.eof and len(out) > 0:
                used = len(d[s:s + 4_000_000]) - len(o.unused_data)
                if skip > 0 or True:
                    name = f"{n:04d}_{s:08x}.{ext(out)}"
                    open(os.path.join(outdir, name), "wb").write(out)
                    crc_ok = struct.unpack_from("<I", d, s + used)[0] == (zlib.crc32(out) & 0xffffffff) if s + used + 4 <= len(d) else False
                    print(f"{name} {len(out)} bytes  skip={skip} crc_after={crc_ok}")
                    n += 1; pos = s + used + (4 if crc_ok else 0); ok = True; break
        except Exception:
            continue
    if not ok:
        pos += 1; fails += 1
print("carved", n, "streams; stopped at", pos, "of", len(d))
