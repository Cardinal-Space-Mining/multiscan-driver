"""Trim an MSCAP001 capture to N whole frames (starting at the first segment-0 packet) and gzip it.

usage: mscap_trim.py IN_FILE OUT_FILE.gz N_FRAMES
"""
import gzip, struct, sys
src, dst, nframes = sys.argv[1], sys.argv[2], int(sys.argv[3])
d = open(src, "rb").read(); o = 8; recs = []
while o < len(d):
    t, p, n = struct.unpack_from("<QHI", d, o); recs.append((t, p, d[o:o + 14 + n])); o += 14 + n
def seg_frame(rec):
    b = rec[2][14:]
    if struct.unpack_from("<I", b, 4)[0] != 1: return None
    return struct.unpack_from("<QQ", b, 32)
out, frames, started = [], [], False
for r in recs:
    sf = seg_frame(r)
    if sf and not started and sf[0] == 0: started = True
    if not started: continue
    if sf:
        if sf[1] not in frames: frames.append(sf[1])
        if len(frames) > nframes: break
    out.append(r[2])
with gzip.open(dst, "wb", compresslevel=9) as f:
    f.write(b"MSCAP001"); [f.write(x) for x in out]
print(dst, len(out), "records", len(frames[:nframes]), "frames")
