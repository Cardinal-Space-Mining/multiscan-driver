"""Capture raw multiScan UDP datagrams to a file.

File format (all little endian), shared with the C++ replay reader:
  header : b"MSCAP001"
  record : u64 recv_time_ns (CLOCK_REALTIME, kernel SO_TIMESTAMPNS), u16 port, u32 length, bytes[length]

usage: mscap_capture.py OUT_FILE SECONDS PORT [PORT ...]

The sensor must be streaming to this host (ScanDataEthSettings / ImuDataEthSettings).
"""
import select
import socket
import struct
import sys
import time

SO_TIMESTAMPNS = 35

out, seconds, ports = sys.argv[1], float(sys.argv[2]), [int(p) for p in sys.argv[3:]]
socks = []
for p in ports:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    s.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
    s.bind(("", p))
    socks.append((s, p))

n = {p: 0 for p in ports}
nbytes = {p: 0 for p in ports}
with open(out, "wb") as f:
    f.write(b"MSCAP001")
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([s for s, _ in socks], [], [], 0.2)
        for s, p in socks:
            if s not in r:
                continue
            data, anc, _, _ = s.recvmsg(70000, 64)
            t_ns = time.time_ns()
            for lvl, typ, val in anc:
                if lvl == socket.SOL_SOCKET and typ == SO_TIMESTAMPNS:
                    sec, nsec = struct.unpack("qq", val[:16])
                    t_ns = sec * 1_000_000_000 + nsec
            f.write(struct.pack("<QHI", t_ns, p, len(data)) + data)
            n[p] += 1
            nbytes[p] += len(data)
for p in ports:
    print(f"port {p}: {n[p]} datagrams, {nbytes[p]} bytes, {n[p] / seconds:.1f}/s, "
          f"{nbytes[p] * 8 / seconds / 1e6:.2f} Mbit/s, avg {nbytes[p] / max(1, n[p]):.0f} B")
