#!/usr/bin/env python3
"""Measure the multiScan136 beam geometry from the driver's point clouds.

The driver decodes every point's elevation and azimuth from the angles the
sensor sends for each beam (Compact format: Phi per layer, Theta per beam).
This tool collects them over N frames and reports, per layer:

  - elevation (and whether it ever varies),
  - the azimuth of index 0 and the step between indices (line fit),
  - how far single beams sit off that line, and how much a beam's azimuth
    moves between frames,

then prints ELEVATION_LUT / AZIMUTH_OFFSET_LUT in the form used by
include/multiscan_driver/multiscan_spec.hpp, next to the values there now,
and can write every beam's angles to JSON.

Needs the fields layer, index, azimuth, elevation (and echo if published) in
output.point_fields; the defaults have them. Layers 6 and 14 (1-based) are the
0.125 deg high-resolution layers; they only appear if sensor.layers enables
them. Interlaced mode shifts the azimuths every frame and is reported.

    ros2 run multiscan_driver ms_geometry.py --frames 200 --json geometry.json
    python3 tools/ms_geometry.py --topic /multiscan/lidar_scan
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

TWO_PI = 2 * math.pi
NUM_LAYERS = 16
HD_LAYERS = (5, 13)  # 0-based layer ids of the high-resolution layers
SPEC = Path(__file__).resolve().parents[1] / "include" / "multiscan_driver" / "multiscan_spec.hpp"

DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_array(msg):
    """PointCloud2 -> numpy structured array (no copy)."""
    dt = np.dtype({
        "names": [f.name for f in msg.fields],
        "formats": [DTYPES[f.datatype] for f in msg.fields],
        "offsets": [f.offset for f in msg.fields],
        "itemsize": msg.point_step,
    })
    return np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)


class Collector:
    def __init__(self):
        # per (layer, index): sum of unit vectors (circular mean), min/max
        # deviation from the first sample, count
        self.az = {}
        self.el = {l: set() for l in range(NUM_LAYERS)}
        self.frames = 0

    def add(self, pts):
        names = pts.dtype.names
        for need in ("layer", "index", "azimuth", "elevation"):
            if need not in names:
                sys.exit(f"the cloud has no '{need}' field; add it to output.point_fields")
        if "echo" in names:  # one sample per beam: the lowest echo present
            pts = pts[np.lexsort((pts["echo"], pts["index"], pts["layer"]))]
        key = pts["layer"].astype(np.int64) * 100000 + pts["index"].astype(np.int64)
        _, first = np.unique(key, return_index=True)
        pts = pts[first]
        for layer in np.unique(pts["layer"]):
            p = pts[pts["layer"] == layer]
            l = int(layer)
            self.el[l].update(np.unique(p["elevation"]).tolist())
            for i, a in zip(p["index"].tolist(), p["azimuth"].astype(np.float64).tolist()):
                s = self.az.get((l, i))
                if s is None:
                    self.az[(l, i)] = [math.cos(a), math.sin(a), a, 0.0, 0.0, 1]
                else:
                    s[0] += math.cos(a)
                    s[1] += math.sin(a)
                    d = math.remainder(a - s[2], TWO_PI)
                    s[3] = min(s[3], d)
                    s[4] = max(s[4], d)
                    s[5] += 1
        self.frames += 1

    def layers(self):
        out = {}
        for l in range(NUM_LAYERS):
            idx = sorted(i for (ll, i) in self.az if ll == l)
            if not idx:
                continue
            idx = np.array(idx)
            mean = np.array([math.atan2(self.az[(l, i)][1], self.az[(l, i)][0]) for i in idx])
            jitter = np.array([self.az[(l, i)][4] - self.az[(l, i)][3] for i in idx])
            # line fit on the unwrapped azimuths (they increase with index)
            un = np.unwrap(mean)
            if un[-1] < un[0]:
                un = -np.unwrap(-mean)
            b, a = np.polyfit(idx, un, 1)
            resid = un - (a + b * idx)
            els = sorted(self.el[l])
            out[l] = {
                "elevation": float(np.median(els)),
                "elevation_spread": float(els[-1] - els[0]),
                "offset": float(a % TWO_PI),   # azimuth of index 0, [0, 2pi)
                "step": float(b),
                "beams": int(len(idx)),
                "max_residual": float(np.abs(resid).max()),
                "max_jitter": float(jitter.max()),
                "azimuth": {int(i): float(m % TWO_PI) for i, m in zip(idx, mean)},
            }
        return out


def read_spec():
    """Current ELEVATION_LUT / AZIMUTH_OFFSET_LUT from multiscan_spec.hpp."""
    if not SPEC.exists():
        return None, None
    text = SPEC.read_text()

    def table(name):
        m = re.search(name + r"\s*\{([^}]*)\}", text)
        return [float(v) for v in re.findall(r"-?\d+\.\d+", m.group(1))] if m else None
    return table("ELEVATION_LUT"), table("AZIMUTH_OFFSET_LUT")


def report(geo, frames):
    deg = math.degrees
    print(f"{frames} frames\n")
    print("layer  beams  elevation [rad]   (deg)    spread   az(index 0) [rad]  step [deg]  "
          "off-line max [deg]  frame-to-frame max [deg]")
    for l, g in geo.items():
        hd = " HD" if l in HD_LAYERS else ""
        print(f"{l:4d}{hd:3s} {g['beams']:5d}  {g['elevation']:+.6f}  ({deg(g['elevation']):+7.3f})  "
              f"{g['elevation_spread']:.1e}  {g['offset']:.6f}          {deg(g['step']):.6f}    "
              f"{deg(g['max_residual']):.4f}              {deg(g['max_jitter']):.4f}")

    jitter = max(g["max_jitter"] for g in geo.values())
    if deg(jitter) > 0.05:
        print("\nWARNING: beam azimuths move between frames by up to %.3f deg: interlaced mode? "
              "A fixed table cannot describe this." % deg(jitter))

    lr = [l for l in range(NUM_LAYERS) if l not in HD_LAYERS and l in geo]
    if len(lr) != NUM_LAYERS - len(HD_LAYERS):
        print(f"\nOnly {len(lr)} of 14 low-resolution layers present; no LUT printed.")
        return
    el = [geo[l]["elevation"] for l in lr]
    off = [geo[l]["offset"] for l in lr]
    print("\n// measured (dense layer order: layer ids " + ", ".join(map(str, lr)) + ")")
    print("constexpr std::array<float, NUM_LR_LAYERS> ELEVATION_LUT{\n    "
          + ",\n    ".join(f"{v:.6f}" for v in el) + "};")
    print("constexpr std::array<float, NUM_LR_LAYERS> AZIMUTH_OFFSET_LUT{\n    "
          + ",\n    ".join(f"{v:.6f}" for v in off) + "};")
    steps = [geo[l]["step"] for l in lr]
    print(f"// step: {min(map(deg, steps)):.6f} .. {max(map(deg, steps)):.6f} deg "
          f"(LR_LAYER_AZIMUTH_STEP_RAD = 2 pi / 360 = 1 deg)")

    cur_el, cur_off = read_spec()
    if cur_el and cur_off:
        print("\nmeasured - multiscan_spec.hpp:")
        print("dense  elevation [deg]  az offset [deg]")
        for d, l in enumerate(lr):
            de = deg(el[d] - cur_el[d])
            do = deg(math.remainder(off[d] - cur_off[d], TWO_PI))
            print(f"{d:5d}  {de:+15.4f}  {do:+15.4f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="lidar_scan")
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--timeout", type=float, default=30.0, help="s to wait for the frames")
    ap.add_argument("--json", type=Path, help="write every beam's angles here")
    args = ap.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node("ms_geometry")
    col = Collector()
    node.create_subscription(PointCloud2, args.topic, lambda m: col.add(cloud_array(m)),
                             qos_profile_sensor_data)
    clock = node.get_clock()
    end = clock.now().nanoseconds + int(args.timeout * 1e9)
    while col.frames < args.frames and clock.now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if col.frames == 0:
        sys.exit(f"no clouds on '{args.topic}'")

    geo = col.layers()
    report(geo, col.frames)
    if args.json:
        args.json.write_text(json.dumps({
            "frames": col.frames,
            "layers": {str(l): {**g, "azimuth": {str(i): a for i, a in g["azimuth"].items()}}
                       for l, g in geo.items()},
        }, indent=1))
        print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
