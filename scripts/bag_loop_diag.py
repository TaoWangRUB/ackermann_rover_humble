#!/usr/bin/env python3
"""
Diagnose 'no loop closures' from an RTAB-Map run by analysing the source bag.

Checks:
1. Image quality at sampled timestamps:
   - mean brightness                    (under/over-exposed?)
   - Laplacian variance                 (motion blur — low = blurry)
   - ORB feature count                  (texture / RTAB-Map can find features?)
2. Trajectory from /t265/odom_base or /cuvslam_odom:
   - Total path length, span (x,y range)
   - Number of revisits within R metres of an earlier waypoint
"""

import argparse
import sys
from pathlib import Path
from collections import defaultdict

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("bag", nargs="?",
                default="/home/taowang/workspace/ackermann_rover_humble/bags/run_20260428_1340_seg3",
                help="bag directory")
ap.add_argument("--time-window", default=None,
                help="restrict analysis to t=[START,END] seconds from bag start "
                     "(e.g. --time-window=35,155 to focus on the walking phase). "
                     "Default: whole bag.")
args = ap.parse_args()

BAG_DIR = Path(args.bag)
mcap_path = next(BAG_DIR.glob("*.mcap"))
print(f"Analysing {mcap_path.name} ({mcap_path.stat().st_size/1e9:.2f} GB)")

window_s = None
if args.time_window:
    a, b = args.time_window.split(",")
    window_s = (float(a), float(b))
    print(f"Time window: t={window_s[0]:.1f}..{window_s[1]:.1f} s (relative to bag start)")
print()

COLOR_TOPIC = "/d435i/color/image_raw"
ODOM_TOPICS = ["/cuvslam_odom", "/t265/odom_base", "/t265/odom"]

# --- Pass 1: gather messages ----------------------------------------------
img_msgs = []
odom_xy = defaultdict(list)
total_color = 0
bag_t0 = None  # absolute bag start time, set on first message
with mcap_path.open("rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    for schema, channel, message, ros_msg in reader.iter_decoded_messages():
        topic = channel.topic
        t_abs = message.log_time / 1e9
        if bag_t0 is None:
            bag_t0 = t_abs
        t_rel = t_abs - bag_t0
        # Apply time window filter
        if window_s and not (window_s[0] <= t_rel <= window_s[1]):
            continue
        if topic == COLOR_TOPIC:
            total_color += 1
            # Sample every Nth frame; we don't know N up-front so collect
            # references and decimate later.
            if total_color % 50 == 0:
                img_msgs.append((t_abs, ros_msg))
        elif topic in ODOM_TOPICS:
            p = ros_msg.pose.pose.position
            odom_xy[topic].append((t_abs, p.x, p.y))

print(f"Color frames: {total_color}, sampled {len(img_msgs)} for image analysis")
for k, v in odom_xy.items():
    print(f"Odom {k}: {len(v)} samples")
print()

# --- Pass 2: image-quality metrics on the samples -------------------------
print(f"{'time(s)':>9} {'bright':>8} {'lap_var':>9} {'orb':>5}  {'verdict'}")
orb = cv2.ORB_create(nfeatures=2000)

t0 = img_msgs[0][0]
quality = []
for t, msg in img_msgs:
    # ros_msg.data is bytes/array; encoding is rgb8 or bgr8 typically
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    if msg.encoding.lower() == "rgb8":
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    else:
        bgr = arr
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    bright = float(gray.mean())
    lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    kp = orb.detect(gray, None)
    n_kp = len(kp)

    flags = []
    if bright < 50:    flags.append("DARK")
    if bright > 200:   flags.append("OVEREXP")
    if lap_var < 60:   flags.append("BLUR")
    if n_kp < 200:     flags.append("LOW_TEX")
    verdict = ",".join(flags) if flags else "ok"

    print(f"{t-t0:9.1f} {bright:8.1f} {lap_var:9.1f} {n_kp:5d}  {verdict}")
    quality.append((t-t0, bright, lap_var, n_kp, flags))

# --- Aggregate ------------------------------------------------------------
qa = np.array([(b, l, k) for _, b, l, k, _ in quality])
print()
print(f"Aggregate ({len(quality)} sampled frames):")
print(f"  brightness  mean {qa[:,0].mean():.1f}  p10 {np.percentile(qa[:,0],10):.1f}  p90 {np.percentile(qa[:,0],90):.1f}")
print(f"  blur (lap)  mean {qa[:,1].mean():.1f}  p10 {np.percentile(qa[:,1],10):.1f}  median {np.median(qa[:,1]):.1f}")
print(f"  ORB feats   mean {qa[:,2].mean():.0f}  p10 {np.percentile(qa[:,2],10):.0f}  median {np.median(qa[:,2]):.0f}")
n_blur = sum(1 for *_ , f in quality if "BLUR" in f)
n_dark = sum(1 for *_ , f in quality if "DARK" in f)
n_overexp = sum(1 for *_ , f in quality if "OVEREXP" in f)
n_lowtex = sum(1 for *_ , f in quality if "LOW_TEX" in f)
print(f"  flagged:  blur {n_blur}  dark {n_dark}  overexp {n_overexp}  low-texture {n_lowtex}  out of {len(quality)}")

# --- Trajectory: revisits ------------------------------------------------
print()
xy_topic = next((t for t in ODOM_TOPICS if odom_xy[t]), None)
if xy_topic is None:
    print("No odom found — skipping revisit analysis")
    sys.exit(0)
xs = np.array([(t,x,y) for (t,x,y) in odom_xy[xy_topic]])
print(f"Trajectory from {xy_topic}: {len(xs)} samples, {xs[-1,0]-xs[0,0]:.1f} s")

# Path length
dxy = np.diff(xs[:,1:3], axis=0)
path_len = np.sum(np.linalg.norm(dxy, axis=1))
print(f"  path length: {path_len:.2f} m")
print(f"  bbox: x [{xs[:,1].min():.2f}, {xs[:,1].max():.2f}]  y [{xs[:,2].min():.2f}, {xs[:,2].max():.2f}]")

# Revisits: pairs of timestamps > 30s apart that are within 0.5 m of each other
R = 0.5      # m, "same place"
T_MIN = 30   # s, minimum gap to call it a "revisit"
# Decimate to ~1 Hz for the O(N^2) check
idx = np.linspace(0, len(xs)-1, num=min(len(xs), 600), dtype=int)
ds = xs[idx]
n_revisits = 0
seen_keys = set()
for i in range(len(ds)):
    for j in range(i+1, len(ds)):
        if ds[j,0] - ds[i,0] < T_MIN: continue
        if (ds[i,1]-ds[j,1])**2 + (ds[i,2]-ds[j,2])**2 < R*R:
            # Count once per ~5-second window of the later sample
            key = int(ds[j,0] / 5)
            if key not in seen_keys:
                seen_keys.add(key)
                n_revisits += 1
                if n_revisits <= 8:
                    print(f"    revisit: t={ds[i,0]-xs[0,0]:.1f}s and t={ds[j,0]-xs[0,0]:.1f}s, "
                          f"distance={np.hypot(ds[i,1]-ds[j,1], ds[i,2]-ds[j,2]):.2f} m")
if n_revisits == 0:
    print(f"  ⚠ NO revisits within {R}m, gap > {T_MIN}s — RTAB-Map has no opportunity to close loops!")
else:
    print(f"  ✓ {n_revisits} revisit windows found — RTAB-Map should have loop closure opportunities")
