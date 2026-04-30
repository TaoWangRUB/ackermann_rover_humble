#!/usr/bin/env python3
"""
For each XY-revisit pair, check whether the camera was pointing in the
SAME direction. RTAB-Map's RGBD pipeline needs viewpoint overlap, not
just position revisit.

Output: revisit pairs ranked by (heading-aligned + image-quality) so
we can see whether the bag actually offers a place that should match.
"""
import argparse
import math
from pathlib import Path
import numpy as np
import cv2
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("bag", nargs="?",
                default="/home/taowang/workspace/ackermann_rover_humble/bags/run_20260428_1340_seg3",
                help="bag directory")
ap.add_argument("--time-window", default=None,
                help="restrict analysis to t=[START,END] seconds from bag start "
                     "(e.g. --time-window=35,155 to focus on the walking phase)")
cli = ap.parse_args()

BAG = Path(cli.bag)
mcap_path = next(BAG.glob("*.mcap"))

window_s = None
if cli.time_window:
    a, b = cli.time_window.split(",")
    window_s = (float(a), float(b))

ODOM_TOPIC = "/t265/odom_base"
COLOR_TOPIC = "/d435i/color/image_raw"
R = 0.5     # m
T_MIN = 30  # s
YAW_TOL = math.radians(30)  # ±30° heading match

def yaw_from_quat(q):
    # ZYX yaw from (x,y,z,w)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# Read all odom + index color msgs
odom = []           # (t, x, y, yaw)
color_index = []    # (t, byte_offset placeholder; we re-read below)
print(f"Pass 1: indexing {mcap_path.name}"
      + (f" (window t={window_s[0]:.1f}..{window_s[1]:.1f}s)" if window_s else "") + "...")
bag_t0 = None
with mcap_path.open("rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[ODOM_TOPIC, COLOR_TOPIC]):
        t = message.log_time / 1e9
        if bag_t0 is None:
            bag_t0 = t
        t_rel = t - bag_t0
        if window_s and not (window_s[0] <= t_rel <= window_s[1]):
            continue
        if channel.topic == ODOM_TOPIC:
            p = ros_msg.pose.pose.position
            odom.append((t, p.x, p.y, yaw_from_quat(ros_msg.pose.pose.orientation)))
        else:
            color_index.append(t)
print(f"  {len(odom)} odom, {len(color_index)} color frames")

odom = np.array(odom)
t_off = odom[0,0]

# Find revisit pairs at same xy AND same heading
ds = odom[::int(len(odom)/600)+1]  # decimate to ~600 samples
pairs = []
for i in range(len(ds)):
    for j in range(i+1, len(ds)):
        if ds[j,0] - ds[i,0] < T_MIN: continue
        d_xy = math.hypot(ds[i,1]-ds[j,1], ds[i,2]-ds[j,2])
        if d_xy > R: continue
        d_yaw = abs(((ds[i,3]-ds[j,3]) + math.pi) % (2*math.pi) - math.pi)
        pairs.append((ds[i,0], ds[j,0], d_xy, d_yaw))

pairs.sort(key=lambda p: p[3])  # by heading delta
xy_only = [p for p in pairs if p[3] >  YAW_TOL]
xy_yaw  = [p for p in pairs if p[3] <= YAW_TOL]
print(f"\nRevisit pairs (within {R}m, gap >{T_MIN}s):")
print(f"  total                     : {len(pairs)}")
print(f"  same xy AND same heading  : {len(xy_yaw)}  (≤ {math.degrees(YAW_TOL):.0f}°)")
print(f"  same xy, different heading: {len(xy_only)}")
print()

if not xy_yaw:
    print("⚠ NONE of the position-revisits also align in heading.")
    print("  RTAB-Map's RGBD loop detector needs camera-direction overlap.")
    print("  Pure XY revisit with different heading = different scene → no closure.")
else:
    print(f"Best aligned revisit pairs (sorted by heading delta):")
    print(f"{'t1':>6} {'t2':>6} {'dxy':>6} {'dyaw_deg':>8}")
    for t1, t2, d_xy, d_yaw in xy_yaw[:8]:
        print(f"{t1-t_off:6.1f} {t2-t_off:6.1f} {d_xy:6.2f} {math.degrees(d_yaw):8.1f}")

# For the best aligned pair (or just the best XY pair if none aligned),
# pull the actual color frame at t1 and t2 and compute ORB-feature
# match count — that's exactly what RTAB-Map's geometric verification does.
test_pairs = xy_yaw[:3] if xy_yaw else pairs[:3]

if not test_pairs:
    raise SystemExit

print("\nImage-similarity check on top revisit pairs:")
# Cache: read every Nth color frame and store for nearest-neighbor lookup
print("  collecting color frames around revisit times...")
needed_times = set()
for t1, t2, *_ in test_pairs:
    needed_times.update([t1, t2])
imgs = {}
with mcap_path.open("rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[COLOR_TOPIC]):
        t = message.log_time / 1e9
        # Find any needed timestamp within 0.2s of this frame
        for nt in needed_times:
            if abs(t - nt) < 0.2 and nt not in imgs:
                h, w = ros_msg.height, ros_msg.width
                arr = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape(h, w, 3)
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if ros_msg.encoding.lower()=="rgb8" else arr
                imgs[nt] = bgr
        if len(imgs) >= len(needed_times):
            break

orb = cv2.ORB_create(nfeatures=2000)
bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

print(f"\n{'t1':>6} {'t2':>6} {'dxy':>5} {'dyaw_deg':>8} {'orb1':>5} {'orb2':>5} {'matches':>7} {'inliers':>7}  verdict")
for t1, t2, d_xy, d_yaw in test_pairs:
    if t1 not in imgs or t2 not in imgs:
        continue
    g1 = cv2.cvtColor(imgs[t1], cv2.COLOR_BGR2GRAY)
    g2 = cv2.cvtColor(imgs[t2], cv2.COLOR_BGR2GRAY)
    kp1, des1 = orb.detectAndCompute(g1, None)
    kp2, des2 = orb.detectAndCompute(g2, None)
    if des1 is None or des2 is None:
        m_count = 0; inliers = 0
    else:
        m = bf.match(des1, des2)
        m.sort(key=lambda x: x.distance)
        m_count = len(m)
        # RANSAC homography to count geometric inliers (RTAB-Map's check is similar)
        if m_count > 8:
            src = np.float32([kp1[mm.queryIdx].pt for mm in m]).reshape(-1,1,2)
            dst = np.float32([kp2[mm.trainIdx].pt for mm in m]).reshape(-1,1,2)
            H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
            inliers = int(mask.sum()) if mask is not None else 0
        else:
            inliers = 0
    verdict = "✅ MATCH" if inliers >= 12 else "❌ no match"
    print(f"{t1-t_off:6.1f} {t2-t_off:6.1f} {d_xy:5.2f} {math.degrees(d_yaw):8.1f} "
          f"{len(kp1):5d} {len(kp2):5d} {m_count:7d} {inliers:7d}  {verdict}")
    # Save side-by-side
    pair_id = f"t{t1-t_off:.0f}_vs_t{t2-t_off:.0f}"
    sbs = np.hstack([imgs[t1], imgs[t2]])
    cv2.putText(sbs, f"t1={t1-t_off:.1f}s  ORB={len(kp1)}", (10,25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    cv2.putText(sbs, f"t2={t2-t_off:.1f}s  ORB={len(kp2)}", (g1.shape[1]+10,25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    cv2.putText(sbs, f"matches={m_count}  inliers={inliers}  dyaw={math.degrees(d_yaw):.0f}deg",
                (10, sbs.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    out = Path("/tmp/loop_pairs"); out.mkdir(exist_ok=True)
    cv2.imwrite(str(out / f"pair_{pair_id}.jpg"), sbs)
    print(f"     → saved /tmp/loop_pairs/pair_{pair_id}.jpg")
