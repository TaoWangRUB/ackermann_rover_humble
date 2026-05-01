---
title: "RTAB-Map Loop-Closure Tuning"
status: Reference
owner: taowang
last_updated: 2026-04-29
doc_type: architecture
ros_distro: jazzy
---

## Purpose

This document records the loop-closure tuning decisions for RTAB-Map on
the Ackermann rover stack — what the launch defaults are, why they
deviate from RTAB-Map's stock values, what failure modes you're likely
to hit, and how to diagnose them.

It's the place future-you reaches for when "RTAB-Map isn't closing
loops" again.

> **Read this first if loops aren't closing:** the structural fix is
> [subscribe_scan must be True](#structural-fix-subscribe_scan). Don't
> tune individual `Vis/*` knobs until you've verified the scan
> subscription. See also the [rtabmap_viz visualization guide](#rtabmap_viz-visualization-guide)
> for how to read the cyan/white/red/yellow/green markers.
>
> **Just want the commands?** See [operational quick reference](#operational-quick-reference)
> below.

## Operational quick reference

Standard invocations for the bag-replay and live-mapping paths, plus
diagnostic and live-tuning commands. All flags are documented in the
respective scripts (`-h` / `--help`).

### Live mapping

```bash
# Jetson Xavier deployment — Jetson preset auto-applied:
./scripts/start_jetson_session.sh --t265-odom

# Same, with telemetry/MQTT bridge to the control center:
./scripts/start_jetson_session.sh --t265-odom --with-telemetry

# Disable Jetson preset on Jetson (e.g. to A/B against full preset):
./scripts/start_jetson_session.sh --t265-odom --no-jetson-profile

# x86 desktop dev / SITL — full closure config:
./scripts/start_ros2_nodes.sh --hw --rtabmap --depth-camera=d435i --t265-odom

# x86 desktop with Jetson preset (verify Jetson behavior locally):
./scripts/start_ros2_nodes.sh --hw --rtabmap --depth-camera=d435i --t265-odom --jetson-profile

# Stop a running session:
./scripts/stop_all.sh --session=jetson      # or --session=ackermann_slam
```

### Bag replay (offline tuning)

```bash
# Default config — matches what live mapping does on x86 dev:
./scripts/start_rosbag_session.sh \
  --replay --name=run_20260429_1342_seg2 \
  --t265-odom --depth-camera=d435i \
  --no-attach

# Replay with the Jetson preset (verify performance for deployment):
./scripts/start_rosbag_session.sh \
  --replay --name=run_20260429_1342_seg2 \
  --t265-odom --depth-camera=d435i \
  --jetson-profile --no-attach

# Override an individual RTAB-Map knob without editing the launch:
./scripts/start_rosbag_session.sh \
  --replay --name=run_X --t265-odom \
  --rtabmap-param=Vis/MinInliers:=15 \
  --rtabmap-param=RGBD/LinearUpdate:=0.4

# Slower replay rate for debugging sync issues:
./scripts/start_rosbag_session.sh --replay --name=run_X --t265-odom --rate=0.5

# Stop the replay session:
./scripts/stop_all.sh --session=rosbag
```

### Diagnostic — verbose RTAB-Map logs

```bash
# Pass --udebug to the rtabmap binary so its internal UDEBUG logs print
# (matcher counts, RANSAC inliers, Bayesian hypothesis values, ...).
# See "Diagnostic technique: --rtabmap-udebug" below for what to grep for.
./scripts/start_rosbag_session.sh \
  --replay --name=run_X --t265-odom \
  --rtabmap-udebug --no-attach

# Tail the udebug output (the rtabmap pane echoes it; capture if needed):
tmux pipe-pane -t rosbag:rosbag.0 'cat >> /tmp/rtab_udebug.log'
```

### Inspecting an RTAB-Map session DB

```bash
# Quick summary of nodes and link types from the host (no Docker needed):
python3 -c "
import sqlite3
c = sqlite3.connect('.rtabmap/rover.db').cursor()
print('nodes:', c.execute('SELECT COUNT(*) FROM Node').fetchone()[0])
for r in c.execute('SELECT type, COUNT(*) FROM Link GROUP BY type ORDER BY type'):
    print(f'  type={r[0]:<2}  count={r[1]}')
"

# Full RTAB-Map summary (parameters used, neighbor/closure breakdowns, RMSE):
docker exec jazzy_slam_x86_64 bash -lc \
  'source /opt/ros/jazzy/setup.bash && rtabmap-info /workspace/.rtabmap/rover.db' \
  | sed 's/\x1b\[[0-9;]*m//g' \
  | grep -E 'Sessions|Total odom|^  Neighbor:|GlobalClosure|LocalSpace'

# Interactive viewer — click closure edges to see the matched RGB pair:
docker exec -it \
  -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY \
  jazzy_slam_x86_64 bash -lc \
  "rtabmap-databaseViewer /workspace/.rtabmap/rover.db"
```

### Verifying live RTAB-Map params

```bash
# Confirm Jetson preset is in effect (or not):
docker exec jazzy_slam_x86_64 bash -lc '
  source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
  for p in subscribe_scan Reg/Strategy Vis/MaxFeatures Kp/MaxFeatures \
           Vis/Iterations Vis/MinInliers Rtabmap/DetectionRate \
           RGBD/LinearUpdate RGBD/AngularUpdate Mem/ImagePostDecimation; do
    echo -n "$p="; ros2 param get /rtabmap "$p" 2>/dev/null | tail -1
  done'

# Watch loop closure activity live:
ros2 topic echo /info --field loop_closure_id --once   # 0 = no closure
ros2 topic echo /info --field proximity_detection_id --once
```

### Camera (RealSense) live tuning

The realsense_camera_bringup node now honors runtime parameter changes
on RGB exposure/gain (no restart needed):

```bash
# Manual mode with known-good values:
ros2 param set /d435i rgb_camera.enable_auto_exposure false
ros2 param set /d435i rgb_camera.exposure 80
ros2 param set /d435i rgb_camera.gain     32

# Or let the device auto-expose:
ros2 param set /d435i rgb_camera.enable_auto_exposure true

# Find good values for a new environment (camera STATIONARY, pointed at scene):
./scripts/tune_camera_exposure.sh --preset=normal --apply

# Watch live image-quality metrics while tuning:
docker exec -it jazzy_slam_x86_64 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  python3 /workspace/scripts/watch_image_quality.py /d435i/color/image_raw'
```

### Bag-side sanity check (does the bag *contain* loop opportunities?)

When RTAB-Map produces 0 closures and you want to verify it's the
algorithm, not the data:

```bash
# Image quality + revisit-distance survey on a bag:
python3 scripts/bag_loop_diag.py bags/run_20260429_1342_seg2

# Heading-aligned revisit pairs + offline ORB+RANSAC inlier check
# (answers: should this bag close loops at all, in any SLAM system?):
python3 scripts/bag_loop_pairs.py bags/run_20260429_1342_seg2
```

## TL;DR

| Param | Our default | RTAB-Map stock | When to override |
|---|---|---|---|
| `Vis/MaxDepth` | **`4.0` m** | `0` (no limit) | Outdoor / long-range scenes: set `0` |
| `Reg/Strategy` | **`2`** (Vis+ICP) | `0` (Vis only) | Pure-LiDAR scenarios: `1` |
| `RGBD/LinearUpdate` | **`0.2` m** | `0.1` m | Slow rover (<0.2 m/s mean speed): lower to `0.1`; high-speed: raise to `0.3-0.5` |
| `RGBD/AngularUpdate` | **`0.2` rad** | `0.1` rad | Filters small angular jitter (≈11.5° per node). Lower if rover yaw is naturally smooth. |
| `Vis/EstimationType` | `1` (PnP) | `1` | Pure-rotation handheld experiments: try `2` (epipolar 2D-2D) |
| `Vis/MinInliers` | **`6`** | `20` | RTAB-Map enforces a floor of 6; this is the most permissive legal value. Raise to 10-20 if false-positive closures appear. |
| `Vis/EpipolarGeometryVar` | `0.02` | `0.02` | Rotation-dominant motion: raise to `0.5` |
| `Kp/DetectorStrategy` | `6` (GFTT-BRIEF) | `6` | Try `2` (ORB) for more selective BoW matches |
| `Vis/MaxFeatures` | `1200` | `1000` | Texture-poor scenes: raise to `1500` |
| `Rtabmap/DetectionRate` | `1.0` Hz | `1` | Rover-mounted high-speed driving: raise to `2-5`; static testing: keep `1` |
| `Mem/RehearsalSimilarity` | `0.3` (set in [rtabmap_slam.launch.py](../../src/rtabmap_bringup/launch/rtabmap_slam.launch.py)) | `0.6` | Aggressive STM merging — current setting reduces near-duplicate retention |

**Five deliberate deviations from RTAB-Map's defaults**, all empirically
validated:

1. **`Vis/MaxDepth=4.0`** — clip far-distance D435i depth (>4 m is unreliable)
2. **`Reg/Strategy=2`** — Vis+ICP cascade gives 4–7 inliers on borderline candidates where Vis-only / ICP-only return 0
3. **`RGBD/LinearUpdate=0.2`** — guarantees ~20 cm baseline between consecutive nodes; the previous `0.05` m caused tiny inter-node baselines that made epipolar verification degenerate even for normal walking motion
4. **`RGBD/AngularUpdate=0.2`** — filter small angular jitter (≈11.5° threshold). Same logic as LinearUpdate, applied to rotation; protects against hand-tremor or noisy IMU integration creating spurious nodes
5. **`Vis/MinInliers=6`** — RTAB-Map's enforced floor; most permissive legal value. Raise to 10-20 if false-positive closures show up on a feature-rich scene

Everything else matches RTAB-Map stock and is exposed as launch
arguments + script flags for command-line tuning.

## Launch arguments (and how they flow)

Eight new launch arguments were added to
[`rtabmap_slam.launch.py`](../../src/rtabmap_bringup/launch/rtabmap_slam.launch.py):

```
rtabmap_vis_estimation_type     -> Vis/EstimationType
rtabmap_vis_max_depth           -> Vis/MaxDepth
rtabmap_vis_min_inliers         -> Vis/MinInliers
rtabmap_kp_detector_strategy    -> Kp/DetectorStrategy
rtabmap_vis_epipolar_var        -> Vis/EpipolarGeometryVar
rtabmap_reg_strategy            -> Reg/Strategy
rtabmap_linear_update           -> RGBD/LinearUpdate
rtabmap_angular_update          -> RGBD/AngularUpdate
```

Pass-through chain:

```
start_jetson_session.sh
    \-> robot_bringup.launch.py     (forwards via launch_arguments)
            \-> rtabmap_slam.launch.py    (declares + reads each)

start_rosbag_session.sh --replay
    \-> rtabmap_slam.launch.py    (declares + reads each)

start_ros2_nodes.sh
    \-> robot_bringup.launch.py     (forwards via launch_arguments)
            \-> rtabmap_slam.launch.py
```

Each entry-point script exposes the corresponding `--rtabmap-vis-*` and
`--rtabmap-kp-*` flag. Examples:

```bash
# Rover deployment (current defaults — recommended starting point)
./scripts/start_jetson_session.sh --t265-odom --with-telemetry

# Bag replay with permissive epipolar verification
./scripts/start_rosbag_session.sh --replay \
    --name=run_xxx --depth-camera=d435i --t265-odom \
    --rtabmap-vis-estimation-type=2 \
    --rtabmap-vis-epipolar-var=0.5

# Outdoor / long-range — disable the depth clip
./scripts/start_jetson_session.sh --t265-odom \
    --rtabmap-vis-max-depth=0
```

Important: RTAB-Map reads these parameters **at construction time**.
Setting them via `ros2 param set` after launch has no effect — you must
relaunch with new launch args.

## Why `Vis/MaxDepth=4.0`

D435i depth quality vs range:

| Range | Quality |
|---|---|
| 0.3 – 1.5 m | excellent (mm-cm accuracy) |
| 1.5 – 3 m   | good |
| 3 – 5 m     | noisy (~5% relative error) |
| 5 – 10 m    | unreliable (often `0` / NaN / wild values) |
| > 10 m      | effectively garbage |

RTAB-Map's PnP and 3D-3D ICP loop verification need **valid depth at
every keypoint pixel** to lift 2D features into 3D before solving. With
`MaxDepth=0` (no limit), unreliable far-distance depth contaminates the
solve and produces 0-inlier rejections even when the BoW match is
correct.

Clipping at 4 m forces RTAB-Map to use only well-conditioned keypoints.
Trade-off: fewer keypoints survive, especially in large rooms or
outdoor scenes — set `--rtabmap-vis-max-depth=0` for those.

## The diagnostic chain — when loop closures fail

Common symptoms and their meaning, in order of frequency we hit them:

### 1. `Not enough inliers 0/N (matches=M)` with `M >> 0`

> RTAB-Map found descriptor matches but PnP/3D-3D rejected all of them.

**Root cause:** D435i depth invalid at the matched keypoint pixels.
Typical scenes: white walls, glass, sunlit highlights, or features
beyond ~4 m.

**Fix path:**
1. `--rtabmap-vis-max-depth=4.0` (already the default)
2. Switch to `--rtabmap-vis-estimation-type=2` (epipolar 2D-2D) — does
   not need depth. Cost: needs translation between views (see #2).

### 2. `Variance is too high! (Max Vis/EpipolarGeometryVar=0.X, variance=Y)`

> Epipolar 2D-2D verification can't recover translation.

**Root cause:** the matched views have **near-zero translation between
them** — even normal walking can hit this if `RGBD/LinearUpdate` is too
small. With the old `0.05` m setting, RTAB-Map created a node every
5 cm; consecutive nodes had ~3 cm baseline at typical walking speed
(1 m/s × 1/30 s = 3 cm/frame). The Essential Matrix decomposition
becomes ill-conditioned when scene depth (3-5 m) divided by baseline
(0.03 m) gives a parallax of <1° per object — algorithm defaults to
"pure rotation" and rejects.

The fix is structural: **make the map nodes spread out spatially**.
Setting `RGBD/LinearUpdate=0.2` enforces ≥20 cm baseline between
neighbors, restoring valid parallax for the visual solver.

**Fix path:**
1. `--rtabmap-linear-update=0.2` (already the new default). Each node
   is 20 cm from the previous → 20 cm baseline guaranteed → epipolar
   variance finite.
2. `--rtabmap-vis-epipolar-var=0.5` — accept higher-variance estimates
   for handheld scenes that still have small per-step translation.
3. **Operator change**: walk continuously without standstill if you're
   running a handheld test; rover deployment rarely hits this regime.
4. Switch to `--rtabmap-vis-estimation-type=0` (3D-3D ICP) — robust to
   pure rotation if depth is OK on both sides. Often blocked by the
   same depth issue as #1 above.

### 3. `Not enough inliers N/8 (matches=M)` with `N` close to threshold

> Borderline match — passes some inliers but fails the strict count.

**Root cause:** scene has limited texture or repetitive features,
giving weak BoW matches.

**Fix path:**
1. Verify image quality (use the image-quality watcher script). Aim for
   brightness 100-150, ORB count > 500.
2. Try `--rtabmap-kp-detector-strategy=2` (ORB) for more discriminative
   descriptors.
3. Lower `--rtabmap-vis-min-inliers=6` (RTAB-Map's enforced minimum).

### 4. `Could not convert rgb/depth msgs! Aborting rtabmap update...`

> Frame transform missing; usually `<camera>_color_optical_frame` not
> available in TF.

**Root cause:** when replaying a bag with `ros2 bag play --start-offset
N` where `N>0`, the bag's `tf_static` (published once at `t=0`) is
**skipped**. RTAB-Map's TF listener never sees the static transforms.

**Fix path:**
1. Don't use `--start=N` on bag replay. Replay from `t=0`.
2. If you must skip a window, manually re-publish the bag's `tf_static`
   first (helper script: [`scripts/publish_bag_tf_static.py`](../../scripts/publish_bag_tf_static.py),
   requires `pip install mcap mcap-ros2-support` inside the container).
3. Or trim the bag to the desired window using `ros2 bag convert` so
   `tf_static` lives at `t=0` of the new bag.

### 5. No loop candidates proposed at all (`accepted=0 rej_inliers=0 rej_var=0`)

> BoW vocabulary doesn't find matching descriptors anywhere.

**Root cause:** image is too dim or too over-exposed for ORB / GFTT to
extract distinctive descriptors. Also happens when most of the bag is
stationary and rehearsal merging collapses near-duplicate views.

**Fix path:**
1. Fix exposure first — use the image-quality watcher and the
   exposure/gain sweep tooling under
   [`scripts/sweep_exp_gain.py`](../../scripts/sweep_exp_gain.py).
2. Verify ORB count > 500 in representative frames.
3. Check the bag's actual motion: many "revisits" at near-identical
   pose (handheld with stops) means no useful loop candidates exist.

## Reference data — empirical screening on `run_20260429_1039_handheld_seg1`

Eight RTAB-Map configurations were screened on a handheld bag with
known-good image quality (brightness 113, ORB 832 median) and 53
detected revisit windows. **All eight produced 0 accepted closures**
because the walk pattern was rotation-dominant and revisits clustered
at near-zero spatial baseline. This is the dataset that exposed every
failure mode in the diagnostic table above.

### Why hand-held bags are worst-case for RTAB-Map

The bag was recorded with the operator carrying the camera by hand.
Hand-held mounting amplifies every rotation-related failure mode:

| Mount | Typical yaw rate during normal walk | Notes |
|---|---|---|
| **Rover chassis** (Ackermann) | 0.0 – 0.5 rad/s | Steering rate × wheel speed; physically capped by `RA_MAX_STR_ANG / wheelbase × velocity` |
| **Head-mounted** (helmet) | 0.5 – 1.5 rad/s peaks | Natural head sway; rotation correlates with body translation |
| **Hand-held** (this bag) | **1 – 3 rad/s peaks** | Hand tremor + arm swing + step bounce + operator-driven panning |
| **Hand-held + active panning** | 3 – 5 rad/s peaks | Operator deliberately looking around |

Hand-held is materially worse than rover-mounted because:

1. **Rotation decouples from translation.** Operator can rotate the
   camera while standing still, or while the body walks — the camera's
   pose graph fills with revisits at the same xy with very different
   headings. BoW similarity suffers.
2. **Operator panning toward landmarks.** Visual attention drives the
   camera; revisits to the same place with the same heading (the only
   useful loop pairs) become rare.
3. **High-frequency hand tremor.** Adds 5-10 Hz angular noise that
   reduces inter-frame correspondence even during "walking straight."

For meaningful loop-closure validation: **mount on the rover, or use a
chest-strap / shoulder rig that constrains the camera to body rotation
only.** Pure hand-held verification is not predictive of rover-mounted
performance.

Full results, scripts, and frame samples:
[`artifacts/ai/2026-04-28_rtabmap_loop_closure_diag/`](../../artifacts/ai/2026-04-28_rtabmap_loop_closure_diag/).

The screening confirmed:
- `Vis/MaxDepth=4.0` materially redistributes failure modes (variance
  failures down 92 %, inlier failures up 11×) — proves the param flows
  through correctly even though the underlying motion problem dominated
  the verdict.
- `Reg/Strategy=2` (Vis+ICP cascade) produced **4–5 inliers per
  candidate** (clustered around the threshold of 6), where Strategy=1
  produced **0 inliers everywhere**. This was a strong signal that
  Strategy=2 should be the production default.
- The screening's "0 closures across all configs" outcome was traced
  to **`RGBD/LinearUpdate=0.05` m** creating tiny baselines (3 cm) between
  consecutive map nodes, making epipolar geometry degenerate even
  during normal walking. Bumping to `0.2` m is the structural fix.
- Param tuning **alone** can't rescue degenerate-motion bags; the
  combination of correct `RGBD/LinearUpdate` + `Reg/Strategy=2` +
  `Vis/MaxDepth=4` covers the realistic deployment cases.

## Recommended workflow when loop closures fail

1. **Verify image quality first** with the watcher
   ([`scripts/watch_image_quality.py`](../../scripts/watch_image_quality.py)).
   Brightness should be 100-150 and ORB count > 500. If not, fix
   exposure / gain / lighting before touching RTAB-Map.
2. **Verify motion data** — read `/t265/odom` and confirm there's
   actual translation between revisit times. Standstill periods or
   pure-rotation segments can't close loops via epipolar.
3. **Read the rejection messages** in the rtabmap pane (or via
   `tmux pipe-pane -O 'cat >> rtab.log'`). Match the pattern to the
   diagnostic table above before changing params.
4. **Use `start_rosbag_session.sh --replay` for tuning** — much faster
   than re-walking. Combine with the screen tooling under
   `/tmp/rtabmap_screen/` to compare configs side-by-side.
5. **Don't bake "permissive emergency" settings** (e.g., `MinInliers=6`
   + `EpipolarVar=0.5` + `EstimationType=2`) into the launch defaults.
   They're useful for diagnostic verification but increase
   false-positive risk in production.

## Related tooling

| Script | Purpose |
|---|---|
| [`scripts/watch_image_quality.py`](../../scripts/watch_image_quality.py) | Live brightness / blur / ORB count from a topic |
| [`scripts/sweep_exp_gain.py`](../../scripts/sweep_exp_gain.py) | Grid sweep of D435i `exposure` × `gain` with metric output |
| [`scripts/publish_bag_tf_static.py`](../../scripts/publish_bag_tf_static.py) | One-shot tf_static republisher for `--start-offset` bag replays |
| [`scripts/bag_loop_diag.py`](../../scripts/bag_loop_diag.py) | Bag-side image-quality + trajectory analysis (brightness, blur, ORB count, revisit detection) |
| [`scripts/bag_loop_pairs.py`](../../scripts/bag_loop_pairs.py) | Heading-aligned revisit pair detection + offline ORB+RANSAC inlier check (sanity-check whether a bag *should* close loops) |
| [`scripts/visualize_cloud_pose.sh`](../../scripts/visualize_cloud_pose.sh) | View `.ply` map cloud + pose `.txt` track in Open3D |
| [`scripts/tune_camera_exposure.sh`](../../scripts/tune_camera_exposure.sh) | Wrapper around `sweep_exp_gain.py` that brings up the camera (or uses a running one) and recommends `(exposure, gain)` for the current scene |

## Future work

- **`Vis/MaxDepth` per-camera-default**: D435i deserves `4.0`; L515
  could go higher (`8.0`+); a future LiDAR or depth-from-cuVSLAM source
  may want different. Consider exposing a `--depth-sensor=` profile.
- **Add `Mem/RehearsalSimilarity` as a launch arg**: currently hard-coded
  at `0.3` in `rtabmap_slam.launch.py`. Worth exposing for easy
  experimentation.
- **Tune `Rtabmap/LoopThr`**: not yet exposed; controls the BoW
  similarity threshold for proposing a candidate. Lower values increase
  candidate rate at the cost of false-positive risk.
- **Per-speed-regime `RGBD/LinearUpdate` profile**: 0.2 m is right for
  ~1 m/s indoor walking. A slow indoor rover (<0.2 m/s mean speed)
  would benefit from `0.1` m to keep map density up; a high-speed
  outdoor scenario should run `0.3-0.5` m to limit graph size.
- **Add 2D LiDAR (RPLiDAR A2 ~$300)** for redundant scan-matching loop
  closure independent of D435i depth quality. Considered but deferred
  pending rover-mounted RTAB-Map test.
- **Validate the new defaults end-to-end on a rover-mounted bag** —
  the small-baseline + Reg/Strategy + MaxDepth fixes should compound
  into actual closures once translation between revisits is naturally
  non-zero.

## RTAB-Map principles: mapping vs. localization

A short conceptual primer for what each mode actually does, since the rest of
this doc assumes it. If you've read the [RTAB-Map paper / wiki](https://github.com/introlab/rtabmap/wiki),
skim ahead.

### The persistent database (`.rtabmap/rover.db`)

A SQLite file. Nine tables matter:

| Table | What it stores |
|---|---|
| `Node` | one row per saved keyframe (id, pose, sensor data link). Node IDs grow monotonically. |
| `Link` | one row per graph edge with `type`: `0`=Neighbor (sequential odom edges), `1`=GlobalClosure (Bayesian recognition), `2`=LocalSpaceClosure (proximity-by-space), `5`=VirtualClosure, `9`=PosePrior. |
| `Word` | one row per visual word (BoW dictionary entry). |
| `Feature` | per-keypoint features attached to nodes. |
| `Data` | RGB + depth + scan blobs for each node. |
| `GlobalDescriptor` | per-node global image descriptors (NetVLAD-style if enabled). |
| `Statistics` | per-frame internal statistics — **grows in both modes**. |
| `Info` | per-session info (parameters used, DB version). |
| `Admin` | session-end pose + optimized graph cache. |

### Mapping mode (`Mem/IncrementalMemory=true`, the default)

Per-frame loop:

1. **Frame arrives** (RGB + depth + odom) →
2. **Feature extraction** at `Kp/MaxFeatures` keypoints →
3. **BoW quantization** — each descriptor mapped to a visual word; **new words can be added** to the dictionary →
4. **Decide if a new node is needed**: `RGBD/LinearUpdate` or `RGBD/AngularUpdate` exceeded since last node? Yes →
5. **Add Node + Neighbor link** to the graph →
6. **Bayesian recognition** — score similarity against past nodes; if `value > Rtabmap/LoopThr` (default 0.11), propose loop hypothesis →
7. **Geometric verification** — `RegistrationVis::computeTransformation` runs PnP/epipolar; if `inliers ≥ Vis/MinInliers` (floor 6), accept GlobalClosure →
8. **Proximity-by-space** — for any node within `RGBD/LocalRadius` of the new node, try registration; on success add LocalSpaceClosure →
9. **Graph optimization** (TORO / g2o / GTSAM) over all nodes + links →
10. **Updated map cloud + occupancy grid published**.

DB grows continuously — `Node`, `Link`, `Word`, `Feature`, `Data` all add rows.
At shutdown, `Admin.opt_last_localization` is written so the next session
knows where to start.

### Localization mode (`Mem/IncrementalMemory=false`)

Same per-frame loop, **with these modifications**:

- Step 3 (BoW): **dictionary frozen**. New descriptors are quantized against
  the existing words but no new words are added. `Kp/IncrementalDictionary` is
  effectively false.
- Step 4 (new node decision): **never adds new nodes**. The graph is fixed.
- Steps 6-7 (recognition + verification): unchanged — same algorithms,
  but now match the current frame against the loaded map nodes.
- Step 8 (proximity-by-space): also runs against map nodes only.
- **Extra: graph-error gate** (`RGBD/OptimizeMaxError`, default 3.0 σ). Even
  if step 7 succeeds, RTAB-Map runs a graph optimization with the proposed
  loop link added; if the resulting graph error ratio exceeds the gate, the
  match is rejected as likely-false-positive. This is more conservative than
  in mapping mode.
- **Confirmation pattern** (`RGBD/MaxOdomCacheSize`, default 10): a single
  successful localization is logged as `"Localization was good, but waiting
  for another one to be more accurate"` until `MaxOdomCacheSize` matches in
  a row agree on the pose. Then `map → odom` TF is published.
- **`Mem/InitWMWithAllNodes=true`**: at startup, all nodes loaded into
  Working Memory (vs. paged-on-demand). Faster relocalization at the cost
  of memory footprint.

### What gets written in localization mode

| Table | Mapping mode | Localization mode |
|---|---|---|
| `Node`, `Link`, `Word`, `Feature`, `Data` | grow | **unchanged** |
| `Statistics` | grows (per frame) | grows (per frame) |
| `Admin.opt_last_localization` | updated at shutdown | updated at each successful localization |

So the persistent map is read-only. The `Mem/LocalizationDataSaved=true`
flag adds Statistics blobs and a final pose snapshot — useful for offline
debugging — but doesn't add new graph nodes.

### When mapping and localization disagree

The same algorithm runs in both modes, but failure modes differ:

| Symptom | Mapping mode | Localization mode |
|---|---|---|
| BoW words not matching past nodes | low closure rate; map drifts | "value=0.0" hypothesis; no relocalization |
| Visual verifier returns 0 inliers from N matches | rejected closure (fewer corrections, more drift) | rejected localization candidate |
| Graph optimization error spike | only triggers `RGBD/MaxOptimizationError` global rejection | triggers per-loop graph-error gate (much stricter) |

For cross-condition robustness (different lighting/day), see
[Cross-condition robustness](#cross-condition-robustness) below.

## Cross-condition robustness

The hardest case in real deployment: built the map on a sunny morning,
localizing on a cloudy afternoon. Same physical room, slightly different
lighting → BoW words from the localization pass don't hash to the same
dictionary entries that mapping built. Symptom: `value=0.0` Bayesian
hypotheses, or 0/N inliers from BoW-matched candidates.

Empirically validated on `1209_seg1` map → `1340_seg3` localization:
51 Vis rejections with 0 inliers from 10–73 raw matches. Same physical area
(70% bbox overlap), 1 day apart, slightly different exposure.

### Lever 1 — Camera consistency (cheapest win)

Mapping and localization should use the **same camera tuning**. The 2026-04-29
defaults are `d435i_rgb_exposure=80, gain=32` — keep both runs at the same
values, OR enable auto-exposure on both:

```bash
# Mapping run — fix exposure to a known-good value:
ros2 param set /d435i rgb_camera.enable_auto_exposure false
ros2 param set /d435i rgb_camera.exposure 80
ros2 param set /d435i rgb_camera.gain     32

# Localization run — same values, OR switch to auto:
ros2 param set /d435i rgb_camera.enable_auto_exposure true
```

Auto-exposure usually wins because it adapts to ambient lighting changes that
manual settings can't compensate for. Cost: ~50 ms latency on exposure
changes (transient).

### Lever 2 — Multi-session mapping

Build the map across **multiple drive-throughs** in different lighting, all
into the same DB. The BoW dictionary then covers a richer set of word
variations.

```bash
# Session 1: morning
./scripts/start_jetson_session.sh --t265-odom
# (drive)

# Session 2: afternoon — extends the same DB (no --wipe-rtabmap-db):
./scripts/start_jetson_session.sh --t265-odom --keep-rtabmap-db
# (drive again)

# Session 3: cloudy day — same:
./scripts/start_jetson_session.sh --t265-odom --keep-rtabmap-db
```

Each session adds its own session-id range to the `Node` table, all under
the same map (since loop closures across sessions get found). Resulting
dictionary is robust to multi-day variation. **Highest practical-value lever**
once your environment is stable.

### Lever 3 — Switch the feature detector

Default is `Kp/DetectorStrategy=6` (GFTT + BRIEF). BRIEF descriptors are
binary and sensitive to gradient sign — that's why small lighting changes
cause word collisions to drift. Alternatives ranked by lighting invariance:

| Strategy | Detector / Descriptor | Pros / Cons |
|---|---|---|
| `2` | ORB / ORB | binary, FAST corners. Slightly more lighting-invariant than GFTT/BRIEF. CPU-friendly. |
| `0` | SURF / SURF | float descriptor, gradient-magnitude based. Much more lighting-robust but **patented + much slower**. |
| `1` | SIFT / SIFT | similar trade-off to SURF, free as of 2020. |
| `11` | SuperPoint | learned descriptor, very robust to lighting/seasonal change. **Needs the PyTorch model** + ideally GPU. Best quality-per-frame. |

Try first: `--rtabmap-kp-detector-strategy=2` (ORB). Two-line change, no new deps.

### Lever 4 — Pre-trained BoW vocabulary

Instead of growing the dictionary from your own footage, load a vocabulary
trained on a large diverse corpus (e.g. ORB-SLAM2's vocabulary). The
mapping run then quantizes its features against the same fixed dictionary
the localization run uses → much higher word-match rate.

```bash
--rtabmap-param=Kp/IncrementalDictionary:=false
--rtabmap-param=Kp/DictionaryPath:=/workspace/vocab/orb_voc.dbow3
```

Cost: shipping the vocabulary file (typically 30–100 MB) and ensuring
features come from the same descriptor type the vocabulary was trained on.

### Lever 5 — Loosen the localization-mode acceptance gates

When mapping is good but localization keeps rejecting candidates:

| Param | Default | Loosen to | Effect |
|---|---|---|---|
| `RGBD/OptimizeMaxError` | 3.0 σ | **5.0 σ** | accept candidates whose graph-error is up to 5× std-dev (saw 3.6–8.4 in our run, so this would accept the borderline ones) |
| `Rtabmap/LoopThr` | 0.11 | **0.05** | more permissive Bayesian recognition (we use this in non-default tuning) |
| `RGBD/MaxOdomCacheSize` | 10 | **3** | publish the `map→odom` TF after fewer confirming matches (faster relocalization, slightly more risk) |
| `RGBD/AggressiveLoopThr` | 0.05 | keep | already permissive for first-lock scenarios |

Try in order; raising `RGBD/OptimizeMaxError=5` is the single biggest
unlock in our cross-bag run (would have accepted the 11 graph-rejected
candidates).

### Lever 6 — Deep matchers (highest ceiling, highest cost)

Replace the descriptor-matching path with **SuperGlue** (`Vis/CorNNType=5`)
or **GMS** (`Vis/CorNNType=6 + pyMatcher`). These are *the* state-of-the-art
for cross-condition matching in 2026. RTAB-Map supports them via Python
bindings (`PyMatcher/Path`).

Cost: 50–200 ms per loop verification on Xavier (vs. ~10 ms for BFMatcher),
plus the model files (~50 MB).

Probably overkill for indoor rover deployment unless you're hitting a real
wall — try Levers 1–5 first.

### Recommended robustness recipe

For our rover (D435i + T265 + Jetson Xavier), in priority order:

1. **Auto-exposure on both mapping and localization** (Lever 1, free)
2. **Multi-session mapping** — record 3–5 drives across different times of
   day, all into the same DB (Lever 2, requires effort but transformative)
3. **`RGBD/OptimizeMaxError=5`** for localization (Lever 5, **measured: 8
   successful localizations vs 0 with the default 3.0 σ gate** on the
   2026-04-30 1209→1340 cross-bag test).

**Skip ORB (Lever 3) for cross-session localization** — measured outcome
contradicted the theoretical expectation. On the same cross-bag test,
ORB dropped the mean BoW match count from 62 (GFTT/BRIEF default) to 3, a
~20× reduction. Even combined with Lever 5 (`OptimizeMaxError=5`), ORB
still produced 0 successful localizations. Use ORB **only when CPU is the
binding constraint** — it's ~11× faster per frame (median 9 ms vs 104 ms)
but at a real cost in cross-session recall.

We don't recommend SuperPoint/SuperGlue (Lever 6) until / unless 1, 2, 5
prove insufficient for the actual deployment scenario. The Jetson preset's
constrained-CPU choices already strain compute; deep matchers would push
the verifier latency past real-time budget.

### Validation matrix (2026-04-30, base map = `run_20260427_1209_seg1`, target = `run_20260428_1340_seg3`)

| Variant | Detector | OptimizeMaxError | Successes | Mean matches | p95 ms |
|---|---|---|---|---|---|
| **A** default | GFTT/BRIEF | 3.0 σ | 0 | 62.6 | 141 |
| **B** Lever 5 | GFTT/BRIEF | **5.0 σ** | **8** | 46.7 | 248 |
| **C** Lever 3 | **ORB** | 3.0 σ | 0 | 3.2 | 35 |
| **D** Lever 3+5 | **ORB** | **5.0 σ** | 0 | 3.4 | 31 |

`Successes` = "Localization was good" events. `Mean matches` = pre-RANSAC
descriptor matches per loop candidate. The cross-condition robustness story
on this dataset is entirely about Lever 5; Lever 3 was a regression at the
matching stage (BoW vocabulary of one detector on day-1 didn't quantize
the same detector's day-2 features as densely as we'd hoped).

## Structural fix: `subscribe_scan`

**Updated 2026-04-29.** After 13+ parameter-tuning iterations on
`bags/run_20260429_1342_seg2` (rover) and
`bags/run_20260429_1039_handheld_seg1` (handheld) all yielding **0
closures**, the actual root cause turned out to be a single missing
subscription, not parameter values.

### The bug

[`rtabmap_slam.launch.py`](../../src/rtabmap_bringup/launch/rtabmap_slam.launch.py)
used to compute `subscribe_scan` as:

```python
subscribe_scan = True if vision == "false" else False
```

Logic was: if the platform has a real lidar (`vision=false`), feed its
scan to RTAB-Map; otherwise (`vision=true`, RGB-D pipeline) skip the
scan. **But the launch *also* runs `depthimage_to_laserscan` whenever
`vision=true`** — which publishes `/scan` from the depth image.
RTAB-Map ignored a scan that was right there.

With `Reg/Strategy=2` (Vis+ICP cascade — RTAB-Map's default), ICP
needs scan data on both sides of a candidate loop closure. With no
`subscribe_scan`, ICP either skipped the cascade silently or returned
a null transform that propagated up. The reject log line is
**literally empty after the colon**:

```
[WARN] Rtabmap.cpp:3069 Rejected loop closure 241 -> 402:
                                                          ^^ nothing here
```

That's the signature of this bug.

### The fix

```python
# rtabmap_slam.launch.py
subscribe_scan = True   # always — depthimage_to_laserscan or real lidar
scan_topic = '/scan'
```

That's it. With this single change, **upstream RTAB-Map defaults
produce 33 GlobalClosure + 37 LocalSpaceClosure** on the rover bag
above (60.7 m trajectory, 243 nodes). No `Vis/*` overrides needed.

### Verification matrix

| Run | `subscribe_scan` | `Reg/Strategy` | Other tuning | Closures |
|---|---|---|---|---|
| Pre-fix v1 | False | 2 (cascade) | upstream defaults | 0 |
| v9 (epipolar workaround) | False | 0 (Vis only) | 7 `Vis/*` overrides | 0 |
| v20 (full workaround) | False | 0 (Vis only) | 9 overrides + Force3DoF=false | 79 + 7 |
| **v23 (real fix + ICP)** | **True** | **2 (cascade)** | upstream defaults | **34 + 54 = 88** |
| v24 (real fix only) | True | 2 (cascade) | upstream defaults | 33 + 37 = 70 |

### Diagnostic technique: `--rtabmap-udebug`

[`scripts/start_rosbag_session.sh`](../../scripts/start_rosbag_session.sh)
takes a `--rtabmap-udebug` flag that passes `--udebug` to the rtabmap
binary, switching utilite's `ULogger` to debug level. The default
log only shows the final `Rtabmap.cpp:3069 Rejected loop closure ...`
line; with `--udebug` you also get:

| Source line | What it tells you |
|---|---|
| `RegistrationVis.cpp:332 Input(N): from=K words, M kpts, ...` | what the verifier received |
| `RegistrationVis.cpp:1477 VWDictionary knn matching` | which matcher branch fired |
| `RegistrationVis.cpp:1654 Not enough inliers N < 6` | final inlier count |
| `RegistrationVis.cpp:2200 inliers=N/M` | RANSAC raw inliers |
| `Rtabmap.cpp:2137 Highest hypothesis=K, value=V` | Bayesian filter score |

Reading these in combination tells you the exact pipeline stage that
killed a candidate. A few interpretations:

- `value=0.000000` consistently → BoW dictionary not matching past
  frames (texture-poor scene or bag too short for vocabulary to
  mature).
- `value=0.95` and rejection text **empty** → Vis succeeded but ICP
  cascade nulled the transform, i.e. you're hitting the
  `subscribe_scan` bug.
- `inliers=0/N` with high N (50+) → matches are spurious, often
  because the matcher took the `VWDictionary knn` path (L2 / FLANN-LSH
  approximate, no cross-check) instead of `cv::BFMatcher`. Triggered
  by certain `Vis/CorNNType` values; see source check below.

### Source-code reference (RTAB-Map 0.22.1 / Jazzy)

Worth knowing for next time. `Vis/MinInliers` is **clamped to 6 inside
the binary** at
[`RegistrationVis.cpp:265-268`](https://github.com/introlab/rtabmap/blob/0.22.1/corelib/src/RegistrationVis.cpp#L265-L268):

```cpp
if(_minInliers < 6) {
    UWARN("%s should be >= 6 but it is set to %d, setting to 6.", ...);
    _minInliers = 6;
}
```

Don't waste time setting it lower.

The matcher branch that uses `cv::BFMatcher` is gated by
`if (_nnType==5 || (_nnType==6 && _pyMatcher) || _nnType==7)` near
line 1700 — values 0–4 fall through to `VWDictionary::knn()` (FLANN-LSH
approximate). If you ever need cross-check matching for loop
verification, you need `Vis/CorNNType` 5/6/7.

The `0.22.1` and `0.22.1-jazzy` tags differ only in CI workflow and
`package.xml` (verified via `git diff` between SHAs); `RegistrationVis.cpp`
is identical, so the upstream `0.22.1` source is authoritative for the
Jazzy debian package.

## rtabmap_viz visualization guide

What the colors and lines mean in rtabmap_viz / RViz when you're
looking at the live SLAM output. This catches up new users without
needing to read RTAB-Map's display source.

### 3D point-cloud view

Three layers, often visible simultaneously:

| Marker | Topic | Color | What it represents |
|---|---|---|---|
| Polyline + dots along trajectory | `/mapPath` + `/mapGraph` | cyan | optimized SLAM pose graph (each dot = saved keyframe, each segment = graph edge after global optimization) |
| Smooth curve, **no** dots | direct subscription to odom topic (`/t265/odom_base`) | white | raw odometry trajectory **before** SLAM correction |
| Dense textured "fog" | `/cloud_map` | RGB from images (looks white/grey on pale walls) | the assembled 3D map — each node's depth points projected through its optimized pose |
| Scattered points / linear edges off the trajectory | `/camera/obstacles` | uniform cyan | output of `obstacles_detection` node — points 0.1–0.8 m above the ground plane, what Nav2's costmap will treat as obstacles |

The visual hierarchy is: **what we mapped** (white/grey RGB cloud) +
**what's an obstacle** (cyan obstacle cloud) + **where we went after
SLAM** (cyan trajectory) + **where T265 thought we went** (white line).

**Diagnostic value of the white-vs-cyan trajectory comparison:**

- Smooth white curve, gentle z-drift relative to cyan → normal T265
  drift; SLAM is correcting it. Healthy.
- White line with **discrete vertical spikes** (zigzag going up and
  down rapidly) → T265 momentarily lost tracking and re-jumped. Common
  causes: feature-poor scene (blank wall), fast rotation exceeding
  fisheye stereo baseline, lighting transition. SLAM's loop-closure
  graph optimization rejects the spike outliers — that's exactly the
  redundancy you want.
- Cyan diverging while white is smooth → bad sign. SLAM is fighting
  good odometry, usually due to a false-positive loop closure. Check
  `Vis/MinInliers` (raise it) and inspect specific edges in
  `rtabmap-databaseViewer`.

`Reg/Force3DoF=true` (our default for the rover) collapses all SLAM
pose corrections onto the x/y/yaw plane, which is why cyan stays
planar even where white shoots up vertically.

### 2D graph + occupancy-grid view

The "top-down map with colored polygon" view. Two layers:

**Background — `/map` (2D occupancy grid)** built from each node's
depth (`Grid/3D=false`, `Grid/RayTracing=true`,
`Grid/MaxObstacleHeight=0.8`):

- Light grey = free space (depth ray traversed without hitting)
- Black = obstacles (points 0.1–0.8 m above the ground plane)
- Mid grey = unknown (never observed)

**Overlay — `/mapGraph`** is a polygon where each vertex is a node
and each edge is colored by its `Link::Type` enum value:

| Color | `Link::Type` | What it means |
|---|---|---|
| **Blue** (forms the trajectory outline) | `kNeighbor` | sequential odometry edges (node N → N+1). Always the dominant count. |
| **Red** (cuts across the trajectory) | `kGlobalClosure` | loops detected by the **Bayesian appearance recognition** path — BoW words triggered, geometric verification accepted. Long-range. |
| **Yellow** (cuts across, often shorter) | `kLocalSpaceClosure` | loops detected by **`RGBD/ProximityBySpace`** — graph optimizer noticed two nodes are physically close (within `RGBD/LocalRadius`), tried registration, accepted. Short-range. |
| **Cyan** (rare) | `kLocalTimeClosure` | proximity by time — disabled here (`RGBD/ProximityByTime=false`). |
| **Green** (single edge, anchored) | `kPosePrior` or `kLandmark` | constraint anchoring a node to an absolute pose (initial-pose prior, GPS, marker). |
| Pink | `kUserClosure` | manually added closure (DB editor). |
| White | `kVirtualClosure` | post-hoc / virtual link. |

Counts in the graph match `rover.db`'s `Link` table 1:1 — querying
`SELECT type, COUNT(*) FROM Link GROUP BY type` gives the same
breakdown shown in `rtabmap-info`.

**Diagnostic patterns:**

- **Corridor cross-stitched with red+yellow** = healthy revisit
  response. Each pass through the corridor in either direction is
  being tied to the previous.
- **Two clusters tied together by a corridor of closures** = SLAM
  treats the environment as one consistent map, not two disconnected
  islands.
- **Red/yellow shooting off into the unknown grey region** = false
  positive. Closures should never extend into never-observed space.
- **No closures at all (only blue)** = either no revisits in the
  trajectory (check the bag) or the verifier is rejecting them all
  (re-read [structural fix: subscribe_scan](#structural-fix-subscribe_scan)
  first; only after that, parameters).

## Jetson-Xavier profile

For deployment on Jetson Xavier-class hardware where compute is the binding
constraint, the package ships a constrained-CPU profile in
[`config/jetson_profile.yaml`](../../src/rtabmap_bringup/config/jetson_profile.yaml).
Activated by the `jetson_profile:=true` launch arg, or via wrapper
script flag:

| Wrapper | Default | Flag |
|---|---|---|
| [`start_rosbag_session.sh`](../../scripts/start_rosbag_session.sh) (replay) | `false` | `--jetson-profile` / `--no-jetson-profile` |
| [`start_ros2_nodes.sh`](../../scripts/start_ros2_nodes.sh) (live mapping, x86) | `false` | `--jetson-profile` / `--no-jetson-profile` |
| [`start_jetson_session.sh`](../../scripts/start_jetson_session.sh) (live mapping, Jetson) | **`true`** (auto-on) | `--no-jetson-profile` to disable |

### What the profile changes

| Param | Default | Jetson | Reasoning |
|---|---|---|---|
| `Vis/MaxFeatures` | 1500 | **800** | half the per-frame descriptor pool → ~2× faster matching |
| `Kp/MaxFeatures` | 1500 | **800** | smaller BoW dictionary → faster Bayesian recognition |
| `Vis/Iterations` | 300 | **200** | fewer RANSAC samples — minor accuracy hit on borderline pairs |
| `Vis/MinInliers` | 6 (hard floor) | **12** | safer acceptance — fewer false-positive closures from ambiguous BoW matches |
| `Rtabmap/DetectionRate` | 0 (every frame) | **1.0 Hz** | only check loops once per sec; misses fast revisits but caps Bayesian filter cost |
| `RGBD/LinearUpdate` | 0.2 m | **0.3 m** | sparser graph → fewer nodes to match against |
| `RGBD/AngularUpdate` | 0.2 rad | **0.3 rad** | same logic for rotation |
| `Mem/ImagePostDecimation` | 1 | **2** | half-res stored images → 4× less storage + matching cost |

### Validated on x86 host (apples-to-apples replay comparison)

5 bags spanning day (brightness 86-192) and night (63-67), same hardware, only
`jetson_profile:=true` differs:

| Bag | Default closures | Jetson closures | Default p95 ms | Jetson p95 ms | Default max ms | Jetson max ms |
|---|---|---|---|---|---|---|
| `1538_seg1` (DAY 182) | 66 | **26** | 214 | **146** | 1515 | **404** |
| `1340_seg2` (DAY 192) | 145 | **34** | 236 | **148** | 3495 | **843** |
| `1342_seg2` (DAY 86) | 86 | **23** | 191 | **121** | 1590 | **403** |
| `2219_seg1` (NIGHT 63) | 68 | **20** | 225 | **139** | 1646 | **422** |
| `2219_seg5` (NIGHT 67) | 86 | **20** | 212 | **130** | 1588 | **436** |
| **avg** | **90** | **25 (28%)** | **216** | **137 (63%)** | **1967** | **502 (26%)** |

**Trade-off summary:** Jetson preset gives **~28% of the closures** but
**~63% of the latency** with **3-5× tighter worst-case spikes**. The big
win is the max-ms column — default config has 1.5-3.5 second spikes
when many BoW candidates land in one cycle, which on Xavier would miss
several /scan + /odometry/filtered cycles. Jetson preset caps worst case
under 850 ms.

Closure count drops are still comfortably double-digit on every bag —
sufficient to correct multi-meter T265 drift on a typical 60-100 m
mapping run.

### When to deviate from the preset

- **Long mapping runs (>5 min, >100 m)**: closures-per-meter is what matters
  for drift correction, not absolute count. The Jetson preset's ~25 closures
  on a 60 m bag = 0.4/m, which scales linearly with traversal — fine.
- **Aggressive driving (>1 m/s)**: bump `RGBD/LinearUpdate` further to 0.4 m
  and `Rtabmap/DetectionRate` to 0.5 Hz to keep node count proportional.
- **Suspected false-positive closures** (cyan trajectory snapping to wrong
  geometry): raise `Vis/MinInliers` further (to 15-20) — already at 12 in
  this profile, but feature-rich indoor scenes can fool BoW.
- **Re-mapping a known environment with a saved DB**: features re-extract
  from stored frames, so the `Mem/ImagePostDecimation=2` setting affects
  the *new* frames only. If working with bags recorded at full res, this
  won't visibly change closure quality.

### Database viewer for ground-truth inspection

`rtabmap-databaseViewer` lets you click any edge in the graph and see
the two RGB frames that were matched plus the inlier overlay. It's the
fastest way to confirm a loop closure is real (matches the same place)
vs. a BoW collision on similar-looking corners. Inside the container:

```bash
docker exec -it -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY \
  jazzy_slam_x86_64 bash -lc \
  "rtabmap-databaseViewer /workspace/.rtabmap/rover.db"
```

Useful click paths:
- *File → Edit optimized poses* — see the optimizer's pose corrections
- *Tools → Show all loop closure links* — visualizes red/yellow edges
  with the matched-frame side-by-side panel
- *Tools → Detect more loop closures* — re-runs detection with current
  parameters; safe to experiment, the DB is read-only by default
