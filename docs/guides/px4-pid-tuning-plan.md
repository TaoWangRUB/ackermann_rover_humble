# PX4 Ackermann Rover PID Tuning Plan

Based on [PX4 official Ackermann tuning guide](https://docs.px4.io/v1.16/en/config_rover/ackermann), adapted to the project's custom ROS 2 mode nodes.

## Prerequisites

### Infrastructure (per stage)

| Stage | Session script | Odometry | Nav2 |
|-------|---------------|----------|------|
| 1 — Physical limits | `start_px4_custom_mode_test_session.sh --mode-type manual` | T265 → VO bridge | No |
| 2 — Speed + Yaw rate | `start_px4_custom_mode_test_session.sh --mode-type speed_rate` | T265 → VO bridge | No |
| 3 — Yaw attitude | `start_px4_custom_mode_test_session.sh --mode-type speed_attitude` | T265 → VO bridge | No |
| 4 — Path following | `start_jetson_session.sh` (full stack) | T265/cuVSLAM → RTAB-Map | Yes |

**NOT needed for Stages 1–3:** D435i, RTAB-Map, Nav2.

### Tools

- [PX4 Flight Review](https://logs.px4.io/) — upload `.ulg` logs from SD card for plotting
- QGroundControl or `mavproxy` — change PX4 params live
- `pub_cmd_vel.sh <linear.x> <angular.z>` or `teleop_twist_keyboard` — send cmd_vel
- `scripts/px4_check_modes.py` — verify mode registration on PX4

### Auto-Tuning Scripts

Automated tuning scripts live in `scripts/tuning/`. They connect to PX4 via MAVLink
(auto-detecting `/dev/ttyACM*`), sweep parameters, command the rover via `cmd_vel`,
measure telemetry, and print results. Requires `pymavlink` on the host.

```bash
pip install pymavlink   # one-time setup
```

| Script | Stage | What it tunes |
|--------|-------|---------------|
| `tune_stage1_limits.py` | 1 | `RO_MAX_THR_SPEED`, `RO_ACCEL_LIM`, `RO_DECEL_LIM`, `RO_YAW_RATE_LIM` |
| `tune_stage2a_speed_ff.py` | 2A | `RO_MAX_THR_SPEED` (feed-forward only, P=0 I=0) |
| `tune_stage2b_speed_pid.py` | 2B | `RO_SPEED_P`, `RO_SPEED_I` |
| `tune_stage2c_yaw_rate_pid.py` | 2C | `RO_YAW_RATE_P`, `RO_YAW_RATE_I` |
| `tune_stage3_yaw_attitude.py` | 3 | `RO_YAW_P` |

Common flags:
- `--device /dev/ttyACM1` — override MAVLink device (default: auto-detect)
- `--apply` — write the best values to PX4 at the end (without this flag, results are only printed)
- `--speed <m/s>` — override test speed (Stages 2A–3)
- `Ctrl+C` — abort safely (stops rover, prints best values so far)

Shared library: `scripts/tuning/px4_tuning_lib.py` — MAVLink connection, param get/set,
telemetry collection, cmd_vel publishing via Docker exec, analysis helpers.

### Frame reference

```
map (global)
 └─ odom (local)        ← attitude/heading lives here (rotation local→body)
     └─ base_link (body) ← steering angle lives here (internal joint)
```

- **Heading** = chassis yaw in local frame, from EKF2 (IMU + magnetometer + VIO orientation). No GPS required.
- **Steering angle** = front wheel angle relative to chassis, an actuator output.

---

## Stage 1: Physical Limits — `RoverManual` mode

**Goal:** Identify the vehicle's physical characteristics. No PID tuning yet.
All parameters are set by physical measurement or observation from flight logs.

| Step | Action | Parameter | How to measure |
|------|--------|-----------|----------------|
| 1.1 | Measure wheelbase | `RA_WHEEL_BASE` [m] | Tape measure: rear axle to front axle |
| 1.2 | Measure max steering angle | `RA_MAX_STR_ANG` [deg] | Turn wheels full lock, measure angle |
| 1.3 | Find max speed | `RO_MAX_THR_SPEED` [m/s] | Full throttle straight line → read `measured_speed_body_x` from `RoverVelocityStatus` |
| 1.4 | Find max acceleration | `RO_ACCEL_LIM` [m/s²] | From standstill, full throttle → plot speed vs time → slope |
| 1.5 | Find max deceleration | `RO_DECEL_LIM` [m/s²] | From full speed, release throttle → plot speed decay |
| 1.6 | Find max steering rate | `RA_STR_RATE_LIM` [deg/s] | Full steering input, increase param until no longer limited |
| 1.7 | Find max yaw rate | `RO_YAW_RATE_LIM` [rad/s] | Full throttle + full steering → read `measured_yaw_rate` from `RoverRateStatus` |

**Log messages:** `RoverVelocityStatus.measured_speed_body_x`, `RoverRateStatus.measured_yaw_rate`

**Auto-tune (steps 1.3–1.7):**
```bash
python3 scripts/tuning/tune_stage1_limits.py              # dry run — print suggestions
python3 scripts/tuning/tune_stage1_limits.py --apply       # write to PX4
python3 scripts/tuning/tune_stage1_limits.py --skip-decel  # skip individual tests
```
Runs max-speed, acceleration, deceleration, and yaw-rate tests automatically.
Each test prompts for confirmation before driving. Use `--skip-speed`, `--skip-accel`,
`--skip-decel`, `--skip-yaw` to run only the tests you need.

---

## Stage 2: Speed + Yaw Rate — `RoverSpeedRate` mode

**Goal:** Tune the two inner-loop PIDs. Speed first (straight line), then yaw rate (turning).

`RoverSpeedRate` engages both the speed PID and the yaw rate PID simultaneously.
Tune speed feed-forward first (driving straight, angular.z = 0) so the yaw rate
loop isn't fighting an untuned speed loop.

### 2A: Speed Feed-Forward (straight line)

| Step | Action | Parameter |
|------|--------|-----------|
| 2A.1 | Set `RO_SPEED_P=0`, `RO_SPEED_I=0` | Feed-forward only |
| 2A.2 | Drive straight at various speeds (cmd_vel linear.x = 0.5, 1.0, 1.5 m/s) | — |
| 2A.3 | Plot `adjusted_speed_body_x_setpoint` vs `measured_speed_body_x` from `RoverVelocityStatus` | — |
| 2A.4 | If actual > setpoint → increase `RO_MAX_THR_SPEED`; if actual < setpoint → decrease | `RO_MAX_THR_SPEED` |
| 2A.5 | Repeat until feed-forward tracks within ~10% | — |

**Auto-tune:**
```bash
python3 scripts/tuning/tune_stage2a_speed_ff.py               # binary search on RO_MAX_THR_SPEED
python3 scripts/tuning/tune_stage2a_speed_ff.py --apply        # write result to PX4
python3 scripts/tuning/tune_stage2a_speed_ff.py --tolerance 0.05  # tighter 5% tolerance
```
Sets P=0, I=0 automatically, drives at 0.5/1.0/1.5 m/s, binary-searches `RO_MAX_THR_SPEED`
until feed-forward tracking is within tolerance (default 10%).

### 2B: Speed PID (refinement)

| Step | Action | Parameter |
|------|--------|-----------|
| 2B.1 | Add small proportional gain to improve step response | `RO_SPEED_P` (start at 0.1) |
| 2B.2 | If steady-state speed error under load, add integral gain | `RO_SPEED_I` (start at 0.01) |
| 2B.3 | Optionally cap maximum commanded speed | `RO_SPEED_LIM` |

**Auto-tune:**
```bash
python3 scripts/tuning/tune_stage2b_speed_pid.py               # sweep P then I
python3 scripts/tuning/tune_stage2b_speed_pid.py --apply        # write result to PX4
python3 scripts/tuning/tune_stage2b_speed_pid.py --speed 1.5    # test at 1.5 m/s instead of 1.0
```
Sweeps `RO_SPEED_P` [0, 0.05, 0.1, 0.2, 0.4, 0.8] with I=0. If steady-state error
remains >8%, sweeps `RO_SPEED_I`. Stops if overshoot exceeds 15%. Runs verification
with best gains at the end.

### 2C: Yaw Rate PID (turning)

| Step | Action | Parameter |
|------|--------|-----------|
| 2C.1 | Drive at moderate speed, command various angular.z (0.3, 0.5, 0.8 rad/s) | — |
| 2C.2 | Plot `adjusted_yaw_rate_setpoint` vs `measured_yaw_rate` from `RoverRateStatus` | — |
| 2C.3 | Increase until measured tracks setpoint without excessive overshoot | `RO_YAW_RATE_P` (start at 1.0) |
| 2C.4 | If steady-state error remains, add integral gain gradually | `RO_YAW_RATE_I` (start at 0.01) |

**Auto-tune:**
```bash
python3 scripts/tuning/tune_stage2c_yaw_rate_pid.py                # sweep P then I
python3 scripts/tuning/tune_stage2c_yaw_rate_pid.py --apply         # write result to PX4
python3 scripts/tuning/tune_stage2c_yaw_rate_pid.py --yaw-rate 0.3  # gentler test turns
python3 scripts/tuning/tune_stage2c_yaw_rate_pid.py --speed 0.8     # slower forward speed
```
Sweeps `RO_YAW_RATE_P` [0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0] with I=0. If steady-state
error remains >10%, sweeps `RO_YAW_RATE_I`. Stops if overshoot exceeds 20%. Verifies
in both CCW and CW directions. Requires ~3m open area for turning.

**Log messages:** `RoverVelocityStatus` (speed setpoint vs measured), `RoverRateStatus` (yaw rate setpoint vs measured)

---

## Stage 3: Yaw Attitude — `RoverSpeedAttitude` mode

**Goal:** Tune the outer yaw heading loop. It cascades into Stage 2's yaw rate PID.

Speed and yaw rate gains from Stage 2 must be tuned first.

| Step | Action | Parameter |
|------|--------|-----------|
| 3.1 | Drive forward, command heading changes via cmd_vel angular.z (mode integrates into heading setpoint) | — |
| 3.2 | Plot `adjusted_yaw_setpoint` vs `measured_yaw` from `RoverAttitudeStatus` | — |
| 3.3 | Increase until heading tracks well without oscillation | `RO_YAW_P` (start at 1.0) |
| 3.4 | Release angular.z → rover should hold heading with zero drift | — |

**Auto-tune:**
```bash
python3 scripts/tuning/tune_stage3_yaw_attitude.py               # sweep RO_YAW_P
python3 scripts/tuning/tune_stage3_yaw_attitude.py --apply         # write result to PX4
python3 scripts/tuning/tune_stage3_yaw_attitude.py --speed 0.5     # slower test speed
```
Sweeps `RO_YAW_P` [0.5, 1.0, 1.5, 2.0, 3.0, 5.0]. Each trial: angular.z pulse for
2s (heading step) then hold for 4s. Measures heading oscillation and drift during hold.
Stops if oscillation exceeds 8.6° (0.15 rad). Verifies best gain with left turn, right
turn, and straight-line heading hold.

**Log messages:** `RoverAttitudeStatus` (heading setpoint vs measured)

**Note:** `RoverSpeedAttitude` reads heading from `vehicle_local_position.heading`
(PX4 EKF2). This only requires local pose from VIO — no GPS/global position needed.

---

## Stage 4: Nav2 Path Following — `RoverSpeedRate` mode + Nav2

**Goal:** Tune pure pursuit path follower. Requires full stack (RTAB-Map + Nav2).

`RoverSpeedRate` is the recommended Nav2 mode because it provides closed-loop
yaw rate control via IMU gyro feedback. `RoverSpeedSteering` is open-loop on
steering and should only be used as a fallback.

| Step | Action | Parameter |
|------|--------|-----------|
| 4.1 | Send Nav2 a simple waypoint, observe path tracking | — |
| 4.2 | If rover oscillates around path → increase | `PP_LOOKAHD_GAIN` (start at 1.0) |
| 4.3 | If rover doesn't drive straight → decrease | `PP_LOOKAHD_GAIN` |
| 4.4 | If oscillates at low speed → increase | `PP_LOOKAHD_MIN` |
| 4.5 | If doesn't track at high speed → decrease | `PP_LOOKAHD_MAX` |

---

## Stage 5: Mission Tuning (optional)

For autonomous waypoint missions with smooth cornering.

| Parameter | Purpose |
|-----------|---------|
| `RO_DECEL_LIM` / `RO_JERK_LIM` | Smooth deceleration at waypoints |
| `NAV_ACC_RAD` | Default waypoint acceptance radius |
| `RA_ACC_RAD_MAX` / `RA_ACC_RAD_GAIN` | Corner cutting behavior |

---

## Parameter Quick Reference

All params verified against Cube Black firmware (PX4 main branch, 2026-05-19).

| Parameter | Stage | Loop | Current value | Starting value |
|-----------|-------|------|---------------|----------------|
| `RA_WHEEL_BASE` | 1 | Geometry | 0.174 m | Re-measure (seems too small) |
| `RA_MAX_STR_ANG` | 1 | Geometry | 0.60 rad (34°) | Measure |
| `RA_STR_RATE_LIM` | 1 | Limit | -1 (disabled) | Increase until not limited |
| `RO_MAX_THR_SPEED` | 1, 2A | Speed FF | 2.0 m/s | Observe max speed |
| `RO_ACCEL_LIM` | 1 | Limit | -1 (disabled) | Observe from log |
| `RO_DECEL_LIM` | 1, 5 | Limit | -1 (disabled) | Observe from log |
| `RO_YAW_RATE_LIM` | 1 | Limit | 90 rad/s (unlimited) | Observe max yaw rate |
| `RO_SPEED_P` | 2B | Speed PID | 0.0 | 0.1 |
| `RO_SPEED_I` | 2B | Speed PID | 0.0 | 0.01 |
| `RO_SPEED_LIM` | 2B | Limit | 2.0 m/s | — |
| `RO_YAW_ACCEL_LIM` | 2C | Limit | -1 (disabled) | Optional: limit yaw acceleration |
| `RO_YAW_RATE_P` | 2C | Yaw rate PID | 0.0 | 1.0 |
| `RO_YAW_RATE_I` | 2C | Yaw rate PID | 0.0 | 0.01 |
| `RO_YAW_RATE_CORR` | 2C | Yaw rate correction | 1.0 | Leave at 1.0 unless needed |
| `RO_YAW_P` | 3 | Yaw attitude | 0.0 | 1.0 |
| `PP_LOOKAHD_GAIN` | 4 | Pure pursuit | — | 1.0 |
| `PP_LOOKAHD_MIN` | 4 | Pure pursuit | — | — |
| `PP_LOOKAHD_MAX` | 4 | Pure pursuit | 10.0 | — |
| `RO_JERK_LIM` | 5 | Speed trajectory | — | — |
| `RO_SPEED_RED` | 5 | Speed reduction | -1 (disabled) | — |
| `NAV_ACC_RAD` | 5 | Waypoint acceptance | — | — |
| `RA_ACC_RAD_MAX` | 5 | Corner cutting | -1 (disabled) | — |
| `RA_ACC_RAD_GAIN` | 5 | Corner cutting | — | 1.0 |

### Params not used in tuning (informational)

| Parameter | Current value | Purpose |
|-----------|---------------|---------|
| `RO_YAW_RATE_TH` | 3.0 | Yaw rate threshold (below this, yaw rate control inactive) |
| `RO_SPEED_TH` | — | Speed threshold (below this, speed control inactive) |
| `RO_YAW_STICK_DZ` | 0.1 | RC stick deadzone (not relevant for cmd_vel modes) |

## Auto-Tune Workflow (complete example)

Run from the host machine with the PX4 test session already active.

```bash
# Stage 1: identify physical limits (RoverManual mode)
# Start session: ./scripts/start_px4_custom_mode_test_session.sh --mode-type manual
python3 scripts/tuning/tune_stage1_limits.py --apply

# Stage 2A: speed feed-forward (RoverSpeedRate mode)
# Start session: ./scripts/start_px4_custom_mode_test_session.sh --mode-type speed_rate
python3 scripts/tuning/tune_stage2a_speed_ff.py --apply

# Stage 2B: speed PID
python3 scripts/tuning/tune_stage2b_speed_pid.py --apply

# Stage 2C: yaw rate PID
python3 scripts/tuning/tune_stage2c_yaw_rate_pid.py --apply

# Stage 3: yaw attitude (RoverSpeedAttitude mode)
# Start session: ./scripts/start_px4_custom_mode_test_session.sh --mode-type speed_attitude
python3 scripts/tuning/tune_stage3_yaw_attitude.py --apply
```

Each script prompts before driving and can be aborted with Ctrl+C at any time.
Without `--apply`, results are printed but not written to PX4 — useful for dry runs.
Parameters are set in volatile RAM; use QGroundControl or `param save` in the MAVLink
shell to persist to flash.

---

## Mode Roles

| Mode | Setpoint | Speed | Steering feedback | Use |
|------|----------|-------|-------------------|-----|
| `RoverManual` | Throttle + normalized steering | Open-loop | None | Stage 1: identify limits |
| `RoverSpeedRate` | Speed [m/s] + yaw rate [rad/s] | Closed-loop | Closed-loop (IMU gyro) | **Stage 2 tuning + Nav2** |
| `RoverSpeedAttitude` | Speed [m/s] + heading [rad] | Closed-loop | Closed-loop (heading) | Stage 3 tuning + teleop |
| `RoverSpeedSteering` | Speed [m/s] + normalized [-1,1] | Closed-loop | None (open-loop) | Fallback only |
