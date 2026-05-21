#!/usr/bin/env python3
"""Stage 2C: Yaw rate PID tuning (RO_YAW_RATE_P, RO_YAW_RATE_I).

After speed is tuned (Stages 2A/2B), this tunes the yaw rate inner loop.
Sends known yaw rate commands, measures actual yaw rate from IMU,
and sweeps P then I gains.

Algorithm:
  1. Drive at moderate speed with angular.z commands
  2. Sweep P: increase until overshoot or good tracking
  3. Sweep I: add if steady-state error remains
  4. Verify with step response

Prerequisites:
  - Stages 2A/2B completed (speed control tuned)
  - Docker container 'ackermann_slam' running with ROS2 workspace built
  - T265 + VO bridge active
  - Open area for turning (~3m radius)

Usage:
    python3 scripts/tuning/tune_stage2c_yaw_rate_pid.py [--device /dev/ttyACM0] [--apply]
    python3 scripts/tuning/tune_stage2c_yaw_rate_pid.py --skip-preflight
"""

import argparse
import math
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from px4_tuning_lib import (
    connect_mavlink, param_get, param_set, collect_telemetry,
    pub_cmd_vel, stop_cmd_vel, compute_stats,
    install_abort_handler, is_aborted, confirm, ensure_mode_and_arm,
)

P_VALUES = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0]
I_VALUES = [0.0, 0.005, 0.01, 0.02, 0.05, 0.1]
DRIVE_SPEED = 1.0         # m/s forward speed during yaw tests
TEST_YAW_RATE = 0.5       # rad/s (ROS convention: CCW+)
DRIVE_DURATION = 6.0
OVERSHOOT_LIMIT = 0.20    # 20% overshoot max
STEADY_ERROR_LIMIT = 0.10  # 10% steady-state error to trigger I


def run_yaw_rate_test(mav, speed: float, yaw_rate: float,
                      duration: float = DRIVE_DURATION):
    """Drive with speed + yaw rate, return (overshoot_frac, ss_error_frac).

    yaw_rate is in ROS convention (CCW+), the mode node negates for PX4.
    Measured yaw_rate from MAVLink ATTITUDE is NED CW+, so we compare |values|.
    """
    proc = pub_cmd_vel(speed, yaw_rate, duration + 1, rate_hz=10)
    time.sleep(0.3)
    samples = collect_telemetry(mav, duration)
    proc.wait()
    stop_cmd_vel()

    # PX4 yaw_rate is NED CW+, cmd_vel angular.z is ENU CCW+
    # The mode node negates, so PX4 target = |yaw_rate|
    target = abs(yaw_rate)
    rates = [abs(s.yaw_rate) for s in samples]

    if len(rates) < 5:
        return 0, 1.0

    # Overshoot: max in first 3 seconds
    early = [abs(s.yaw_rate) for s in samples if s.t < 3.0]
    max_early = max(early) if early else 0
    overshoot = (max_early - target) / target if target > 0 else 0

    # Steady-state: last 2 seconds
    late = [abs(s.yaw_rate) for s in samples if s.t > duration - 2.0]
    if late:
        mean_late = sum(late) / len(late)
        ss_error = abs(mean_late - target) / target if target > 0 else 0
    else:
        ss_error = 1.0

    return max(overshoot, 0), ss_error


def main():
    parser = argparse.ArgumentParser(description="Stage 2C: Yaw rate PID tuning")
    parser.add_argument("--device", default=None)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--yaw-rate", type=float, default=TEST_YAW_RATE,
                        help=f"Test yaw rate [rad/s] (default {TEST_YAW_RATE})")
    parser.add_argument("--speed", type=float, default=DRIVE_SPEED,
                        help=f"Forward speed during test (default {DRIVE_SPEED})")
    parser.add_argument("--mode-id", type=int, default=23,
                        help="PX4 external mode nav_state (default: 23)")
    parser.add_argument("--skip-preflight", action="store_true",
                        help="Skip mode registration and arming checks")
    args = parser.parse_args()

    mav = connect_mavlink(args.device)

    if not args.skip_preflight:
        if not ensure_mode_and_arm(mav, mode_id=args.mode_id):
            print("Pre-flight failed. Exiting.")
            mav.close()
            sys.exit(1)

    print(f"\nCurrent yaw rate params:")
    for p in ["RO_YAW_RATE_P", "RO_YAW_RATE_I", "RO_YAW_RATE_LIM"]:
        val = param_get(mav, p)
        print(f"  {p} = {val}")

    print("\n" + "=" * 60)
    print("STAGE 2C: Yaw Rate PID Tuning")
    print(f"Test: speed={args.speed} m/s, yaw_rate={args.yaw_rate} rad/s")
    print(f"P sweep: {P_VALUES}")
    print("=" * 60)
    if not confirm("Rover will drive in circles. Need ~3m open area. Ready?"):
        mav.close()
        return

    install_abort_handler()

    # --- P gain sweep ---
    print("\n--- P Gain Sweep (I=0) ---")
    param_set(mav, "RO_YAW_RATE_I", 0.0)
    best_p = 0.0
    best_p_error = 1.0

    for p in P_VALUES:
        if is_aborted():
            break

        param_set(mav, "RO_YAW_RATE_P", p)
        time.sleep(0.5)

        overshoot, ss_error = run_yaw_rate_test(mav, args.speed, args.yaw_rate)
        print(f"  P={p:.2f}  overshoot={overshoot * 100:+.1f}%  "
              f"ss_error={ss_error * 100:.1f}%")

        if overshoot > OVERSHOOT_LIMIT:
            print(f"  → Overshoot exceeded {OVERSHOOT_LIMIT * 100:.0f}% — stopping")
            break

        if ss_error < best_p_error:
            best_p = p
            best_p_error = ss_error

        time.sleep(1)

    print(f"\n  Best P={best_p:.2f} (ss_error={best_p_error * 100:.1f}%)")
    param_set(mav, "RO_YAW_RATE_P", best_p)

    # --- I gain sweep ---
    best_i = 0.0
    if best_p_error > STEADY_ERROR_LIMIT and not is_aborted():
        print(f"\n--- I Gain Sweep (P={best_p:.2f}) ---")

        best_i_error = best_p_error
        for i in I_VALUES[1:]:
            if is_aborted():
                break

            param_set(mav, "RO_YAW_RATE_I", i)
            time.sleep(0.5)

            overshoot, ss_error = run_yaw_rate_test(mav, args.speed, args.yaw_rate)
            print(f"  I={i:.4f}  overshoot={overshoot * 100:+.1f}%  "
                  f"ss_error={ss_error * 100:.1f}%")

            if overshoot > OVERSHOOT_LIMIT:
                print(f"  → Overshoot exceeded — stopping")
                break

            if ss_error < best_i_error:
                best_i = i
                best_i_error = ss_error

            if ss_error < STEADY_ERROR_LIMIT:
                best_i = i
                break

            time.sleep(1)
    else:
        print(f"\n  Tracking acceptable — skipping I sweep")

    stop_cmd_vel()

    # Verification
    if not is_aborted():
        print(f"\n--- Verification (P={best_p:.2f}, I={best_i:.4f}) ---")
        param_set(mav, "RO_YAW_RATE_P", best_p)
        param_set(mav, "RO_YAW_RATE_I", best_i)
        time.sleep(0.5)

        # Test both directions
        for direction, label in [(1.0, "CCW"), (-1.0, "CW")]:
            overshoot, ss_error = run_yaw_rate_test(
                mav, args.speed, args.yaw_rate * direction)
            print(f"  {label}: overshoot={overshoot * 100:+.1f}%  "
                  f"ss_error={ss_error * 100:.1f}%")
            time.sleep(1)

    print("\n" + "=" * 60)
    print("STAGE 2C RESULT")
    print(f"  RO_YAW_RATE_P = {best_p:.4f}")
    print(f"  RO_YAW_RATE_I = {best_i:.4f}")
    print("=" * 60)

    if args.apply:
        param_set(mav, "RO_YAW_RATE_P", best_p)
        param_set(mav, "RO_YAW_RATE_I", best_i)
        print("Applied.")
    else:
        print("Run with --apply to keep these values.")

    mav.close()


if __name__ == "__main__":
    main()
