#!/usr/bin/env python3
"""Stage 2B: Speed PID tuning (RO_SPEED_P, RO_SPEED_I).

After Stage 2A has set RO_MAX_THR_SPEED for good feed-forward tracking,
this script adds P and I gains to eliminate residual speed error.

Algorithm:
  1. Start with P=0, I=0 (feed-forward only baseline)
  2. Sweep P upward: measure step response at several speeds
     - Increase P until overshoot exceeds threshold or tracking is good
  3. If steady-state error remains, add small I
  4. Verify final gains with a speed step test

Prerequisites:
  - Stage 2A completed (RO_MAX_THR_SPEED tuned)
  - Docker container 'ackermann_slam' running with ROS2 workspace built
  - T265 + VO bridge active

Usage:
    python3 scripts/tuning/tune_stage2b_speed_pid.py [--device /dev/ttyACM0] [--apply]
    python3 scripts/tuning/tune_stage2b_speed_pid.py --skip-preflight
"""

import argparse
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from px4_tuning_lib import (
    connect_mavlink, param_get, param_set, collect_telemetry,
    pub_cmd_vel, stop_cmd_vel, compute_stats,
    install_abort_handler, is_aborted, confirm, ensure_mode_and_arm,
    shutdown_cmd_vel_publisher,
)

P_VALUES = [0.0, 0.05, 0.1, 0.2, 0.4, 0.8]
I_VALUES = [0.0, 0.005, 0.01, 0.02, 0.05]
TEST_SPEED = 1.0        # m/s
DRIVE_DURATION = 6.0     # seconds per test
OVERSHOOT_LIMIT = 0.15   # 15% overshoot max
STEADY_ERROR_LIMIT = 0.08  # 8% steady-state error to trigger I


def run_speed_test(mav, setpoint: float, duration: float = DRIVE_DURATION):
    """Drive at setpoint speed, return (samples, overshoot_frac, ss_error_frac)."""
    proc = pub_cmd_vel(setpoint, 0.0, duration + 1, rate_hz=10)
    time.sleep(0.3)
    samples = collect_telemetry(mav, duration)
    proc.wait()
    stop_cmd_vel()

    speeds = [s.speed for s in samples]
    if len(speeds) < 5:
        return samples, 0, 1.0

    # Overshoot: max speed in first 3 seconds
    early = [s.speed for s in samples if s.t < 3.0]
    max_early = max(early) if early else 0
    overshoot = (max_early - setpoint) / setpoint if setpoint > 0 else 0

    # Steady-state error: last 2 seconds
    late = [s.speed for s in samples if s.t > duration - 2.0]
    if late:
        mean_late = sum(late) / len(late)
        ss_error = abs(mean_late - setpoint) / setpoint if setpoint > 0 else 0
    else:
        ss_error = 1.0

    return samples, max(overshoot, 0), ss_error


def main():
    parser = argparse.ArgumentParser(description="Stage 2B: Speed PID tuning")
    parser.add_argument("--device", default=None)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--speed", type=float, default=TEST_SPEED,
                        help=f"Test speed setpoint (default {TEST_SPEED})")
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

    mts = param_get(mav, "RO_MAX_THR_SPEED")
    print(f"\nRO_MAX_THR_SPEED = {mts} (from Stage 2A)")

    print("\n" + "=" * 60)
    print("STAGE 2B: Speed PID Tuning")
    print(f"Test speed: {args.speed} m/s")
    print(f"P sweep: {P_VALUES}")
    print("=" * 60)
    if not confirm("Rover will drive straight repeatedly. Ready?"):
        mav.close()
        return

    install_abort_handler()

    # --- P gain sweep ---
    print("\n--- P Gain Sweep (I=0) ---")
    param_set(mav, "RO_SPEED_I", 0.0)
    best_p = 0.0
    best_p_error = 1.0

    for p in P_VALUES:
        if is_aborted():
            break

        param_set(mav, "RO_SPEED_P", p)
        time.sleep(0.5)

        _, overshoot, ss_error = run_speed_test(mav, args.speed)
        print(f"  P={p:.3f}  overshoot={overshoot * 100:+.1f}%  "
              f"ss_error={ss_error * 100:.1f}%")

        if overshoot > OVERSHOOT_LIMIT:
            print(f"  → Overshoot exceeded {OVERSHOOT_LIMIT * 100:.0f}% — stopping P sweep")
            break

        if ss_error < best_p_error:
            best_p = p
            best_p_error = ss_error

        time.sleep(1)

    print(f"\n  Best P={best_p:.3f} (ss_error={best_p_error * 100:.1f}%)")
    param_set(mav, "RO_SPEED_P", best_p)

    # --- I gain sweep (only if steady-state error remains) ---
    best_i = 0.0
    if best_p_error > STEADY_ERROR_LIMIT and not is_aborted():
        print(f"\n--- I Gain Sweep (P={best_p:.3f}) ---")
        print(f"  Steady-state error {best_p_error * 100:.1f}% > "
              f"{STEADY_ERROR_LIMIT * 100:.0f}% — adding I")

        best_i_error = best_p_error
        for i in I_VALUES[1:]:  # skip 0
            if is_aborted():
                break

            param_set(mav, "RO_SPEED_I", i)
            time.sleep(0.5)

            _, overshoot, ss_error = run_speed_test(mav, args.speed)
            print(f"  I={i:.4f}  overshoot={overshoot * 100:+.1f}%  "
                  f"ss_error={ss_error * 100:.1f}%")

            if overshoot > OVERSHOOT_LIMIT:
                print(f"  → Overshoot exceeded — stopping I sweep")
                break

            if ss_error < best_i_error:
                best_i = i
                best_i_error = ss_error

            if ss_error < STEADY_ERROR_LIMIT:
                print(f"  → Tracking within tolerance")
                best_i = i
                break

            time.sleep(1)
    else:
        print(f"\n  Steady-state error acceptable — skipping I sweep")

    stop_cmd_vel()

    # Final verification
    if not is_aborted():
        print(f"\n--- Verification (P={best_p:.3f}, I={best_i:.4f}) ---")
        param_set(mav, "RO_SPEED_P", best_p)
        param_set(mav, "RO_SPEED_I", best_i)
        time.sleep(0.5)
        _, overshoot, ss_error = run_speed_test(mav, args.speed)
        print(f"  Final: overshoot={overshoot * 100:+.1f}%  "
              f"ss_error={ss_error * 100:.1f}%")

    # Summary
    print("\n" + "=" * 60)
    print("STAGE 2B RESULT")
    print(f"  RO_SPEED_P = {best_p:.4f}")
    print(f"  RO_SPEED_I = {best_i:.4f}")
    print("=" * 60)

    if args.apply:
        param_set(mav, "RO_SPEED_P", best_p)
        param_set(mav, "RO_SPEED_I", best_i)
        print("Applied.")
    else:
        # Restore original values
        if not args.apply:
            print("Run with --apply to keep these values.")

    shutdown_cmd_vel_publisher()
    mav.close()


if __name__ == "__main__":
    main()
