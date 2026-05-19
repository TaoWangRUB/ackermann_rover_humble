#!/usr/bin/env python3
"""Stage 2A: Speed feed-forward tuning (RO_MAX_THR_SPEED).

Uses RoverSpeedRate mode with RO_SPEED_P=0, RO_SPEED_I=0 (feed-forward only).
Sends known speed setpoints, measures actual speed, adjusts RO_MAX_THR_SPEED
until feed-forward alone tracks within tolerance.

Algorithm:
  1. Set P=0, I=0 to isolate feed-forward
  2. For each test speed (0.5, 1.0, 1.5 m/s):
     - Send cmd_vel, record actual speed
  3. If actual > setpoint: increase RO_MAX_THR_SPEED (less throttle per m/s)
     If actual < setpoint: decrease RO_MAX_THR_SPEED (more throttle per m/s)
  4. Binary search until mean error < tolerance

Prerequisites:
  - PX4 test session running with RoverSpeedRate mode activated
  - T265 + VO bridge active

Usage:
    python3 scripts/tuning/tune_stage2a_speed_ff.py [--device /dev/ttyACM0] [--apply]
"""

import argparse
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from px4_tuning_lib import (
    connect_mavlink, param_get, param_set, collect_telemetry,
    pub_cmd_vel, stop_cmd_vel, steady_state_samples, compute_stats,
    install_abort_handler, is_aborted, confirm,
)

TOLERANCE = 0.10   # 10% acceptable error
MAX_ITERS = 8
TEST_SPEEDS = [0.5, 1.0, 1.5]  # m/s
DRIVE_DURATION = 5.0  # seconds per test speed


def measure_speed_tracking(mav, setpoint: float) -> float:
    """Send a speed setpoint, return mean measured speed."""
    proc = pub_cmd_vel(setpoint, 0.0, DRIVE_DURATION + 1, rate_hz=10)
    time.sleep(0.5)
    samples = collect_telemetry(mav, DRIVE_DURATION)
    proc.wait()
    stop_cmd_vel()

    ss = steady_state_samples(samples, skip_seconds=2.0)
    speeds = [s.speed for s in ss]
    if not speeds:
        return 0.0
    mean, _, _, _ = compute_stats(speeds)
    return mean


def main():
    parser = argparse.ArgumentParser(description="Stage 2A: Speed feed-forward tuning")
    parser.add_argument("--device", default=None)
    parser.add_argument("--apply", action="store_true",
                        help="Auto-apply final RO_MAX_THR_SPEED")
    parser.add_argument("--tolerance", type=float, default=TOLERANCE,
                        help=f"Acceptable tracking error fraction (default {TOLERANCE})")
    args = parser.parse_args()

    mav = connect_mavlink(args.device)

    # Ensure P and I are zero
    current_p = param_get(mav, "RO_SPEED_P")
    current_i = param_get(mav, "RO_SPEED_I")
    print(f"\nCurrent: RO_SPEED_P={current_p}, RO_SPEED_I={current_i}")
    if current_p and current_p > 0.001:
        print("Setting RO_SPEED_P=0 for feed-forward only tuning")
        param_set(mav, "RO_SPEED_P", 0.0)
    if current_i and current_i > 0.001:
        print("Setting RO_SPEED_I=0 for feed-forward only tuning")
        param_set(mav, "RO_SPEED_I", 0.0)

    current_mts = param_get(mav, "RO_MAX_THR_SPEED")
    print(f"Current RO_MAX_THR_SPEED = {current_mts}")

    print("\n" + "=" * 60)
    print("STAGE 2A: Speed Feed-Forward Tuning")
    print(f"Test speeds: {TEST_SPEEDS} m/s")
    print(f"Tolerance: {args.tolerance * 100:.0f}%")
    print("=" * 60)
    if not confirm("Rover will drive straight at multiple speeds. Ready?"):
        mav.close()
        return

    install_abort_handler()

    lo = 0.5
    hi = max(current_mts * 2 if current_mts else 4.0, 4.0)
    best_mts = current_mts or 2.0

    for iteration in range(MAX_ITERS):
        if is_aborted():
            stop_cmd_vel()
            break

        mid = (lo + hi) / 2
        param_set(mav, "RO_MAX_THR_SPEED", mid)
        time.sleep(0.5)

        print(f"\n--- Iteration {iteration + 1}/{MAX_ITERS} "
              f"(RO_MAX_THR_SPEED={mid:.3f}, range=[{lo:.2f}, {hi:.2f}]) ---")

        total_error = 0.0
        n_tests = 0
        for sp in TEST_SPEEDS:
            if is_aborted():
                break
            if sp >= mid:
                print(f"  Skip {sp} m/s (above current MTS={mid:.2f})")
                continue

            measured = measure_speed_tracking(mav, sp)
            error = (measured - sp) / sp if sp > 0 else 0
            print(f"  Setpoint={sp:.1f}  Measured={measured:.3f}  "
                  f"Error={error * 100:+.1f}%")
            total_error += error
            n_tests += 1
            time.sleep(1)

        if n_tests == 0:
            print("  No valid tests — adjusting range")
            hi = mid
            continue

        avg_error = total_error / n_tests
        print(f"  Average error: {avg_error * 100:+.1f}%")

        if abs(avg_error) < args.tolerance:
            print(f"  ✓ Within tolerance!")
            best_mts = mid
            break

        if avg_error > 0:
            # Actual > setpoint → too much throttle → increase MTS
            lo = mid
        else:
            # Actual < setpoint → too little throttle → decrease MTS
            hi = mid

        best_mts = mid

    stop_cmd_vel()

    # Summary
    print("\n" + "=" * 60)
    print("STAGE 2A RESULT")
    print(f"  RO_MAX_THR_SPEED = {best_mts:.3f}")
    print("=" * 60)

    if args.apply:
        param_set(mav, "RO_MAX_THR_SPEED", best_mts)
        print("Applied.")
    else:
        print("Run with --apply to write to PX4.")

    mav.close()


if __name__ == "__main__":
    main()
