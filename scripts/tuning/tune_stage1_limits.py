#!/usr/bin/env python3
"""Stage 1: Identify physical limits using RoverManual mode.

Runs automated test maneuvers to measure:
  1. Max speed (RO_MAX_THR_SPEED)
  2. Max acceleration (RO_ACCEL_LIM)
  3. Max deceleration (RO_DECEL_LIM)
  4. Max yaw rate (RO_YAW_RATE_LIM)

The script automatically:
  - Checks if the RoverManual mode node is running, launches it if needed
  - Activates the mode on PX4
  - Arms the vehicle (with retry on failure)

Prerequisites:
  - Docker container 'ackermann_slam' running with ROS2 workspace built
  - T265 odometry + VO bridge active (PX4 EKF needs velocity)
  - Vehicle on flat ground with space to drive

Usage:
    python3 scripts/tuning/tune_stage1_limits.py [--device /dev/ttyACM0] [--apply]
    python3 scripts/tuning/tune_stage1_limits.py --skip-preflight  # manual mode/arm
"""

import argparse
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from px4_tuning_lib import (
    connect_mavlink, param_get, param_set, collect_telemetry,
    pub_cmd_vel, stop_cmd_vel, compute_stats, install_abort_handler,
    is_aborted, confirm, ensure_mode_and_arm,
)


def test_max_speed(mav, args):
    """Full throttle straight line to measure max speed."""
    print("\n" + "=" * 60)
    print("TEST 1: Max Speed (full throttle straight line)")
    print("  The rover will drive at full speed for 5 seconds.")
    print("  Ensure clear path ahead (>10m).")
    print("=" * 60)
    if not confirm("Ready?"):
        return None

    install_abort_handler()

    # Start telemetry collection in parallel with cmd_vel
    proc = pub_cmd_vel(2.0, 0.0, 6.0, rate_hz=10)
    time.sleep(0.5)  # let cmd_vel start
    samples = collect_telemetry(mav, 5.5)
    proc.wait()
    stop_cmd_vel()

    if is_aborted():
        stop_cmd_vel()
        return None

    # Analyze: take last 3 seconds as steady-state
    speeds = [s.speed for s in samples if s.t > 2.5]
    if not speeds:
        print("  ERROR: No telemetry received")
        return None

    mean, std, smin, smax = compute_stats(speeds)
    print(f"\n  Results ({len(speeds)} samples, last 3s):")
    print(f"    Mean speed:  {mean:.3f} m/s")
    print(f"    Max speed:   {smax:.3f} m/s")
    print(f"    Std dev:     {std:.3f} m/s")
    print(f"\n  → Suggested RO_MAX_THR_SPEED = {smax:.2f}")
    return smax


def test_acceleration(mav, args):
    """Full throttle from standstill to measure acceleration."""
    print("\n" + "=" * 60)
    print("TEST 2: Max Acceleration (standstill → full throttle)")
    print("  The rover will accelerate from standstill for 4 seconds.")
    print("=" * 60)
    if not confirm("Ready?"):
        return None

    install_abort_handler()

    proc = pub_cmd_vel(2.0, 0.0, 5.0, rate_hz=10)
    time.sleep(0.3)
    samples = collect_telemetry(mav, 4.5)
    proc.wait()
    stop_cmd_vel()

    if is_aborted():
        stop_cmd_vel()
        return None

    # Find time from ~0 to ~90% of max speed
    speeds = [(s.t, s.speed) for s in samples]
    if len(speeds) < 5:
        print("  ERROR: Not enough telemetry")
        return None

    max_speed = max(s for _, s in speeds)
    threshold = 0.9 * max_speed

    t_start = None
    t_end = None
    for t, s in speeds:
        if t_start is None and s > 0.1:
            t_start = t
        if t_start and s >= threshold:
            t_end = t
            break

    if t_start is None or t_end is None or t_end <= t_start:
        print(f"  Could not determine acceleration (max_speed={max_speed:.2f})")
        return None

    accel = threshold / (t_end - t_start)
    print(f"\n  Results:")
    print(f"    Time to {threshold:.2f} m/s: {t_end - t_start:.2f} s")
    print(f"    Max speed reached:     {max_speed:.2f} m/s")
    print(f"\n  → Suggested RO_ACCEL_LIM = {accel:.2f} m/s²")
    return accel


def test_deceleration(mav, args):
    """Release throttle from speed to measure deceleration."""
    print("\n" + "=" * 60)
    print("TEST 3: Deceleration (full speed → coast)")
    print("  The rover will drive, then coast to a stop.")
    print("=" * 60)
    if not confirm("Ready?"):
        return None

    install_abort_handler()

    # Drive up to speed
    proc = pub_cmd_vel(2.0, 0.0, 4.0, rate_hz=10)
    proc.wait()

    if is_aborted():
        stop_cmd_vel()
        return None

    # Now stop commanding and record deceleration
    stop_cmd_vel()
    samples = collect_telemetry(mav, 5.0)

    speeds = [(s.t, s.speed) for s in samples]
    if len(speeds) < 3:
        print("  ERROR: Not enough telemetry")
        return None

    # Find initial speed and time to reach ~10% of it
    initial_speed = speeds[0][1] if speeds else 0
    if initial_speed < 0.3:
        print(f"  Rover not moving fast enough (speed={initial_speed:.2f})")
        return None

    threshold = 0.1 * initial_speed
    t_start = speeds[0][0]
    t_end = None
    for t, s in speeds:
        if s <= threshold:
            t_end = t
            break

    if t_end is None:
        # Didn't fully stop — use last sample
        t_end = speeds[-1][0]
        final_speed = speeds[-1][1]
        decel = (initial_speed - final_speed) / (t_end - t_start)
    else:
        decel = initial_speed / (t_end - t_start)

    print(f"\n  Results:")
    print(f"    Initial speed: {initial_speed:.2f} m/s")
    print(f"    Coast time:    {t_end - t_start:.2f} s")
    print(f"\n  → Suggested RO_DECEL_LIM = {decel:.2f} m/s²")
    return decel


def test_max_yaw_rate(mav, args):
    """Full throttle + full steering to measure max yaw rate."""
    print("\n" + "=" * 60)
    print("TEST 4: Max Yaw Rate (full throttle + full steering)")
    print("  The rover will drive in a tight circle for 5 seconds.")
    print("  Ensure open area (~3m radius).")
    print("=" * 60)
    if not confirm("Ready?"):
        return None

    install_abort_handler()

    proc = pub_cmd_vel(1.5, 1.0, 6.0, rate_hz=10)
    time.sleep(0.5)
    samples = collect_telemetry(mav, 5.5)
    proc.wait()
    stop_cmd_vel()

    if is_aborted():
        stop_cmd_vel()
        return None

    # Analyze yaw rates from steady-state
    yaw_rates = [abs(s.yaw_rate) for s in samples if s.t > 2.0]
    if not yaw_rates:
        print("  ERROR: No telemetry received")
        return None

    mean, std, rmin, rmax = compute_stats(yaw_rates)
    print(f"\n  Results ({len(yaw_rates)} samples, last 3s):")
    print(f"    Mean |yaw_rate|: {mean:.3f} rad/s")
    print(f"    Max |yaw_rate|:  {rmax:.3f} rad/s")
    print(f"\n  → Suggested RO_YAW_RATE_LIM = {rmax:.2f} rad/s")
    return rmax


def main():
    parser = argparse.ArgumentParser(description="Stage 1: Identify physical limits")
    parser.add_argument("--device", default=None, help="MAVLink device (auto-detect)")
    parser.add_argument("--apply", action="store_true",
                        help="Auto-apply suggested values to PX4")
    parser.add_argument("--skip-speed", action="store_true")
    parser.add_argument("--skip-accel", action="store_true")
    parser.add_argument("--skip-decel", action="store_true")
    parser.add_argument("--skip-yaw", action="store_true")
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

    results = {}

    if not args.skip_speed:
        val = test_max_speed(mav, args)
        if val is not None:
            results["RO_MAX_THR_SPEED"] = val

    if not args.skip_accel:
        val = test_acceleration(mav, args)
        if val is not None:
            results["RO_ACCEL_LIM"] = val

    if not args.skip_decel:
        val = test_deceleration(mav, args)
        if val is not None:
            results["RO_DECEL_LIM"] = val

    if not args.skip_yaw:
        val = test_max_yaw_rate(mav, args)
        if val is not None:
            results["RO_YAW_RATE_LIM"] = val

    # Summary
    print("\n" + "=" * 60)
    print("STAGE 1 SUMMARY")
    print("=" * 60)
    if not results:
        print("  No tests completed.")
        mav.close()
        return

    for name, val in results.items():
        current = param_get(mav, name)
        current_str = f"{current:.4f}" if current is not None else "???"
        print(f"  {name:25s}  current={current_str:>10s}  → suggested={val:.4f}")

    if args.apply:
        print("\nApplying values...")
        for name, val in results.items():
            param_set(mav, name, val)
        print("Done. Values applied to PX4 (volatile — save params to persist).")
    elif results:
        print("\nRun with --apply to write these values to PX4.")

    mav.close()


if __name__ == "__main__":
    main()
