#!/usr/bin/env python3
"""Stage 3: Yaw attitude tuning (RO_YAW_P).

Uses RoverSpeedAttitude mode. The mode integrates cmd_vel angular.z into
a heading setpoint; PX4's attitude controller drives heading error through
the yaw rate PID (tuned in Stage 2C) to produce steering commands.

Algorithm:
  1. Drive forward, command a heading step (angular.z pulse)
  2. Measure heading tracking: setpoint vs actual
  3. Sweep RO_YAW_P until good tracking without oscillation
  4. Verify heading hold (angular.z=0 → rover holds heading)

Prerequisites:
  - Stages 2A/2B/2C completed (speed + yaw rate tuned)
  - RoverSpeedAttitude mode activated
  - T265 + VO bridge active

Usage:
    python3 scripts/tuning/tune_stage3_yaw_attitude.py [--device /dev/ttyACM0] [--apply]
"""

import argparse
import math
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from px4_tuning_lib import (
    connect_mavlink, param_get, param_set, collect_telemetry,
    pub_cmd_vel, stop_cmd_vel, compute_stats,
    install_abort_handler, is_aborted, confirm,
)

P_VALUES = [0.5, 1.0, 1.5, 2.0, 3.0, 5.0]
DRIVE_SPEED = 0.8          # m/s
HEADING_STEP = 0.5         # rad/s angular.z pulse for 2 seconds → ~1 rad heading change
STEP_DURATION = 2.0        # seconds of angular.z command
HOLD_DURATION = 4.0        # seconds of heading hold (angular.z=0) after step
OSCILLATION_LIMIT = 0.15   # rad — max heading oscillation amplitude in hold phase


def wrap_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def run_heading_step_test(mav, speed: float, yaw_rate_cmd: float):
    """Command a heading step then hold. Return (heading_oscillation, drift_rate).

    1. Drive forward with angular.z for STEP_DURATION (heading ramp)
    2. Drive forward with angular.z=0 for HOLD_DURATION (heading hold)
    3. Measure oscillation and drift during hold phase
    """
    total = STEP_DURATION + HOLD_DURATION + 1

    # Phase 1: heading step
    proc1 = pub_cmd_vel(speed, yaw_rate_cmd, STEP_DURATION + 0.5, rate_hz=10)
    time.sleep(0.3)
    step_samples = collect_telemetry(mav, STEP_DURATION)
    proc1.wait()

    # Phase 2: heading hold
    proc2 = pub_cmd_vel(speed, 0.0, HOLD_DURATION + 0.5, rate_hz=10)
    time.sleep(0.3)
    hold_samples = collect_telemetry(mav, HOLD_DURATION)
    proc2.wait()
    stop_cmd_vel()

    # Analyze hold phase
    headings = [s.heading for s in hold_samples]
    if len(headings) < 5:
        return 999, 999

    # Oscillation: max deviation from mean heading
    mean_heading = sum(headings) / len(headings)
    deviations = [abs(wrap_pi(h - mean_heading)) for h in headings]
    max_oscillation = max(deviations) if deviations else 0

    # Drift: heading change over hold period
    if len(headings) >= 2:
        h_start = headings[0]
        h_end = headings[-1]
        dt = hold_samples[-1].t - hold_samples[0].t if hold_samples else 1
        drift_rate = abs(wrap_pi(h_end - h_start)) / max(dt, 0.1)
    else:
        drift_rate = 0

    return max_oscillation, drift_rate


def main():
    parser = argparse.ArgumentParser(description="Stage 3: Yaw attitude tuning")
    parser.add_argument("--device", default=None)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--speed", type=float, default=DRIVE_SPEED)
    args = parser.parse_args()

    mav = connect_mavlink(args.device)

    print(f"\nCurrent: RO_YAW_P = {param_get(mav, 'RO_YAW_P')}")
    print(f"  (depends on tuned RO_YAW_RATE_P = {param_get(mav, 'RO_YAW_RATE_P')})")

    print("\n" + "=" * 60)
    print("STAGE 3: Yaw Attitude (Heading) Tuning")
    print(f"Test: speed={args.speed} m/s, heading step={HEADING_STEP} rad/s × {STEP_DURATION}s")
    print(f"P sweep: {P_VALUES}")
    print("=" * 60)
    if not confirm("Rover will turn and hold heading. Need open area. Ready?"):
        mav.close()
        return

    install_abort_handler()
    best_p = P_VALUES[0]
    best_score = 999.0

    for p in P_VALUES:
        if is_aborted():
            break

        param_set(mav, "RO_YAW_P", p)
        time.sleep(0.5)

        oscillation, drift = run_heading_step_test(mav, args.speed, HEADING_STEP)
        # Score: lower is better (weighted combination)
        score = oscillation + drift * 2
        print(f"  P={p:.1f}  oscillation={math.degrees(oscillation):.1f}°  "
              f"drift={math.degrees(drift):.2f}°/s  score={score:.3f}")

        if oscillation > OSCILLATION_LIMIT:
            print(f"  → Oscillating ({math.degrees(oscillation):.1f}° > "
                  f"{math.degrees(OSCILLATION_LIMIT):.0f}°) — stopping")
            break

        if score < best_score:
            best_p = p
            best_score = score

        time.sleep(1)

    stop_cmd_vel()

    # Verification
    if not is_aborted():
        print(f"\n--- Verification (P={best_p:.1f}) ---")
        param_set(mav, "RO_YAW_P", best_p)
        time.sleep(0.5)

        for direction, label in [(HEADING_STEP, "Left turn"), (-HEADING_STEP, "Right turn")]:
            osc, drift = run_heading_step_test(mav, args.speed, direction)
            print(f"  {label}: oscillation={math.degrees(osc):.1f}°  "
                  f"drift={math.degrees(drift):.2f}°/s")
            time.sleep(1)

        # Straight line hold test
        print(f"  Straight hold:")
        proc = pub_cmd_vel(args.speed, 0.0, 5.0, rate_hz=10)
        time.sleep(0.3)
        samples = collect_telemetry(mav, 4.5)
        proc.wait()
        stop_cmd_vel()

        headings = [s.heading for s in samples if s.t > 1.0]
        if headings:
            mean_h = sum(headings) / len(headings)
            max_dev = max(abs(wrap_pi(h - mean_h)) for h in headings)
            print(f"    Heading deviation: ±{math.degrees(max_dev):.1f}°")

    print("\n" + "=" * 60)
    print("STAGE 3 RESULT")
    print(f"  RO_YAW_P = {best_p:.4f}")
    print("=" * 60)

    if args.apply:
        param_set(mav, "RO_YAW_P", best_p)
        print("Applied.")
    else:
        print("Run with --apply to keep this value.")

    mav.close()


if __name__ == "__main__":
    main()
