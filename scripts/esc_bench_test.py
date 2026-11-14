#!/usr/bin/env python3
"""Wheels-up bench test for the rover's drive ESC.

Runs on the Jetson host (needs pymavlink) and talks MAVLink to the Cube Black:
directly on the USB serial port, or through mavlink_bridge.py (UDP 14551) when
the Jetson session runs with telemetry and owns the port.

Commands:
  status                 Read-only: armed state, MAIN 1/2 outputs, ESC parameters.
  apply --esc-unpowered  Write the bidirectional ESC profile, the values that
                         upload_params.sh --reversible-drive writes: CA_R_REV=1,
                         PWM_MAIN_MIN1=1100, PWM_MAIN_MAX1=1900, PWM_MAIN_DIS1=1500.
                         The disarmed throttle output changes at once, so the ESC
                         battery must be disconnected while this runs.
  hold PWM SECONDS       PX4 actuator test on the throttle output with the flight
                         controller disarmed: hold PWM (checked on SERVO_OUTPUT_RAW),
                         then release and check the output is back at the disarmed
                         value.

Built-in limits: apply and hold refuse to run while armed; hold accepts 1400-1600 us
only (--wide allows PWM_MAIN_MIN1..MAX1) for 1-30 s. The test is re-sent every 0.3 s
with a 1 s timeout, so if this script dies PX4 releases the output within a second.
hold stops at once if the output shows anything but the requested PWM.

PX4's actuator test does not follow CA_R_REV directly: it applies the reversible
flag of the last actuator_motors message (src/lib/mixer_module/actuator_test.cpp).
Until the rover controller has commanded the motor since boot there is none, and
the test maps forward-only (0..1 -> MIN..MAX). hold reads that state through the
PX4 shell (listener actuator_motors) and computes the test value to match.

Procedure (rover on blocks with all wheels off the ground, RC transmitter off,
someone holding the ESC battery connector the whole time):
  1. ESC battery disconnected:  esc_bench_test.py apply --esc-unpowered
  2. Connect the ESC battery. The wheels must not move: the output is 1500.
  3. esc_bench_test.py hold 1540 3   -> slow forward, then stop on release.
  4. Optional: esc_bench_test.py hold 1460 3, twice -> brake only, or reverse.
Disconnect the battery at once if the wheels move while the output is 1500.
"""

import argparse
import glob
import re
import struct
import subprocess
import time

from pymavlink import mavutil

# Must match upload_params.sh --reversible-drive and CA_R_REV in
# src/px4_bringup/config/cube_black_ackermann.params.
REVERSIBLE_PROFILE = [
    ("CA_R_REV", 1),
    ("PWM_MAIN_MIN1", 1100),
    ("PWM_MAIN_MAX1", 1900),
    ("PWM_MAIN_DIS1", 1500),
]
EXPECTED = dict(REVERSIBLE_PROFILE, PWM_MAIN_FUNC1=101, THR_MDL_FAC=0)
# THR_MDL_FAC bends the throttle-to-PWM curve; RO_ACCEL_LIM/RO_DECEL_LIM (> 0)
# slew the throttle that the rover controller sends.
SHOWN = ["CA_R_REV", "PWM_MAIN_FUNC1", "PWM_MAIN_MIN1", "PWM_MAIN_MAX1",
         "PWM_MAIN_DIS1", "PWM_MAIN_FAIL1", "THR_MDL_FAC", "RO_ACCEL_LIM",
         "RO_DECEL_LIM", "RO_MAX_THR_SPEED"]
SAFE_WINDOW = (1400, 1600)
MIN_HOLD_S = 1.0
MAX_HOLD_S = 30.0
# A test start ramps the output from the disarmed value to the target over 0.5 s
# (RAMP_TIME_US in src/lib/mixer_module/mixer_module.cpp); allow that plus latency.
RAMP_GRACE_S = 1.0

MAV_CMD_ACTUATOR_TEST = 310
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MSG_ID_SERVO_OUTPUT_RAW = 36
ACTUATOR_FUNCTION_MOTOR1 = 1


def default_device():
    """The PX4 USB port, or the mavlink_bridge.py UDP output when the bridge owns it."""
    bridge = subprocess.run(["pgrep", "-f", "mavlink_bridge.py"],
                            stdout=subprocess.DEVNULL).returncode == 0
    if bridge:
        return "udpin:127.0.0.1:14551"
    by_id = sorted(glob.glob("/dev/serial/by-id/*PX4*"))
    return by_id[0] if by_id else "/dev/ttyACM0"


def param_value(msg):
    """PX4 sends INT32 parameters bytewise in the float field."""
    if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
    return msg.param_value


class FlightController:
    def __init__(self, device, baud):
        self.device = device
        self.armed = None
        self.outputs = {}
        self.outputs_time = 0.0
        self.last_test_result = None
        self.m = mavutil.mavlink_connection(device, baud=baud, source_system=250)
        heartbeat = self.m.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise SystemExit("No heartbeat from the flight controller on %s" % device)
        self._handle(heartbeat)
        # PX4's USB MAVLink can ignore the first requests of a new connection;
        # a few GCS heartbeats wake it (as in scripts/tuning/px4_tuning_lib.py).
        for _ in range(5):
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.pump(0.2)
        # SERVO_OUTPUT_RAW at 20 Hz, so hold can catch a wrong output quickly.
        self.m.mav.command_long_send(1, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                     MSG_ID_SERVO_OUTPUT_RAW, 50000, 0, 0, 0, 0, 0)

    def _handle(self, msg):
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            if (msg.get_srcComponent() == 1
                    and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4):
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "SERVO_OUTPUT_RAW":
            self.outputs = {1: msg.servo1_raw, 2: msg.servo2_raw}
            self.outputs_time = time.time()
        elif kind == "STATUSTEXT":
            print("  FC: %s" % msg.text)
        elif kind == "COMMAND_ACK" and msg.command == MAV_CMD_ACTUATOR_TEST:
            if msg.result != 0 and msg.result != self.last_test_result:
                print("  Actuator test rejected (MAV_RESULT %d)" % msg.result)
            self.last_test_result = msg.result

    def pump(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=min(0.2, max(0.01, end - time.time())))
            if msg is not None:
                self._handle(msg)

    def output(self, channel=1, max_age=1.0, wait=3.0):
        """Latest PWM on a MAIN output, or None if SERVO_OUTPUT_RAW is not arriving."""
        start = time.time()
        while time.time() - start < wait:
            if self.outputs and time.time() - self.outputs_time <= max_age:
                return self.outputs.get(channel)
            self.pump(0.2)
        return None

    def shell(self, command, timeout=5.0):
        """Run a PX4 NSH command over the MAVLink shell (as scripts/px4_cmd.py does)."""
        data = list((command + "\n").encode())
        flags = (mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                 | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND)
        for device_id in (10, 0):
            for i in range(0, len(data), 70):
                chunk = data[i:i + 70]
                self.m.mav.serial_control_send(device_id, flags, 0, 0, len(chunk),
                                               chunk + [0] * (70 - len(chunk)))
            text = b""
            end = time.time() + timeout
            while time.time() < end:
                msg = self.m.recv_match(blocking=True, timeout=0.3)
                if msg is None:
                    continue
                if msg.get_type() != "SERIAL_CONTROL":
                    self._handle(msg)
                    continue
                if msg.count > 0:
                    text += bytes(msg.data[:msg.count])
                    end = time.time() + 1.0
                if b"nsh>" in text[-30:]:
                    break
            self.m.mav.serial_control_send(device_id, 0, 0, 0, 0, [0] * 70)
            if text:
                return text.decode("utf-8", errors="replace")
        return None

    def motor_test_mapping(self):
        """('reversible' | 'forward-only' | None, reason) for PX4's Motor 1 actuator test."""
        text = self.shell("listener actuator_motors -n 1")
        if text is None:
            return None, "no reply from the PX4 shell"
        if "never published" in text:
            return "forward-only", "motor not commanded since the flight controller started"
        found = re.search(r"reversible_flags:\s*(\d+)", text)
        if not found:
            return None, "unexpected listener output: %s" % " ".join(text.split())[-160:]
        flags = int(found.group(1))
        mapping = "reversible" if flags & 1 else "forward-only"
        return mapping, "last motor command had reversible_flags=%d" % flags

    def _param_reply(self, name, timeout):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=0.3)
            if msg is None:
                continue
            if msg.get_type() == "PARAM_VALUE" and msg.param_id.rstrip("\x00") == name:
                return param_value(msg)
            self._handle(msg)
        return None

    def param_get(self, name):
        for _ in range(5):
            self.m.mav.param_request_read_send(1, 1, name.encode(), -1)
            value = self._param_reply(name, 1.5)
            if value is not None:
                return value
        return None

    def param_set_int(self, name, value):
        raw = struct.unpack("<f", struct.pack("<i", value))[0]
        for _ in range(3):
            self.m.mav.param_set_send(1, 1, name.encode(), raw,
                                      mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            if self._param_reply(name, 1.5) == value:
                return True
        return self.param_get(name) == value

    def require_disarmed(self):
        end = time.time() + 3
        while self.armed is None and time.time() < end:
            self.pump(0.2)
        if self.armed is None:
            raise SystemExit("No arming state from the flight controller")
        if self.armed:
            raise SystemExit("The flight controller is ARMED: disarm it first")

    def actuator_test(self, value, timeout):
        self.m.mav.command_long_send(1, 1, MAV_CMD_ACTUATOR_TEST, 0, value, timeout,
                                     0, 0, ACTUATOR_FUNCTION_MOTOR1, 0, 0)


def cmd_status(fc):
    fc.pump(1.0)
    values = {name: fc.param_get(name) for name in SHOWN}
    state = {True: "ARMED", False: "disarmed", None: "unknown"}[fc.armed]
    print("Link: %s" % fc.device)
    print("Flight controller: %s" % state)
    print("Throttle output (MAIN 1): %s us, steering output (MAIN 2): %s us"
          % (fc.output(1), fc.output(2)))
    mapping, reason = fc.motor_test_mapping()
    print("Motor test mapping: %s (%s)" % (mapping or "unknown", reason))
    for name in SHOWN:
        value = values[name]
        want = EXPECTED.get(name)
        note = "" if want is None else ("ok" if value == want else "expected %d" % want)
        print("  %-15s %6s  %s" % (name, value, note))
    low, high, dis = values["PWM_MAIN_MIN1"], values["PWM_MAIN_MAX1"], values["PWM_MAIN_DIS1"]
    if None not in (low, high, dis) and dis < (low + high) / 2 - 20:
        print("WARNING: the disarmed output %d us is below the centre of %d-%d us. With a "
              "centre-stop ESC, disarm and E-STOP drive the wheels backward." % (dis, low, high))
    return 0


def cmd_apply(fc, esc_unpowered):
    if not esc_unpowered:
        raise SystemExit("Disconnect the ESC battery, then run again with --esc-unpowered: "
                         "the throttle output changes as soon as PWM_MAIN_DIS1 is written.")
    fc.require_disarmed()
    ok = True
    for name, value in REVERSIBLE_PROFILE:
        before = fc.param_get(name)
        if before == value:
            print("  %-14s %d (already set)" % (name, value))
            continue
        done = fc.param_set_int(name, value)
        ok = ok and done
        print("  %-14s %s -> %d  %s" % (name, before, value, "ok" if done else "FAILED"))
    fc.pump(1.0)
    out = fc.output(1)
    print("Throttle output while disarmed: %s us (expected 1500)" % out)
    if out != 1500:
        print("WARNING: the output is not 1500")
        ok = False
    return 0 if ok else 1


def cmd_hold(fc, pwm, seconds, wide):
    fc.require_disarmed()
    low = fc.param_get("PWM_MAIN_MIN1")
    high = fc.param_get("PWM_MAIN_MAX1")
    dis = fc.param_get("PWM_MAIN_DIS1")
    curve = fc.param_get("THR_MDL_FAC")
    if None in (low, high, dis, curve):
        raise SystemExit("Could not read the ESC parameters")
    if abs(curve) > 1e-6:
        raise SystemExit("THR_MDL_FAC is %g: the test value would not map linearly" % curve)
    window = (low, high) if wide else SAFE_WINDOW
    if not window[0] <= pwm <= window[1]:
        hint = "" if wide else " (--wide allows %d-%d us)" % (low, high)
        raise SystemExit("%d us is outside %d-%d us%s" % (pwm, window[0], window[1], hint))
    if not MIN_HOLD_S <= seconds <= MAX_HOLD_S:
        raise SystemExit("SECONDS must be between %g and %g" % (MIN_HOLD_S, MAX_HOLD_S))
    mapping, reason = fc.motor_test_mapping()
    if mapping is None:
        raise SystemExit("Cannot tell how PX4 maps the motor test (%s): not testing" % reason)
    before = fc.output(1)
    if before is None:
        raise SystemExit("No SERVO_OUTPUT_RAW from the flight controller: cannot check the output")

    if mapping == "reversible":
        value = 2.0 * (pwm - low) / (high - low) - 1.0   # -1..1 -> MIN..MAX
    else:
        value = (pwm - low) / float(high - low)          # 0..1 -> MIN..MAX
    print("Motor test mapping: %s (%s)." % (mapping, reason))
    print("Output %d us. Holding %d us (test value %+.3f) for %g s."
          % (before, pwm, value, seconds))
    ok = True
    confirmed = False
    start = time.time()
    last_send = 0.0
    try:
        while time.time() - start < seconds:
            if time.time() - last_send >= 0.3:
                fc.actuator_test(value, 1.0)
                last_send = time.time()
            fc.pump(0.05)
            if fc.armed:
                print("  The flight controller armed during the test: stopping")
                ok = False
                break
            if fc.outputs_time <= start:
                continue
            actual = fc.outputs.get(1)
            if actual is not None and abs(actual - pwm) <= 5:
                if not confirmed:
                    confirmed = True
                    print("  Output %d us after %.2f s" % (actual, time.time() - start))
            elif (actual is not None and not confirmed and time.time() - start < RAMP_GRACE_S
                  and min(dis, pwm) - 5 <= actual <= max(dis, pwm) + 5):
                continue  # PX4 ramps the output from the disarmed value over 0.5 s
            else:
                print("  Output %s us instead of %d us after %.2f s: stopping"
                      % (actual, pwm, time.time() - start))
                ok = False
                break
    finally:
        for _ in range(3):
            fc.actuator_test(float("nan"), 0.0)
            fc.pump(0.2)
    if not confirmed:
        ok = False
    fc.pump(0.8)
    after = fc.output(1, max_age=0.5)
    print("Released. Output %s us (disarmed value %d us)." % (after, dis))
    if after != dis:
        print("WARNING: the output is not back at the disarmed value")
        ok = False
    return 0 if ok else 1


def cmd_watch(fc, seconds):
    """Read-only: log arming changes and MAIN 1/2 PWM changes with wall-clock times."""
    if not 0 < seconds <= 600:
        raise SystemExit("SECONDS must be between 0 and 600")
    end = time.time() + seconds
    last = None
    while time.time() < end:
        fc.pump(0.05)
        state = (fc.armed, fc.outputs.get(1), fc.outputs.get(2))
        if state != last:
            now = time.time()
            stamp = time.strftime("%H:%M:%S", time.localtime(now)) + ".%02d" % int(now % 1 * 100)
            armed = {True: "ARMED", False: "disarmed", None: "unknown"}[state[0]]
            print("%s  %-8s  throttle %s us  steering %s us" % (stamp, armed, state[1], state[2]),
                  flush=True)
            last = state
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", help="MAVLink connection (default: the PX4 USB port, "
                        "or udpin:127.0.0.1:14551 when mavlink_bridge.py is running)")
    parser.add_argument("--baud", type=int, default=57600)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="read-only check of the outputs and ESC parameters")
    apply_parser = sub.add_parser("apply", help="write the bidirectional ESC profile")
    apply_parser.add_argument("--esc-unpowered", action="store_true",
                              help="confirm the ESC battery is disconnected")
    hold_parser = sub.add_parser("hold", help="hold a throttle PWM with the actuator test")
    hold_parser.add_argument("pwm", type=int)
    hold_parser.add_argument("seconds", type=float)
    hold_parser.add_argument("--wide", action="store_true",
                             help="allow PWM_MAIN_MIN1..MAX1 instead of 1400-1600 us")
    watch_parser = sub.add_parser("watch", help="read-only log of arming and PWM changes")
    watch_parser.add_argument("seconds", type=float)
    args = parser.parse_args()

    fc = FlightController(args.device or default_device(), args.baud)
    try:
        if args.command == "status":
            return cmd_status(fc)
        if args.command == "apply":
            return cmd_apply(fc, args.esc_unpowered)
        if args.command == "watch":
            return cmd_watch(fc, args.seconds)
        return cmd_hold(fc, args.pwm, args.seconds, args.wide)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    finally:
        fc.m.close()


if __name__ == "__main__":
    raise SystemExit(main())
