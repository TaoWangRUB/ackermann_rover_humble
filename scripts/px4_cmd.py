#!/usr/bin/env python3
"""Run a PX4 NSH command via MAVLink shell protocol and print the output.

Usage:
    python3 scripts/px4_cmd.py "listener cpuload -n 1"
    python3 scripts/px4_cmd.py "commander status" 8

Connects to PX4 via /dev/ttyACM0 using pymavlink SERIAL_CONTROL.
"""
import os
import sys
import time
import serial
from pymavlink import mavutil

DEVICE = os.environ.get('PX4_DEVICE', '/dev/ttyACM0')
BAUD = int(os.environ.get('PX4_BAUD', '57600'))
ASSERT_DTR = os.environ.get('PX4_ASSERT_DTR', '0') == '1'

def _assert_dtr(device, baud):
    """Assert DTR to wake PX4's USB MAVLink instance."""
    try:
        ser = serial.Serial(device, baud, dsrdtr=True, timeout=0.5)
        ser.setDTR(True)
        time.sleep(0.5)
        ser.close()
    except Exception:
        pass  # best-effort; pymavlink may still work without it

def connect_mavlink(retries=6):
    """Connect to PX4 with retry to handle USB CDC flakiness."""
    if ASSERT_DTR:
        _assert_dtr(DEVICE, BAUD)
    for attempt in range(retries):
        try:
            mav = mavutil.mavlink_connection(DEVICE, baud=BAUD, source_system=254)
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            hb = mav.wait_heartbeat(timeout=5)
            if hb:
                return mav
            print(f"Attempt {attempt+1}: no heartbeat, retrying...", file=sys.stderr)
            mav.close()
        except Exception as e:
            print(f"Attempt {attempt+1}: {e}, retrying...", file=sys.stderr)
            try:
                mav.close()
            except Exception:
                pass
        time.sleep(3)
    return None


def run_serial_command(mav, cmd, timeout):
    """Send a command via SERIAL_CONTROL and return decoded shell output."""
    cmd_bytes = (cmd + "\n").encode()

    for dev_id in [10, 0]:
        for i in range(0, len(cmd_bytes), 70):
            chunk = cmd_bytes[i:i+70]
            buf = list(chunk) + [0] * (70 - len(chunk))
            mav.mav.serial_control_send(
                dev_id,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, len(chunk), buf)

        deadline = time.time() + timeout
        output = b""
        while time.time() < deadline:
            msg = mav.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.5)
            if msg is not None and msg.count > 0:
                output += bytes(msg.data[:msg.count])
                deadline = time.time() + 1.0
            if b"nsh>" in output[-30:]:
                break

        if output:
            mav.mav.serial_control_send(dev_id, 0, 0, 0, 0, [0] * 70)
            text = output.decode('utf-8', errors='replace')
            lines = []
            for line in text.split('\n'):
                stripped = line.strip()
                if stripped and stripped != cmd.strip() and stripped != 'nsh>' and not stripped.endswith('nsh>'):
                    lines.append(line.rstrip())
            return '\n'.join(lines)

    return None

def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(__doc__.strip())
        sys.exit(0 if len(sys.argv) >= 2 else 1)

    cmd = sys.argv[1]
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    last_error = None
    for attempt in range(4):
        mav = connect_mavlink(retries=3)
        if not mav:
            last_error = "No heartbeat from PX4 after retries"
            continue

        try:
            output = run_serial_command(mav, cmd, timeout)
            if output is not None:
                if output:
                    print(output)
                mav.close()
                return
            last_error = "SERIAL_CONTROL not supported or shell not ready yet"
            print(f"Command attempt {attempt+1}: {last_error}, retrying...", file=sys.stderr)
        except Exception as exc:
            last_error = str(exc)
            print(f"Command attempt {attempt+1}: {exc}, retrying...", file=sys.stderr)
        finally:
            try:
                mav.close()
            except Exception:
                pass

        time.sleep(2)

    print(f"ERROR: {last_error or 'No heartbeat from PX4 after retries'}", file=sys.stderr)
    print("Hint: on this Cube Black setup, USB MAVLink/NSH on /dev/ttyACM0 may need a few seconds to stabilize after reboot or replug.", file=sys.stderr)
    print("Hint: verify the autopilot is connected directly to host USB, not through a flaky hub.", file=sys.stderr)
    print("Hint: if necessary, set PX4_DEVICE=/dev/ttyACM0 or PX4_BAUD=57600 in the environment.", file=sys.stderr)
    sys.exit(1)

if __name__ == '__main__':
    main()
