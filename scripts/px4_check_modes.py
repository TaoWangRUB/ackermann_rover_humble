#!/usr/bin/env python3
"""Query PX4 registered external components and monitor unregistration.

Usage:
    # Snapshot: show currently registered modes
    python3 scripts/px4_check_modes.py

    # Monitor: watch for register/unregister events in real time
    python3 scripts/px4_check_modes.py --monitor [SECONDS]

Connects via /dev/ttyACM0 (override with PX4_DEVICE env var).
"""
import os
import sys
import time
from pymavlink import mavutil

DEVICE = os.environ.get('PX4_DEVICE', '/dev/ttyACM0')
BAUD = int(os.environ.get('PX4_BAUD', '57600'))


def connect(timeout=10):
    m = mavutil.mavlink_connection(DEVICE, baud=BAUD, source_system=254)
    for _ in range(5):
        m.mav.heartbeat_send(6, 8, 0, 0, 0)
        time.sleep(0.2)

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg and msg.get_type() != 'BAD_DATA' and msg.get_srcSystem() == 1:
            print(f"Connected to PX4 (system {msg.get_srcSystem()})")
            return m
    print("ERROR: could not connect to PX4", file=sys.stderr)
    m.close()
    return None


def send_shell_cmd(mav, cmd, timeout=8):
    cmd_bytes = (cmd + "\n").encode()
    for i in range(0, len(cmd_bytes), 70):
        chunk = cmd_bytes[i:i+70]
        buf = list(chunk) + [0] * (70 - len(chunk))
        mav.mav.serial_control_send(
            10,
            mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
            mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
            0, 0, len(chunk), buf)

    deadline = time.time() + timeout
    output = b""
    while time.time() < deadline:
        msg = mav.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.5)
        if msg and msg.count > 0:
            output += bytes(msg.data[:msg.count])
            deadline = time.time() + 1.5
        if b"nsh>" in output[-40:]:
            break

    mav.mav.serial_control_send(10, 0, 0, 0, 0, [0] * 70)
    return output.decode('utf-8', errors='replace')


def snapshot(mav):
    print("\n=== PX4 Registered External Components ===")

    raw = send_shell_cmd(mav, "listener register_ext_component_reply -n 20", timeout=10)
    if not raw.strip() or 'never published' in raw.lower():
        print("No register_ext_component_reply data (topic never published or PX4 freshly booted)")
    else:
        for line in raw.split('\n'):
            line = line.strip()
            if not line or line.startswith('nsh>') or line.endswith('nsh>'):
                continue
            if 'listener' in line and 'register_ext' in line:
                continue
            print(f"  {line}")

    print("\n=== Vehicle Status (nav_state & arming) ===")
    raw = send_shell_cmd(mav, "listener vehicle_status -n 1", timeout=6)
    for line in raw.split('\n'):
        line = line.strip()
        if any(k in line for k in ['arming_state', 'nav_state', 'pre_flight', 'valid_nav_states']):
            print(f"  {line}")


def monitor(mav, duration):
    print(f"\n=== Monitoring for register/unregister events ({duration}s) ===")
    print("Watching: register_ext_component_reply, unregister_ext_component\n")

    start = time.time()
    check_interval = 3
    last_check = 0

    while time.time() - start < duration:
        now = time.time()
        if now - last_check >= check_interval:
            last_check = now
            elapsed = int(now - start)

            raw = send_shell_cmd(mav, "listener register_ext_component_reply -n 1", timeout=4)
            ts_line = ""
            name_line = ""
            mode_line = ""
            success_line = ""
            for line in raw.split('\n'):
                s = line.strip()
                if 'timestamp:' in s:
                    ts_line = s
                if 'name:' in s:
                    name_line = s
                if 'mode_id:' in s:
                    mode_line = s
                if 'success:' in s:
                    success_line = s

            raw2 = send_shell_cmd(mav, "listener unregister_ext_component -n 1", timeout=4)
            unreg_found = 'never published' not in raw2.lower() and 'name:' in raw2

            unreg_name = ""
            unreg_mode = ""
            if unreg_found:
                for line in raw2.split('\n'):
                    s = line.strip()
                    if 'name:' in s:
                        unreg_name = s
                    if 'mode_id:' in s:
                        unreg_mode = s

            print(f"[{elapsed:3d}s] reg: {name_line} {mode_line} | unreg: {'YES ' + unreg_name + ' ' + unreg_mode if unreg_found else 'none'}")

    print("\nMonitoring complete.")


def main():
    monitor_mode = '--monitor' in sys.argv
    duration = 60

    if monitor_mode:
        idx = sys.argv.index('--monitor')
        if idx + 1 < len(sys.argv):
            try:
                duration = int(sys.argv[idx + 1])
            except ValueError:
                pass

    mav = connect()
    if not mav:
        sys.exit(1)

    try:
        if monitor_mode:
            monitor(mav, duration)
        else:
            snapshot(mav)
    finally:
        mav.close()


if __name__ == '__main__':
    main()
