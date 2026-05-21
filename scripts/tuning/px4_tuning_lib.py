"""Shared helpers for PX4 Ackermann rover auto-tuning scripts.

Provides:
- MAVLink connection with auto-detection of /dev/ttyACM*
- PX4 parameter read/write
- MAVLink telemetry streaming (LOCAL_POSITION_NED, ATTITUDE, etc.)
- cmd_vel publishing via docker exec
- Pre-flight: mode registration check, activation, arming with retry
- Data recording and basic analysis
"""

import glob
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from pymavlink import mavutil


# ---------------------------------------------------------------------------
# MAVLink connection
# ---------------------------------------------------------------------------

def find_acm_device() -> str:
    """Auto-detect the first available /dev/ttyACM* device."""
    devs = sorted(glob.glob("/dev/ttyACM*"))
    if not devs:
        raise RuntimeError("No /dev/ttyACM* device found — is PX4 USB plugged in?")
    return devs[0]


def _port_is_busy(device: str) -> bool:
    """Check if a serial device is already opened by another process."""
    try:
        with open(f"/proc/locks") as f:
            pass  # /proc/locks doesn't help for char devices
        # Use fuser to check
        result = subprocess.run(
            ["fuser", device], capture_output=True, timeout=2)
        return result.returncode == 0  # 0 = someone is using it
    except Exception:
        return False


def connect_mavlink(device: Optional[str] = None, baud: int = 57600,
                    timeout: float = 10) -> mavutil.mavfile:
    """Connect to PX4 via MAVLink, wait for first valid message.

    If ``device`` looks like a serial port and is already in use (e.g. by
    MAVProxy), fall back to ``udp:127.0.0.1:14550`` automatically.
    """
    if device is None:
        device = find_acm_device()

    # Auto-detect MAVProxy holding the serial port
    if device.startswith("/dev/") and _port_is_busy(device):
        udp = "udpin:0.0.0.0:14551"
        print(f"{device} is busy (bridge?) — connecting via {udp}")
        device = udp
    print(f"Connecting to PX4 on {device} @ {baud}...")
    m = mavutil.mavlink_connection(device, baud=baud, source_system=254)
    for _ in range(5):
        m.mav.heartbeat_send(6, 8, 0, 0, 0)
        time.sleep(0.2)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg and msg.get_type() != "BAD_DATA" and msg.get_srcSystem() == 1:
            print(f"Connected (got {msg.get_type()})")
            return m
    m.close()
    raise RuntimeError(f"No PX4 messages on {device} within {timeout}s")


# ---------------------------------------------------------------------------
# Parameter read / write
# ---------------------------------------------------------------------------

def param_get(mav: mavutil.mavfile, name: str, timeout: float = 3) -> Optional[float]:
    """Read a single PX4 parameter. Returns None if not found."""
    mav.mav.param_request_read_send(1, 1, name.encode(), -1)
    msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
    if msg and msg.param_id.rstrip('\x00') == name:
        return msg.param_value
    return None


def param_set(mav: mavutil.mavfile, name: str, value: float,
              param_type: int = mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
              timeout: float = 3) -> bool:
    """Write a PX4 parameter. Returns True on ACK."""
    mav.mav.param_set_send(1, 1, name.encode(), value, param_type)
    msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
    if msg and msg.param_id.rstrip('\x00') == name:
        if abs(msg.param_value - value) < 1e-4:
            print(f"  {name} = {value:.4f} ✓")
            return True
        print(f"  {name} set failed: sent {value}, got {msg.param_value}")
    return False


def param_get_multi(mav: mavutil.mavfile, names: List[str]) -> Dict[str, Optional[float]]:
    """Read multiple params, reconnecting if needed."""
    result = {}
    for name in names:
        val = param_get(mav, name)
        result[name] = val
        time.sleep(0.1)  # avoid overwhelming the serial link
    return result


# ---------------------------------------------------------------------------
# MAVLink telemetry streaming
# ---------------------------------------------------------------------------

def request_data_stream(mav: mavutil.mavfile, stream_id: int = 0, rate_hz: int = 10):
    """Request MAVLink data stream from PX4."""
    mav.mav.request_data_stream_send(
        1, 1, stream_id, rate_hz, 1)


@dataclass
class TelemetrySample:
    """Single telemetry snapshot."""
    t: float = 0.0           # monotonic time [s]
    vx: float = 0.0          # body-x velocity [m/s]
    vy: float = 0.0          # body-y velocity [m/s]
    speed: float = 0.0       # ground speed [m/s]
    heading: float = 0.0     # yaw [rad, NED CW+]
    yaw_rate: float = 0.0    # yaw rate [rad/s]
    roll: float = 0.0
    pitch: float = 0.0


def collect_telemetry(mav: mavutil.mavfile, duration: float,
                      rate_hz: int = 10) -> List[TelemetrySample]:
    """Collect telemetry for `duration` seconds via MAVLink.

    Uses LOCAL_POSITION_NED for velocity and ATTITUDE for heading/rates.
    """
    # Request streams
    request_data_stream(mav, 6, rate_hz)   # POSITION
    request_data_stream(mav, 10, rate_hz)  # EXTRA1 (attitude)
    time.sleep(0.3)

    samples: List[TelemetrySample] = []
    current = TelemetrySample()
    start = time.monotonic()
    deadline = start + duration

    while time.monotonic() < deadline:
        msg = mav.recv_match(
            type=["LOCAL_POSITION_NED", "ATTITUDE", "GLOBAL_POSITION_INT"],
            blocking=True, timeout=0.5)
        if not msg:
            continue

        now = time.monotonic() - start
        mtype = msg.get_type()

        if mtype == "LOCAL_POSITION_NED":
            current.t = now
            current.vx = msg.vx
            current.vy = msg.vy
            current.speed = math.sqrt(msg.vx**2 + msg.vy**2)
            samples.append(TelemetrySample(**vars(current)))

        elif mtype == "ATTITUDE":
            current.t = now
            current.heading = msg.yaw
            current.yaw_rate = msg.yawspeed
            current.roll = msg.roll
            current.pitch = msg.pitch

    # Stop streams
    request_data_stream(mav, 0, 0)
    return samples


# ---------------------------------------------------------------------------
# cmd_vel publishing
# ---------------------------------------------------------------------------

DC_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "lib", "dc.sh")


def pub_cmd_vel(linear_x: float, angular_z: float, duration: float,
                rate_hz: float = 10, container: Optional[str] = None):
    """Publish cmd_vel inside the Docker container for `duration` seconds."""
    if container is None:
        container = _find_container()
    # Use ros2 topic pub with a rate and timeout
    count = int(duration * rate_hz)
    ros_cmd = (
        f"source /opt/ros/$ROS_DISTRO/setup.bash && "
        f"source /workspace/install/setup.bash && "
        f"ros2 topic pub -r {rate_hz} -t {count} /cmd_vel "
        f"geometry_msgs/msg/TwistStamped "
        f"\"{{header: {{frame_id: ackermann/base_link}}, "
        f"twist: {{linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}}}}}\""
    )
    docker_cmd = [
        "docker", "exec", container, "bash", "-c", ros_cmd
    ]
    proc = subprocess.Popen(docker_cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    return proc


def stop_cmd_vel(container: Optional[str] = None):
    """Publish zero cmd_vel briefly to stop the rover."""
    pub_cmd_vel(0.0, 0.0, 1.0, rate_hz=10, container=container).wait()


# ---------------------------------------------------------------------------
# Pre-flight: mode registration, activation, arming
# ---------------------------------------------------------------------------

_ROS2_SOURCE = (
    "source /opt/ros/$ROS_DISTRO/setup.bash && "
    "source /workspace/install/setup.bash"
)

_CONTAINER_CACHE: Optional[str] = None


def _find_container() -> str:
    """Auto-detect the running ROS2 Docker container name."""
    global _CONTAINER_CACHE
    if _CONTAINER_CACHE:
        return _CONTAINER_CACHE
    try:
        result = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}"],
            capture_output=True, text=True, timeout=5)
        for name in result.stdout.strip().split("\n"):
            if "slam" in name or "ackermann" in name:
                _CONTAINER_CACHE = name
                return name
    except Exception:
        pass
    _CONTAINER_CACHE = "ackermann_slam"  # fallback
    return _CONTAINER_CACHE


def _docker_ros2(cmd: str, container: Optional[str] = None,
                 timeout: float = 10) -> Tuple[int, str]:
    """Run a ros2 command inside the Docker container. Returns (rc, stdout)."""
    if container is None:
        container = _find_container()
    full = f"{_ROS2_SOURCE} && {cmd}"
    try:
        result = subprocess.run(
            ["docker", "exec", container, "bash", "-c", full],
            capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout.strip()
    except subprocess.TimeoutExpired:
        return 1, "timeout"


def is_mode_node_running(node_name: str = "rover_manual_mode",
                         container: Optional[str] = None) -> bool:
    """Check if the mode node is running inside the container."""
    rc, out = _docker_ros2(f"ros2 node list 2>/dev/null | grep -q {node_name}",
                           container)
    return rc == 0


def launch_mode_node(container: Optional[str] = None) -> subprocess.Popen:
    """Launch rover_manual_mode node in background inside the container."""
    if container is None:
        container = _find_container()
    full = (
        f"{_ROS2_SOURCE} && "
        "ros2 run px4_bringup rover_manual_mode "
        "--ros-args -p skip_message_compatibility_check:=true -p use_stamped:=true"
    )
    proc = subprocess.Popen(
        ["docker", "exec", container, "bash", "-c", full],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return proc


def _send_activate_mavlink(mode_id: int, mav: mavutil.mavfile) -> Tuple[bool, str]:
    """Try to activate external mode via MAVLink DO_SET_MODE.

    Uses MAV_CMD_DO_SET_MODE (176) with custom_mode=mode_id.
    Returns (success, detail_message).
    """
    # Drain stale ACKs before sending
    while mav.recv_match(type="COMMAND_ACK", blocking=False):
        pass

    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    mav.mav.command_long_send(
        1, 1,           # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command 176
        0,              # confirmation
        1,              # param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        float(mode_id), # param2: custom_mode (nav_state)
        0, 0, 0, 0, 0)

    # Wait for ACK with matching command
    deadline = time.time() + 3
    while time.time() < deadline:
        ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if ack is None:
            continue
        if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack.result == 0:
                return True, "ACK result=0 (accepted)"
            else:
                result_name = {
                    0: "ACCEPTED", 1: "TEMPORARILY_REJECTED",
                    2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED",
                    5: "IN_PROGRESS", 6: "CANCELLED",
                }.get(ack.result, "UNKNOWN")
                return False, f"ACK result={ack.result} ({result_name})"
        # ACK for a different command — keep waiting
    return False, "no ACK received for MAV_CMD_DO_SET_MODE (176)"


def _send_activate_dds(mode_id: int, container: Optional[str] = None) -> bool:
    """Send a single DDS VehicleCommand(100001) to request mode switch."""
    yaml = (
        f'"{{command: 100001, param1: {mode_id}.0, '
        'target_system: 1, target_component: 1, '
        'source_system: 255, source_component: 0, from_external: true}"'
    )
    cmd = (
        "ros2 topic pub --once /fmu/in/vehicle_command "
        f"px4_msgs/msg/VehicleCommand {yaml}"
    )
    rc, _ = _docker_ros2(cmd, container, timeout=60)
    return rc == 0


def activate_mode(mode_id: int, mav: mavutil.mavfile = None,
                  container: Optional[str] = None,
                  retries: int = 3, retry_delay: float = 5) -> bool:
    """Switch PX4 into the given external mode with retry.

    Tries MAVLink COMMAND_LONG(100001) first (instant), falls back to DDS
    (~20s discovery delay). Confirms via MAVLink heartbeat nav_state.
    """
    for attempt in range(1, retries + 1):
        # --- Try MAVLink first (fast path) ---
        if mav is not None:
            print(f"    Attempt {attempt}/{retries}: activating via MAVLink "
                  f"COMMAND_LONG(100001, param1={mode_id})...")
            ok, detail = _send_activate_mavlink(mode_id, mav)
            print(f"    MAVLink: {detail}")
            if ok:
                time.sleep(0.5)
                current = check_nav_state(mav, timeout=3)
                if current == mode_id:
                    return True
                print(f"    nav_state={current}, expected {mode_id}")

        # --- Fallback: DDS (slow path) ---
        print(f"    Attempt {attempt}/{retries}: activating via DDS "
              f"VehicleCommand 100001 (~20s for discovery...)")
        ok = _send_activate_dds(mode_id, container)
        if ok:
            time.sleep(1)
            current = check_nav_state(mav, timeout=3) if mav else None
            if current == mode_id:
                return True
            print(f"    nav_state={current}, expected {mode_id}")
        else:
            print(f"    DDS publish failed")

        if attempt < retries:
            print(f"    Retrying in {retry_delay}s...")
            time.sleep(retry_delay)

    return False


def send_arm_command(mav: mavutil.mavfile = None,
                     container: Optional[str] = None) -> bool:
    """Arm the vehicle. Prefers MAVLink when *mav* is provided."""
    if mav is not None:
        mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)  # param1=1 → arm
        ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        return ack is not None and ack.result == 0
    # Fallback: DDS
    yaml = (
        '"{command: 400, param1: 1.0, '
        'target_system: 1, target_component: 1, '
        'source_system: 255, source_component: 0, from_external: true}"'
    )
    cmd = (
        "ros2 topic pub --once /fmu/in/vehicle_command "
        f"px4_msgs/msg/VehicleCommand {yaml}"
    )
    rc, _ = _docker_ros2(cmd, container, timeout=60)
    return rc == 0


def check_armed(mav: mavutil.mavfile, timeout: float = 3) -> bool:
    """Check if PX4 reports armed state via MAVLink HEARTBEAT."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == 1:
            return bool(msg.base_mode & 128)
    return False


def check_nav_state(mav: mavutil.mavfile, timeout: float = 3) -> Optional[int]:
    """Read current nav_state from SYS_STATUS / HEARTBEAT custom_mode."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == 1:
            return msg.custom_mode
    return None


def ensure_mode_and_arm(mav: mavutil.mavfile, mode_id: int = 23,
                        container: Optional[str] = None) -> bool:
    """Ensure custom mode is registered, activated, and vehicle is armed.

    Returns True if armed successfully, False if user aborted.
    """
    # 1. Check if mode node is running
    print(f"\n{'='*60}")
    print("PRE-FLIGHT CHECK")
    print(f"{'='*60}")

    if is_mode_node_running(container=container):
        print(f"  Mode node running ✓")
    else:
        print(f"  Mode node not found — launching rover_manual_mode...")
        launch_mode_node(container=container)
        print(f"  Launched (registration will be confirmed during activation)")

    time.sleep(0.5)

    # 2. Check current nav_state
    current = check_nav_state(mav)
    if current is not None and current == mode_id:
        print(f"  Already in mode {mode_id} ✓")
    else:
        print(f"  Activating mode {mode_id}...")
        if activate_mode(mode_id, mav=mav, container=container):
            print(f"  Mode {mode_id} active ✓")
        else:
            print(f"  ERROR: Failed to activate mode {mode_id} after retries")
            return False

    # 3. Check if already armed
    if check_armed(mav):
        print(f"  Already armed ✓")
        print(f"{'='*60}\n")
        return True

    # 4. Arm with retry
    if not confirm("Arm the rover?"):
        return False

    max_attempts = 3
    for attempt in range(1, max_attempts + 1):
        print(f"  Arming attempt {attempt}/{max_attempts}...")
        send_arm_command(mav=mav, container=container)
        time.sleep(1.5)

        if check_armed(mav):
            print(f"  Armed ✓")
            print(f"{'='*60}\n")
            return True

        print(f"  Arming failed (PX4 preflight checks may not pass)")
        if attempt < max_attempts:
            resp = input("  Clear the fault and retry? [y/N] ").strip().lower()
            if resp not in ("y", "yes"):
                return False
            time.sleep(1)

    print("  Arming failed after all attempts. Exiting.")
    return False


# ---------------------------------------------------------------------------
# Analysis helpers
# ---------------------------------------------------------------------------

def compute_stats(values: List[float]) -> Tuple[float, float, float, float]:
    """Return (mean, std, min, max) for a list of values."""
    if not values:
        return 0, 0, 0, 0
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / max(n - 1, 1)
    std = math.sqrt(var)
    return mean, std, min(values), max(values)


def steady_state_samples(samples: List[TelemetrySample],
                         skip_seconds: float = 2.0) -> List[TelemetrySample]:
    """Skip initial transient, return steady-state portion."""
    return [s for s in samples if s.t >= skip_seconds]


def compute_tracking_error(setpoint: float,
                           measured: List[float]) -> Tuple[float, float]:
    """Return (mean_error, rms_error) for setpoint tracking."""
    if not measured:
        return 0, 0
    errors = [m - setpoint for m in measured]
    mean_err = sum(errors) / len(errors)
    rms = math.sqrt(sum(e**2 for e in errors) / len(errors))
    return mean_err, rms


# ---------------------------------------------------------------------------
# Safety
# ---------------------------------------------------------------------------

_abort = False


def _signal_handler(sig, frame):
    global _abort
    _abort = True
    print("\n*** ABORT — stopping rover ***")


def install_abort_handler():
    """Install Ctrl+C handler that sets abort flag."""
    global _abort
    _abort = False
    signal.signal(signal.SIGINT, _signal_handler)


def is_aborted() -> bool:
    return _abort


def confirm(prompt: str) -> bool:
    """Ask user for confirmation. Returns True if yes."""
    resp = input(f"{prompt} [y/N] ").strip().lower()
    return resp in ("y", "yes")
