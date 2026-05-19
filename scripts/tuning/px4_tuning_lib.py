"""Shared helpers for PX4 Ackermann rover auto-tuning scripts.

Provides:
- MAVLink connection with auto-detection of /dev/ttyACM*
- PX4 parameter read/write
- MAVLink telemetry streaming (LOCAL_POSITION_NED, ATTITUDE, etc.)
- cmd_vel publishing via docker exec
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


def connect_mavlink(device: Optional[str] = None, baud: int = 57600,
                    timeout: float = 10) -> mavutil.mavfile:
    """Connect to PX4 via MAVLink, wait for first valid message."""
    if device is None:
        device = find_acm_device()
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
                rate_hz: float = 10, container: str = "ackermann_slam"):
    """Publish cmd_vel inside the Docker container for `duration` seconds."""
    # Use ros2 topic pub with a rate and timeout
    count = int(duration * rate_hz)
    ros_cmd = (
        f"source /opt/ros/$ROS_DISTRO/setup.bash && "
        f"source /workspace/install/setup.bash && "
        f"ros2 topic pub -r {rate_hz} -t {count} /cmd_vel "
        f"geometry_msgs/msg/Twist "
        f"'{{linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}}}'"
    )
    docker_cmd = [
        "docker", "exec", container, "bash", "-c", ros_cmd
    ]
    proc = subprocess.Popen(docker_cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    return proc


def stop_cmd_vel(container: str = "ackermann_slam"):
    """Publish zero cmd_vel briefly to stop the rover."""
    pub_cmd_vel(0.0, 0.0, 1.0, rate_hz=10, container=container).wait()


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
