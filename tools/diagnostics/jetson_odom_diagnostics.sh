#!/usr/bin/env bash
# Jetson odometry diagnostics script
# Run on the Jetson (SSH), with ROS 2 sourced (e.g. source ~/.bashrc)

LOG="/tmp/jetson_odom_diag_$(date +%Y%m%d-%H%M%S).log"
exec 2>&1

echo "Jetson odom diagnostics started: $(date)" | tee -a "$LOG"
echo "Host info:" | tee -a "$LOG"
uname -a | tee -a "$LOG"
lscpu | tee -a "$LOG"

echo "\nEnvironment:" | tee -a "$LOG"
echo "ROS_DISTRO=${ROS_DISTRO:-unset}" | tee -a "$LOG"
python3 -V 2>&1 | tee -a "$LOG"

echo "\nROS2 nodes:" | tee -a "$LOG"
ros2 node list 2>&1 | tee -a "$LOG"

echo "\nROS2 topics (filtered):" | tee -a "$LOG"
ros2 topic list 2>&1 | tee -a "$LOG"

collect_hz() {
  topic=$1
  w=${2:-5}
  echo "\n>>> ros2 topic hz $topic -w $w" | tee -a "$LOG"
  ros2 topic hz "$topic" -w "$w" 2>&1 | tee -a "$LOG" || true
}

# Topic rate checks (adjust names if remapped in your system)
collect_hz /t265/odom 5
collect_hz /d435i/imu/raw 5
collect_hz /imu/data 5
collect_hz /px4_vehicle_odom 10
collect_hz /px4_vehicle_odom_base 10
collect_hz /fmu/out/vehicle_odometry 5

echo "\nTop processes (ps):" | tee -a "$LOG"
ps aux --sort=-%cpu | head -n 60 | tee -a "$LOG"

echo "\nTop (batch):" | tee -a "$LOG"
top -b -n 1 | head -n 60 | tee -a "$LOG"

echo "\nFinding PIDs for nodes of interest:" | tee -a "$LOG"
for p in px4_vehicle_odometry.py px4_vision_odom.py imu_transformer odom_tf_relay; do
  echo "=== $p ===" | tee -a "$LOG"
  pgrep -af "$p" 2>&1 | tee -a "$LOG" || true
done

# Save per-PID ps + /proc info (if present)
echo "\nPer-PID details:" | tee -a "$LOG"
for name in px4_vehicle_odometry.py px4_vision_odom.py imu_transformer odom_tf_relay; do
  pgrep -af "$name" | awk '{print $1}' | while read -r pid; do
    [ -z "$pid" ] && continue
    echo "PID $pid ($name):" | tee -a "$LOG"
    ps -p "$pid" -o pid,ppid,%cpu,%mem,cmd | tee -a "$LOG"
    if [ -r "/proc/$pid/status" ]; then
      echo "--- /proc/$pid/status ---" | tee -a "$LOG"
      sed -n '1,60p' "/proc/$pid/status" | tee -a "$LOG"
    fi
  done
done

echo "\nTF frames (tf2_tools view_frames):" | tee -a "$LOG"
# view_frames may create files; wrap in timeout to avoid hangs
timeout 5s ros2 run tf2_tools view_frames 2>&1 | tee -a "$LOG" || true

# TF lookup latency probe (rclpy)
echo "\nTF lookup latency probe (rclpy):" | tee -a "$LOG"
python3 - <<'PY' 2>&1 | tee -a "$LOG"
import time
import rclpy
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

rclpy.init()
node = rclpy.create_node('tf_probe')
buf = Buffer()
TransformListener(buf, node)
frames = [ ('odom','ackermann/base_link'), ('cubepilot_link','ackermann/base_link') ]
for src,tgt in frames:
    times = []
    for i in range(5):
        t0 = time.time()
        try:
            buf.lookup_transform(src, tgt, Time())
            ok = True
        except Exception as e:
            ok = False
        t1 = time.time()
        times.append((t1-t0, ok))
        time.sleep(0.05)
    print(f"lookup {src} <- {tgt}: {times}")

rclpy.shutdown()
PY

echo "\nDone. Saved log to: $LOG" | tee -a "$LOG"

cat <<'HINT' | tee -a "$LOG"
Profiling hints (run manually if deeper traces are needed):
- py-spy (no restart, requires installation):
    sudo py-spy top --pid <PID> --duration 10
- cProfile (restart node under profiler):
    python3 -m cProfile -o /tmp/prof.out path/to/node_script.py
- perf (system-level, requires sudo):
    sudo perf record -F 99 -p <PID> -g -- sleep 10; sudo perf report
HINT

exit 0
