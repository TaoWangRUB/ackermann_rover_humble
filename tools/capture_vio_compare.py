#!/usr/bin/env python3
"""
Capture 50 s of T265 hardware VIO and VINS odometry, then:
  - Generate a PDF with x-y trajectory comparison
  - Print FPS statistics for both odom streams and the T265 image stream
"""
import sys
import time
import math
import collections
import threading
import argparse

import subprocess

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

CAPTURE_SECONDS = 50


class VioCapture(Node):
    def __init__(self):
        super().__init__('vio_capture')

        self.t265_odom: list[tuple[float, float, float]] = []   # (ros_t, x, y)
        self.vins_odom: list[tuple[float, float, float]] = []
        self.img_stamps: list[float] = []

        self.create_subscription(Odometry, '/t265/odom',
                                 self._t265_cb, 10)
        self.create_subscription(Odometry, '/vins_odom',
                                 self._vins_cb, 10)
        self.create_subscription(Image, '/t265/fisheye1/image_raw',
                                 self._img_cb, 10)

    # ---------- callbacks ----------
    def _t265_cb(self, msg: Odometry):
        t = _stamp(msg)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.t265_odom.append((t, x, y))

    def _vins_cb(self, msg: Odometry):
        t = _stamp(msg)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.vins_odom.append((t, x, y))

    def _img_cb(self, msg: Image):
        self.img_stamps.append(_stamp(msg))


# ------------------------------------------------------------------
# GPU monitoring
# ------------------------------------------------------------------
_gpu_samples: list[tuple[float, float, float, float]] = []  # (wall_t, util%, mem_MB, power_W)
_gpu_stop = threading.Event()


def _gpu_poll_thread():
    """Background thread: poll nvidia-smi every 0.5 s."""
    while not _gpu_stop.is_set():
        try:
            result = subprocess.run(
                ['nvidia-smi',
                 '--query-gpu=utilization.gpu,memory.used,power.draw',
                 '--format=csv,noheader,nounits'],
                capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                parts = [p.strip() for p in result.stdout.strip().split(',')]
                if len(parts) == 3:
                    _gpu_samples.append((
                        time.time(),
                        float(parts[0]),   # GPU util %
                        float(parts[1]),   # mem used MB
                        float(parts[2]),   # power W
                    ))
        except Exception:
            pass
        _gpu_stop.wait(0.5)


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# ------------------------------------------------------------------
def fps_stats(stamps: list[float], name: str) -> dict:
    if len(stamps) < 2:
        print(f"  {name}: too few samples ({len(stamps)})")
        return {}
    stamps = sorted(stamps)
    gaps = [stamps[i+1] - stamps[i] for i in range(len(stamps)-1)]
    mean_gap = sum(gaps) / len(gaps)
    min_gap  = min(gaps)
    max_gap  = max(gaps)
    fps_mean = 1.0 / mean_gap if mean_gap > 0 else float('nan')
    fps_min  = 1.0 / max_gap  if max_gap  > 0 else float('nan')
    fps_max  = 1.0 / min_gap  if min_gap  > 0 else float('nan')
    # std dev of gap
    var = sum((g - mean_gap)**2 for g in gaps) / len(gaps)
    std_gap = math.sqrt(var)
    total_msgs = len(stamps)
    wall_span  = stamps[-1] - stamps[0]
    return dict(name=name, total=total_msgs, wall_span=wall_span,
                fps_mean=fps_mean, fps_min=fps_min, fps_max=fps_max,
                std_gap=std_gap*1000, mean_gap=mean_gap*1000)


def print_stats(s: dict):
    if not s:
        return
    print(f"\n{'─'*52}")
    print(f"  {s['name']}")
    print(f"{'─'*52}")
    print(f"  Messages  : {s['total']}")
    print(f"  Time span : {s['wall_span']:.2f} s")
    print(f"  FPS mean  : {s['fps_mean']:.2f}")
    print(f"  FPS min   : {s['fps_min']:.2f}   FPS max: {s['fps_max']:.2f}")
    print(f"  Gap mean  : {s['mean_gap']:.2f} ms   σ = {s['std_gap']:.2f} ms")


def print_gpu_stats(samples: list, t0: float):
    if not samples:
        print("\n  GPU stats: no nvidia-smi data collected")
        return
    utils  = [s[1] for s in samples]
    mems   = [s[2] for s in samples]
    powers = [s[3] for s in samples]
    wall_t = [s[0] - t0 for s in samples]
    print(f"\n{'─'*52}")
    print(f"  GPU (nvidia-smi, N={len(samples)} samples @ 0.5 s)")
    print(f"{'─'*52}")
    print(f"  Util  : mean={sum(utils)/len(utils):.1f}%  "
          f"min={min(utils):.1f}%  max={max(utils):.1f}%")
    print(f"  Mem   : mean={sum(mems)/len(mems):.0f}MB  "
          f"min={min(mems):.0f}MB  max={max(mems):.0f}MB")
    print(f"  Power : mean={sum(powers)/len(powers):.1f}W  "
          f"min={min(powers):.1f}W  max={max(powers):.1f}W")


# ------------------------------------------------------------------
def plot_pdf(t265_data, vins_data, gpu_samples: list, out_path: str, capture_t0: float):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    import numpy as np

    t265_t = np.array([d[0] for d in t265_data])
    t265_x = np.array([d[1] for d in t265_data])
    t265_y = np.array([d[2] for d in t265_data])

    vins_t = np.array([d[0] for d in vins_data])
    vins_x = np.array([d[1] for d in vins_data])
    vins_y = np.array([d[2] for d in vins_data])

    # normalise time to start at 0
    t0 = min(t265_t[0] if len(t265_t) else 0,
             vins_t[0]  if len(vins_t)  else 0)
    t265_t -= t0
    vins_t  -= t0

    has_gpu = len(gpu_samples) >= 2
    nrows   = 3 if has_gpu else 2

    fig = plt.figure(figsize=(11, 4.5 * nrows))
    gs  = gridspec.GridSpec(nrows, 2, figure=fig, hspace=0.40, wspace=0.35)

    # ── 1. XY trajectory ──
    ax_xy = fig.add_subplot(gs[0, :])
    ax_xy.plot(t265_x, t265_y, color='steelblue',  lw=1.2, label='T265 hw VIO')
    ax_xy.plot(vins_x, vins_y, color='darkorange',  lw=1.2, label='VINS')
    if len(t265_x):
        ax_xy.scatter([t265_x[0]], [t265_y[0]], color='steelblue', marker='o', s=40, zorder=5)
    if len(vins_x):
        ax_xy.scatter([vins_x[0]], [vins_y[0]], color='darkorange', marker='o', s=40, zorder=5)
    ax_xy.set_xlabel('X  (m)')
    ax_xy.set_ylabel('Y  (m)')
    ax_xy.set_title('X-Y Trajectory  (50 s capture)')
    ax_xy.legend(loc='best', fontsize=9)
    ax_xy.set_aspect('equal', adjustable='datalim')
    ax_xy.grid(True, ls='--', alpha=0.4)

    # ── 2. X vs time ──
    ax_x = fig.add_subplot(gs[1, 0])
    ax_x.plot(t265_t, t265_x, color='steelblue', lw=0.9, label='T265')
    ax_x.plot(vins_t,  vins_x,  color='darkorange',  lw=0.9, label='VINS')
    ax_x.set_xlabel('Time  (s)')
    ax_x.set_ylabel('X  (m)')
    ax_x.set_title('X position vs time')
    ax_x.legend(fontsize=8)
    ax_x.grid(True, ls='--', alpha=0.4)

    # ── 3. Y vs time ──
    ax_y = fig.add_subplot(gs[1, 1])
    ax_y.plot(t265_t, t265_y, color='steelblue', lw=0.9, label='T265')
    ax_y.plot(vins_t,  vins_y,  color='darkorange',  lw=0.9, label='VINS')
    ax_y.set_xlabel('Time  (s)')
    ax_y.set_ylabel('Y  (m)')
    ax_y.set_title('Y position vs time')
    ax_y.legend(fontsize=8)
    ax_y.grid(True, ls='--', alpha=0.4)

    # ── 4. GPU utilisation (optional) ──
    if has_gpu:
        gpu_t     = np.array([s[0] - capture_t0 for s in gpu_samples])
        gpu_util  = np.array([s[1] for s in gpu_samples])
        gpu_mem   = np.array([s[2] for s in gpu_samples])
        gpu_power = np.array([s[3] for s in gpu_samples])

        ax_gut = fig.add_subplot(gs[2, 0])
        ax_gut.plot(gpu_t, gpu_util, color='mediumseagreen', lw=1.0)
        ax_gut.fill_between(gpu_t, gpu_util, alpha=0.2, color='mediumseagreen')
        ax_gut.set_xlabel('Time  (s)')
        ax_gut.set_ylabel('GPU Util  (%)')
        ax_gut.set_title('GPU Utilisation')
        ax_gut.set_ylim(0, 105)
        ax_gut.grid(True, ls='--', alpha=0.4)
        mean_util = float(np.mean(gpu_util))
        ax_gut.axhline(mean_util, color='mediumseagreen', ls=':', alpha=0.8)
        ax_gut.text(gpu_t[-1]*0.98, mean_util + 2, f'mean {mean_util:.0f}%',
                    ha='right', fontsize=7, color='darkgreen')

        ax_gmem = fig.add_subplot(gs[2, 1])
        ax_gmem.plot(gpu_t, gpu_mem, color='orchid', lw=1.0)
        ax_gmem.fill_between(gpu_t, gpu_mem, alpha=0.2, color='orchid')
        ax_gmem.set_xlabel('Time  (s)')
        ax_gmem.set_ylabel('GPU Mem  (MB)')
        ax_gmem.set_title('GPU Memory Usage')
        ax_gmem.grid(True, ls='--', alpha=0.4)
        # annotate max mem
        ax_gmem.axhline(float(np.max(gpu_mem)), color='orchid', ls=':', alpha=0.8)
        ax_gmem.text(gpu_t[-1]*0.98, float(np.max(gpu_mem)) + 5,
                     f'max {float(np.max(gpu_mem)):.0f} MB',
                     ha='right', fontsize=7, color='purple')

    fig.suptitle('VIO Comparison: VINS vs T265 hardware odometry\n'
                 f'Capture duration ≈ {CAPTURE_SECONDS} s',
                 fontsize=12, y=0.99)

    fig.savefig(out_path, bbox_inches='tight', dpi=150)
    plt.close(fig)
    print(f"\n  Plot saved → {out_path}")


# ------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='/tmp/vio_compare.pdf')
    args = parser.parse_args()

    rclpy.init()
    node = VioCapture()

    # Start GPU monitoring thread
    gpu_thread = threading.Thread(target=_gpu_poll_thread, daemon=True)
    gpu_thread.start()

    print(f"Capturing {CAPTURE_SECONDS} s of VIO data …")
    capture_start = time.time()
    deadline = capture_start + CAPTURE_SECONDS

    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = CAPTURE_SECONDS - (deadline - time.time())
        if int(elapsed) % 10 == 0 and elapsed > 0:
            gpu_util_str = (
                f"  gpu={_gpu_samples[-1][1]:.0f}%" if _gpu_samples else ""
            )
            sys.stdout.write(
                f"\r  {elapsed:.0f}/{CAPTURE_SECONDS}s  "
                f"t265={len(node.t265_odom)}  vins={len(node.vins_odom)}  "
                f"imgs={len(node.img_stamps)}{gpu_util_str}   "
            )
            sys.stdout.flush()

    # Stop GPU thread
    _gpu_stop.set()
    gpu_thread.join(timeout=2.0)

    print(f"\nCapture done.  "
          f"t265={len(node.t265_odom)}  vins={len(node.vins_odom)}  imgs={len(node.img_stamps)}  "
          f"gpu_samples={len(_gpu_samples)}")

    # --- stats ---
    s1 = fps_stats([d[0] for d in node.t265_odom], 'T265 /t265/odom')
    s2 = fps_stats([d[0] for d in node.vins_odom],  'VINS /vins_odom')
    s3 = fps_stats(node.img_stamps,                  'T265 fisheye1 /image_raw')
    print_stats(s1)
    print_stats(s2)
    print_stats(s3)
    print_gpu_stats(_gpu_samples, capture_start)

    # --- plot ---
    plot_pdf(node.t265_odom, node.vins_odom, _gpu_samples, args.out, capture_start)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
