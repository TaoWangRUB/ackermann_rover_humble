#!/usr/bin/env python3
"""Sweep (exposure, gain) on a RealSense camera and report image-quality metrics.

Designed to be run with the camera **stationary**, pointing at a representative
scene (kitchen, parking lot, hallway, etc.). It writes a CSV summary, optionally
saves a representative frame per (exp, gain) cell, and prints a recommended
(exp, gain) pair based on brightness/sharpness/feature targets.

Usage from inside the container:
    python3 /workspace/scripts/sweep_exp_gain.py --preset dim
    python3 /workspace/scripts/sweep_exp_gain.py --preset dark --camera d435i
    python3 /workspace/scripts/sweep_exp_gain.py --preset normal --save-frames
    python3 /workspace/scripts/sweep_exp_gain.py \\
        --exposures 1000,2000,4000,6000,8000 --gains 64,128,192,248

Presets (starting points; refine with --exposures / --gains as needed):
    bright   sunny daylight indoor (windows + sun): exp 40..480, gain 16..128
    normal   normal indoor (lamps + daylight):       exp 80..960, gain 16..128
    dim      dim indoor (single lamp / cloudy / dusk): exp 200..3000, gain 32..192
    dark     very dark (parking lot / night):         exp 1000..8000, gain 64..248

Targets in the verdict logic:
    brightness 80-160     (sweet spot for ORB)
    Laplacian >= 80       (sharp; lower = blur or out of focus)
    ORB count >= 400      (raw feature count comfortable for RTAB-Map)
    saturation < 10%      (no significant highlight clipping)
"""
import argparse
import statistics
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


PRESETS = {
    'bright':  {'exp': [40, 80, 120, 160, 240, 320, 480],          'gain': [16, 32, 64, 128]},
    'normal':  {'exp': [80, 160, 240, 320, 480, 640, 960],          'gain': [16, 32, 64, 128]},
    'dim':     {'exp': [200, 400, 800, 1200, 1800, 2400, 3000],     'gain': [32, 64, 128, 192]},
    'dark':    {'exp': [1000, 2000, 3000, 4500, 6000, 7500, 8000],  'gain': [64, 128, 192, 248]},
}


def parse_int_list(s):
    return [int(x) for x in s.split(',') if x.strip()]


def set_param(node_name, name, value):
    subprocess.run(
        ['ros2', 'param', 'set', node_name, name, str(value)],
        capture_output=True, check=False,
    )


class IQ(Node):
    def __init__(self, topic: str):
        super().__init__('iq_sweep')
        self._orb = cv2.ORB_create(nfeatures=2000)
        self._reset()

        self.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)

    def _reset(self):
        self._brights = []; self._laps = []; self._orbs = []; self._sats = []
        self._latest_bgr = None

    def _cb(self, msg):
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
        if arr.shape[2] == 3:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == 'rgb8' else arr
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        else:
            bgr = cv2.cvtColor(arr[:, :, 0], cv2.COLOR_GRAY2BGR)
            gray = arr[:, :, 0]
        self._brights.append(float(gray.mean()))
        self._laps.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
        self._orbs.append(len(self._orb.detect(gray, None)))
        self._sats.append(100.0 * (gray >= 250).mean())
        self._latest_bgr = bgr

    def stats(self):
        if not self._brights:
            return None
        return {
            'n':       len(self._brights),
            'bright':  statistics.median(self._brights),
            'lap':     statistics.median(self._laps),
            'orb':     statistics.median(self._orbs),
            'sat_pct': statistics.median(self._sats),
            'frame':   self._latest_bgr,
        }


def verdict_flags(s):
    flags = []
    if s['bright'] < 60:    flags.append('DARK')
    if s['bright'] > 180:   flags.append('OVEREXP')
    if s['lap']    < 80:    flags.append('BLUR')
    if s['orb']    < 400:   flags.append('LOW_TEX')
    if s['sat_pct'] > 10:   flags.append('CLIP')
    return ','.join(flags) if flags else 'OK'


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                 description=__doc__)
    ap.add_argument('--preset', choices=PRESETS.keys(), default='normal',
                    help='exposure/gain grid preset (default: normal)')
    ap.add_argument('--exposures', type=parse_int_list, default=None,
                    help='comma-separated exposure values (overrides preset)')
    ap.add_argument('--gains', type=parse_int_list, default=None,
                    help='comma-separated gain values (overrides preset)')
    ap.add_argument('--camera', default='d435i',
                    help='camera node name (default: d435i)')
    ap.add_argument('--topic', default=None,
                    help='color image topic (default: /<camera>/color/image_raw)')
    ap.add_argument('--sample-s', type=float, default=4.0,
                    help='seconds to sample per cell (default 4)')
    ap.add_argument('--warmup-s', type=float, default=1.5,
                    help='seconds to wait after applying params before sampling (default 1.5)')
    ap.add_argument('--save-frames', action='store_true',
                    help='save one frame per (exp, gain) cell to /tmp/sweep_frames/')
    ap.add_argument('--csv', default='/tmp/exp_gain_sweep.csv',
                    help='CSV output path (default: /tmp/exp_gain_sweep.csv)')
    args = ap.parse_args()

    exposures = args.exposures or PRESETS[args.preset]['exp']
    gains     = args.gains     or PRESETS[args.preset]['gain']
    topic     = args.topic or f'/{args.camera}/color/image_raw'
    node_name = f'/{args.camera}'

    out_dir = Path('/tmp/sweep_frames')
    if args.save_frames:
        out_dir.mkdir(parents=True, exist_ok=True)

    print(f"sweep preset={args.preset}  topic={topic}  node={node_name}")
    print(f"exposures: {exposures}")
    print(f"gains:     {gains}")
    print(f"sample window: {args.sample_s}s per cell, warmup {args.warmup_s}s")
    print()

    rclpy.init()
    node = IQ(topic)
    set_param(node_name, 'rgb_camera.enable_auto_exposure', 'false')

    print(f'{"exp":>5} {"gain":>5}   {"bright":>6} {"lap":>6} {"orb":>5} {"sat%":>5}  verdict')
    rows = []
    for exp in exposures:
        for gain in gains:
            set_param(node_name, 'rgb_camera.exposure', exp)
            set_param(node_name, 'rgb_camera.gain', gain)
            time.sleep(args.warmup_s)
            node._reset()
            end = time.monotonic() + args.sample_s
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.2)
            s = node.stats()
            if s is None:
                print(f'{exp:>5} {gain:>5}   no samples')
                continue
            verdict = verdict_flags(s)
            print(f'{exp:>5} {gain:>5}   {s["bright"]:6.1f} {s["lap"]:6.1f} '
                  f'{int(s["orb"]):>5d} {s["sat_pct"]:5.1f}  {verdict}', flush=True)
            rows.append((exp, gain, s['bright'], s['lap'], s['orb'], s['sat_pct'], verdict))
            if args.save_frames and s['frame'] is not None:
                stamp = f"e{exp:05d}_g{gain:03d}"
                cv2.putText(s['frame'],
                            f"exp={exp} gain={gain} bright={s['bright']:.0f} ORB={int(s['orb'])}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.imwrite(str(out_dir / f"{stamp}.jpg"), s['frame'])

    with open(args.csv, 'w') as f:
        f.write('exposure,gain,brightness,laplacian,orb,sat_pct,verdict\n')
        for r in rows:
            f.write(f'{r[0]},{r[1]},{r[2]:.1f},{r[3]:.1f},{int(r[4])},{r[5]:.1f},{r[6]}\n')
    print(f'\nSaved {args.csv}')
    if args.save_frames:
        print(f'Saved frames to {out_dir}/')

    ok = [r for r in rows if r[6] == 'OK']
    if ok:
        # Optimise: most ORB > lowest gain (low noise) > lowest sat
        ok.sort(key=lambda r: (-r[4], r[1], r[5]))
        print('\nTop 3 recommended (most ORB, lowest gain, lowest saturation):')
        for r in ok[:3]:
            print(f'  exp={r[0]:>5}  gain={r[1]:>3}  bright={r[2]:5.1f}  '
                  f'ORB={int(r[4]):>4}  sat={r[5]:4.1f}%')
        winner = ok[0]
        print(f'\nApply with:')
        print(f'  ros2 param set {node_name} rgb_camera.enable_auto_exposure false')
        print(f'  ros2 param set {node_name} rgb_camera.exposure {winner[0]}')
        print(f'  ros2 param set {node_name} rgb_camera.gain     {winner[1]}')
    else:
        nearest = min(rows, key=lambda r: abs(r[2] - 120) + (r[5] if r[5] > 5 else 0))
        print('\nNo (exp, gain) cell hit the OK targets. Suggestions:')
        print(f'  closest to brightness=120 with low clipping: '
              f'exp={nearest[0]} gain={nearest[1]} bright={nearest[2]:.1f}  '
              f'ORB={int(nearest[4])}  sat={nearest[5]:.1f}%  ({nearest[6]})')
        print('  consider: scene lighting, larger exposure range, or expand --gains')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
