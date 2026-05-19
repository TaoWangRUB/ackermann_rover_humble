#!/usr/bin/env python3
"""Subscribe to a color image and print live image-quality metrics.

Run inside the container:
  docker exec -it jazzy_slam_x86_64 bash -lc \\
    "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \\
     python3 /workspace/scripts/watch_image_quality.py /d435i/color/image_raw"

Usage:
  watch_image_quality.py [TOPIC]
    TOPIC: default /d435i/color/image_raw

Metrics:
  bright:    mean grayscale (target ~120/255)
  lap:       Laplacian variance (sharpness; >100 is sharp, <30 is blurry)
  ORB:       ORB feature count (target >500 for reliable loop closure)
  sat%:      fraction of pixels >= 250 (clipping)
"""
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


class IQWatcher(Node):
    def __init__(self, topic: str):
        super().__init__('image_quality_watcher')
        self._orb = cv2.ORB_create(nfeatures=2000)
        self._last_print = 0.0
        self.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f'subscribed to {topic} (1 Hz print)')

    def _cb(self, msg: Image):
        now = time.monotonic()
        if now - self._last_print < 1.0:
            return
        self._last_print = now

        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
        if arr.shape[2] == 3:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == 'rgb8' else arr
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        else:
            gray = arr[:, :, 0]

        bright = float(gray.mean())
        lap = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        orb_count = len(self._orb.detect(gray, None))
        sat_pct = 100.0 * (gray >= 250).mean()
        dark_pct = 100.0 * (gray <= 5).mean()

        flags = []
        if bright < 50:    flags.append('DARK')
        if bright > 200:   flags.append('OVEREXP')
        if lap < 60:       flags.append('BLUR')
        if orb_count < 200:flags.append('LOW_TEX')
        verdict = ','.join(flags) if flags else 'ok'

        print(f'bright={bright:5.1f}  lap={lap:6.1f}  ORB={orb_count:4d}  '
              f'sat={sat_pct:4.1f}%  dark={dark_pct:4.1f}%  {verdict}', flush=True)


def main():
    rclpy.init()
    topic = sys.argv[1] if len(sys.argv) > 1 else '/d435i/color/image_raw'
    node = IQWatcher(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
