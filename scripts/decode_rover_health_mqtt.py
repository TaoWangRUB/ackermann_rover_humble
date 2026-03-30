#!/usr/bin/env python3
"""Decode Protobuf RoverHealth messages from mosquitto_sub.

Works in two modes:
  - One-shot: mosquitto_sub -C 1 | python3 decode_rover_health_mqtt.py
  - Streaming: mosquitto_sub | python3 decode_rover_health_mqtt.py
"""

import sys


def decode_one(rover_health_pb2, data):
    """Try to decode a single RoverHealth message from binary data."""
    if not data:
        return
    if data.endswith(b"\n"):
        data = data[:-1]
    if not data:
        return

    msg = rover_health_pb2.RoverHealth()
    try:
        msg.ParseFromString(data)
    except Exception as exc:
        print(f"decode error: {exc}", file=sys.stderr, flush=True)
        return

    print(
        "seq={seq} overall={overall} cam={cam} fps={fps:.1f} px4={px4} "
        "armed={armed} mode={mode} batt={batt:.1f} alerts={alerts}".format(
            seq=msg.seq,
            overall=msg.overall_health,
            cam=msg.camera.connected,
            fps=msg.camera.depth_fps,
            px4=msg.px4.connected,
            armed=msg.px4.armed,
            mode=msg.px4.nav_state_label,
            batt=msg.px4.battery_remaining_pct,
            alerts=list(msg.active_alerts),
        ),
        flush=True,
    )


def main():
    sys.path.insert(0, "/tmp")

    try:
        import rover_health_pb2
    except ImportError as exc:
        print(f"failed to import rover_health_pb2: {exc}", file=sys.stderr, flush=True)
        return 1

    # Read line-by-line for streaming mode (mosquitto_sub outputs one
    # message per line).  Falls back to read-all for piped one-shot mode.
    try:
        for line in sys.stdin.buffer:
            decode_one(rover_health_pb2, line)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
