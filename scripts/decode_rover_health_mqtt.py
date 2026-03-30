#!/usr/bin/env python3
"""Decode Protobuf RoverHealth messages from mosquitto_sub.

Usage (streaming, recommended):
  mosquitto_sub -h HOST -t 'rover/health/#' -F '%x' | python3 decode_rover_health_mqtt.py --hex

Usage (one-shot, legacy):
  mosquitto_sub -h HOST -t 'rover/health/#' -C 1 | python3 decode_rover_health_mqtt.py
"""

import sys


def format_msg(msg):
    return (
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
        )
    )


def decode(rover_health_pb2, data):
    if not data:
        return
    msg = rover_health_pb2.RoverHealth()
    try:
        msg.ParseFromString(data)
    except Exception as exc:
        print(f"decode error: {exc}", file=sys.stderr, flush=True)
        return
    print(format_msg(msg), flush=True)


def main():
    sys.path.insert(0, "/tmp")

    try:
        import rover_health_pb2
    except ImportError as exc:
        print(f"failed to import rover_health_pb2: {exc}", file=sys.stderr, flush=True)
        return 1

    hex_mode = "--hex" in sys.argv

    if hex_mode:
        # Streaming: each line is a hex-encoded protobuf payload
        try:
            for line in sys.stdin:
                line = line.strip()
                if not line:
                    continue
                try:
                    data = bytes.fromhex(line)
                except ValueError as exc:
                    print(f"hex decode error: {exc}", file=sys.stderr, flush=True)
                    continue
                decode(rover_health_pb2, data)
        except KeyboardInterrupt:
            pass
    else:
        # One-shot: read raw binary from stdin until EOF
        data = sys.stdin.buffer.read()
        if data and data.endswith(b"\n"):
            data = data[:-1]
        if not data:
            print("no MQTT payload received", file=sys.stderr, flush=True)
            return 1
        decode(rover_health_pb2, data)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
