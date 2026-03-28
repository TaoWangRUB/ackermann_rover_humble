#!/usr/bin/env python3

import sys


def main() -> int:
    sys.path.insert(0, "/tmp")

    try:
        import rover_health_pb2
    except ImportError as exc:
        print(f"failed to import rover_health_pb2: {exc}", file=sys.stderr, flush=True)
        return 1

    data = sys.stdin.buffer.read()
    if not data:
        print("no MQTT payload received", file=sys.stderr, flush=True)
        return 1
    if data.endswith(b"\n"):
        data = data[:-1]
    if not data:
        print("empty MQTT payload received", file=sys.stderr, flush=True)
        return 1

    msg = rover_health_pb2.RoverHealth()
    try:
        msg.ParseFromString(data)
    except Exception as exc:
        print(f"failed to decode RoverHealth payload: {exc}", file=sys.stderr, flush=True)
        return 1

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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
