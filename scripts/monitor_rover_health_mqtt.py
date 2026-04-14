#!/usr/bin/env python3
"""Subscribe to rover health telemetry over MQTT and print decoded summaries."""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import paho.mqtt.client as mqtt


PROJECT_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from control_center.proto import rover_health_pb2  # noqa: E402


def format_msg(msg: rover_health_pb2.RoverHealth) -> str:
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


def build_client(client_id: str) -> mqtt.Client:
    callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api_version is not None:
        return mqtt.Client(
            callback_api_version=callback_api_version.VERSION1,
            client_id=client_id,
        )
    return mqtt.Client(client_id=client_id)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--topic", default="rover/health/#")
    parser.add_argument("--client-id", default="host-rover-health-monitor")
    args = parser.parse_args()

    client = build_client(args.client_id)
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    def on_connect(_client, _userdata, _flags, reason_code):
        print(f"Connected to MQTT broker {args.host}:{args.port} rc={reason_code}", flush=True)
        _client.subscribe(args.topic)

    def on_disconnect(_client, _userdata, reason_code):
        print(f"Disconnected from MQTT broker rc={reason_code}", flush=True)

    def on_message(_client, _userdata, msg):
        telemetry = rover_health_pb2.RoverHealth()
        try:
            telemetry.ParseFromString(msg.payload)
        except Exception as exc:
            print(f"decode error on {msg.topic}: {exc}", file=sys.stderr, flush=True)
            return
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] {msg.topic} {format_msg(telemetry)}", flush=True)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    print(f"Subscribing to {args.topic} on {args.host}:{args.port}...", flush=True)
    client.connect(args.host, args.port, keepalive=30)

    try:
        client.loop_forever(retry_first_connection=True)
    except KeyboardInterrupt:
        print("Stopping MQTT monitor", flush=True)
    finally:
        try:
            client.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())