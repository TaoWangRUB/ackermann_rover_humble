#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

run_local_monitor() {
    exec python3 "${PROJECT_DIR}/scripts/monitor_rover_health_mqtt.py" --host localhost
}

find_control_center_container() {
    docker ps \
        --filter label=com.docker.compose.service=control_center \
        --format '{{.Names}}' \
        | head -n 1
}

run_container_monitor() {
    local container_name
    container_name="$(find_control_center_container)"
    if [[ -z "${container_name}" ]]; then
        echo "control_center container is not running" >&2
        exit 1
    fi

    exec docker exec -i "${container_name}" python - <<'PY'
import sys
import time

import paho.mqtt.client as mqtt
from proto import rover_health_pb2


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


client = mqtt.Client(client_id="host-rover-health-monitor")
client.reconnect_delay_set(min_delay=1, max_delay=30)


def on_connect(client, _userdata, _flags, reason_code):
    print(f"Connected to MQTT broker localhost:1883 rc={reason_code}", flush=True)
    client.subscribe("rover/health/#")


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

print("Subscribing to rover/health/# on localhost:1883 via control_center container...", flush=True)
client.connect("localhost", 1883, keepalive=30)
client.loop_forever(retry_first_connection=True)
PY
}

echo '--- MQTT monitor (rover/health/#) ---'

if python3 -c 'import paho.mqtt.client, google.protobuf' >/dev/null 2>&1; then
    run_local_monitor
fi

run_container_monitor