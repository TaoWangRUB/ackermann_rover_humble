#!/usr/bin/env python3
"""Create the 'Rover Health Monitor' dashboard in InfluxDB 2.x.

Idempotent — skips creation if a dashboard with the same name already exists.
Run once after `docker compose up -d` to provision the default dashboard.

Usage:
    python3 control_center/scripts/setup_influxdb_dashboard.py
    # or with custom URL/token:
    INFLUX_URL=http://localhost:8086 INFLUX_TOKEN=rover-dev-token python3 ...
"""
import json
import os
import sys
import time
import urllib.error
import urllib.request

URL = os.environ.get("INFLUX_URL", "http://localhost:8086")
TOKEN = os.environ.get("INFLUX_TOKEN", "rover-dev-token")
ORG = os.environ.get("INFLUX_ORG", "rover")
DASHBOARD_NAME = "Rover Health Monitor"

HEADERS = {
    "Authorization": f"Token {TOKEN}",
    "Content-Type": "application/json",
}


def api(method, path, body=None):
    data = json.dumps(body).encode() if body else None
    req = urllib.request.Request(
        f"{URL}{path}", data=data, method=method, headers=HEADERS)
    resp = urllib.request.urlopen(req)
    return json.loads(resp.read()) if resp.status == 200 or resp.status == 201 else {}


def get_org_id():
    d = api("GET", "/api/v2/orgs")
    for org in d.get("orgs", []):
        if org["name"] == ORG:
            return org["id"]
    sys.exit(f"Org '{ORG}' not found")


def dashboard_exists():
    d = api("GET", "/api/v2/dashboards")
    for dash in d.get("dashboards", []):
        if dash["name"] == DASHBOARD_NAME:
            return dash["id"]
    return None


def create_dashboard(org_id):
    d = api("POST", "/api/v2/dashboards", {"name": DASHBOARD_NAME, "orgID": org_id})
    return d["id"]


def add_cell(dash_id, x, y, w, h, name):
    d = api("POST", f"/api/v2/dashboards/{dash_id}/cells",
            {"x": x, "y": y, "w": w, "h": h, "name": name})
    return d["id"]


def set_xy_view(dash_id, cell_id, name, query,
                y_bounds=None, y_label="", legend=False):
    props = {
        "type": "xy",
        "queries": [{
            "text": query,
            "editMode": "advanced",
            "name": "",
            "builderConfig": {
                "buckets": [], "tags": [], "functions": [],
                "aggregateWindow": {"period": "auto"},
            },
        }],
        "axes": {
            "x": {"bounds": ["", ""], "label": "", "prefix": "", "suffix": "",
                   "base": "10", "scale": "linear"},
            "y": {"bounds": y_bounds or ["", ""], "label": y_label,
                   "prefix": "", "suffix": "", "base": "10", "scale": "linear"},
        },
        "colors": [{"id": "base", "type": "scale", "hex": "#31C0F6",
                     "name": "Nineteen Eighty Four", "value": 0}],
        "shape": "chronograf-v2",
        "note": "",
        "showNoteWhenEmpty": False,
        "position": "overlaid",
        "staticLegend": {
            "colorizeRows": True,
            "heightRatio": 0.3,
            "show": legend,
            "widthRatio": 1.0,
        } if legend else {},
        "legendColorizeRows": legend,
        "legendHide": not legend,
        "legendOpacity": 1.0 if legend else 0,
        "legendOrientationThreshold": 5 if legend else 0,
    }
    api("PATCH", f"/api/v2/dashboards/{dash_id}/cells/{cell_id}/view",
        {"name": name, "properties": props})


def set_table_view(dash_id, cell_id, name, query):
    props = {
        "type": "table",
        "queries": [{
            "text": query,
            "editMode": "advanced",
            "name": "",
            "builderConfig": {
                "buckets": [], "tags": [], "functions": [],
                "aggregateWindow": {"period": "auto"},
            },
        }],
        "colors": [{"id": "base", "type": "text", "hex": "#31C0F6",
                     "name": "Nineteen Eighty Four", "value": 0}],
        "shape": "chronograf-v2",
        "note": "",
        "showNoteWhenEmpty": False,
        "tableOptions": {
            "verticalTimeAxis": True,
            "sortBy": {"internalName": "count", "displayName": "count",
                        "visible": True},
            "fixFirstColumn": False,
        },
        "fieldOptions": [
            {"internalName": "alert_id", "displayName": "Alert", "visible": True},
            {"internalName": "count", "displayName": "Count", "visible": True},
        ],
        "decimalPlaces": {"isEnforced": False, "digits": 0},
        "timeFormat": "",
    }
    api("PATCH", f"/api/v2/dashboards/{dash_id}/cells/{cell_id}/view",
        {"name": name, "properties": props})


# ── Flux queries ──────────────────────────────────────────────────────

Q_BATTERY_PCT = (
    'from(bucket: "rover_telemetry")'
    " |> range(start: v.timeRangeStart, stop: v.timeRangeStop)"
    ' |> filter(fn: (r) => r._measurement == "rover_telemetry")'
    ' |> filter(fn: (r) => r._field == "px4_battery_pct")'
    " |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)"
    ' |> yield(name: "mean")'
)

Q_BATTERY_V = (
    'from(bucket: "rover_telemetry")'
    " |> range(start: v.timeRangeStart, stop: v.timeRangeStop)"
    ' |> filter(fn: (r) => r._measurement == "rover_telemetry")'
    ' |> filter(fn: (r) => r._field == "px4_battery_v")'
    " |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)"
    ' |> yield(name: "mean")'
)

Q_CAM_FPS = (
    'from(bucket: "rover_telemetry")'
    " |> range(start: v.timeRangeStart, stop: v.timeRangeStop)"
    ' |> filter(fn: (r) => r._measurement == "rover_telemetry")'
    ' |> filter(fn: (r) => r._field == "cam_depth_fps")'
    " |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)"
    ' |> yield(name: "mean")'
)

Q_JETSON_TEMPS = (
    'from(bucket: "rover_telemetry")'
    " |> range(start: v.timeRangeStart, stop: v.timeRangeStop)"
    ' |> filter(fn: (r) => r._measurement == "rover_telemetry")'
    ' |> filter(fn: (r) => r._field == "jetson_temp_cpu_c"'
    ' or r._field == "jetson_temp_gpu_c")'
    " |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)"
    ' |> yield(name: "mean")'
)

Q_JETSON_GPU = (
    'from(bucket: "rover_telemetry")'
    " |> range(start: v.timeRangeStart, stop: v.timeRangeStop)"
    ' |> filter(fn: (r) => r._measurement == "rover_telemetry")'
    ' |> filter(fn: (r) => r._field == "jetson_gpu_pct")'
    " |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)"
    ' |> yield(name: "mean")'
)

Q_ALERTS = (
    'from(bucket: "rover_telemetry")'
    " |> range(start: v.timeRangeStart, stop: v.timeRangeStop)"
    ' |> filter(fn: (r) => r._measurement == "rover_alerts")'
    ' |> filter(fn: (r) => r._field == "severity")'
    ' |> group(columns: ["alert_id", "_field"])'
    " |> count()"
    ' |> rename(columns: {"_value": "count"})'
    " |> group()"
    ' |> sort(columns: ["count"], desc: true)'
    ' |> keep(columns: ["alert_id", "count"])'
)


# ── Main ──────────────────────────────────────────────────────────────

def main():
    # Wait for InfluxDB to be ready (up to 30s)
    for i in range(30):
        try:
            req = urllib.request.Request(f"{URL}/health")
            resp = urllib.request.urlopen(req)
            health = json.loads(resp.read())
            if health.get("status") == "pass":
                break
        except (urllib.error.URLError, ConnectionRefusedError):
            pass
        if i == 29:
            sys.exit("InfluxDB not ready after 30s")
        time.sleep(1)

    # Check if dashboard already exists
    existing = dashboard_exists()
    if existing:
        print(f"Dashboard '{DASHBOARD_NAME}' already exists (ID: {existing}) — skipping.")
        return

    org_id = get_org_id()
    dash_id = create_dashboard(org_id)
    print(f"Created dashboard '{DASHBOARD_NAME}' (ID: {dash_id})")

    # Row 1: PX4 Battery %, PX4 Battery Voltage, Camera Depth FPS
    c1 = add_cell(dash_id, 0, 0, 4, 3, "PX4 Battery %")
    c2 = add_cell(dash_id, 4, 0, 4, 3, "PX4 Battery Voltage")
    c3 = add_cell(dash_id, 8, 0, 4, 3, "Camera Depth FPS")

    # Row 2: Jetson Temperatures, Jetson GPU %, Active Alerts
    c4 = add_cell(dash_id, 0, 3, 4, 3, "Jetson Temperatures (°C)")
    c5 = add_cell(dash_id, 4, 3, 4, 3, "Jetson GPU %")
    c6 = add_cell(dash_id, 8, 3, 4, 3, "Active Alerts")

    # Configure views
    set_xy_view(dash_id, c1, "PX4 Battery %", Q_BATTERY_PCT)
    set_xy_view(dash_id, c2, "PX4 Battery Voltage", Q_BATTERY_V)
    set_xy_view(dash_id, c3, "Camera Depth FPS", Q_CAM_FPS)
    set_xy_view(dash_id, c4, "Jetson Temperatures (°C)", Q_JETSON_TEMPS,
                y_bounds=["30", "80"], y_label="°C", legend=True)
    set_xy_view(dash_id, c5, "Jetson GPU %", Q_JETSON_GPU)
    set_table_view(dash_id, c6, "Active Alerts", Q_ALERTS)

    print("Dashboard provisioned with 6 panels.")


if __name__ == "__main__":
    main()
