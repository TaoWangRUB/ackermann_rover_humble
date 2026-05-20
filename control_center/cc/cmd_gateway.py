"""CC-6b: Command Gateway — serialize and publish RoverCommand to MQTT."""
import logging
import time
import uuid
from typing import Any

from cc.event_bus import EventBus
from cc.mqtt_receiver import MQTTReceiver
from cc.safety_gate import SafetyGate

logger = logging.getLogger(__name__)

try:
    from proto import rover_health_pb2 as pb
except ImportError:
    pb = None

CMD_TYPE_MAP = {
    "nav_goal": 1,  # CMD_NAV_GOAL
    "arm": 2,       # CMD_ARM
    "disarm": 3,    # CMD_DISARM
    "set_mode": 4,  # CMD_SET_MODE
    "estop": 5,     # CMD_ESTOP
    "cancel_goal": 6,  # CMD_CANCEL_GOAL
    "set_param": 7,  # CMD_SET_PARAM
    "drive": 8,     # CMD_DRIVE
    "record": 9,    # CMD_RECORD
    "register_mode": 10,    # CMD_REGISTER_MODE
    "unregister_mode": 11,  # CMD_UNREGISTER_MODE
}

TOPIC_MAP = {
    "nav_goal": "rover/cmd/goal",
    "arm": "rover/cmd/arm",
    "disarm": "rover/cmd/disarm",
    "set_mode": "rover/cmd/mode",
    "estop": "rover/cmd/estop",
    "cancel_goal": "rover/cmd/cancel_goal",
    "drive": "rover/cmd/drive",
    "record": "rover/cmd/record",
    "register_mode": "rover/cmd/register_mode",
    "unregister_mode": "rover/cmd/unregister_mode",
}


class CommandGateway:
    """Validates, serializes, and publishes commands to MQTT."""

    def __init__(self, bus: EventBus, mqtt: MQTTReceiver,
                 safety: SafetyGate, cfg: dict[str, Any]) -> None:
        self._bus = bus
        self._mqtt = mqtt
        self._safety = safety
        self._qos = cfg.get("publish_qos", 2)
        self._topics = self._normalize_topics(cfg.get("topics", {}))
        self._command_log: list[dict[str, Any]] = []

    @staticmethod
    def _normalize_topics(configured_topics: dict[str, Any]) -> dict[str, str]:
        topics = dict(TOPIC_MAP)
        alias_map = {
            "goal": "nav_goal",
            "nav_goal": "nav_goal",
            "mode": "set_mode",
            "set_mode": "set_mode",
            "arm": "arm",
            "disarm": "disarm",
            "estop": "estop",
            "cancel_goal": "cancel_goal",
            "drive": "drive",
            "record": "record",
            "register_mode": "register_mode",
            "unregister_mode": "unregister_mode",
        }

        for key, value in configured_topics.items():
            canonical_key = alias_map.get(key)
            if canonical_key and isinstance(value, str):
                topics[canonical_key] = value

        return topics

    async def send_command(self, cmd_type: str, params: dict[str, Any],
                           issued_by: str = "dashboard") -> str:
        """Validate, serialize, and send a command. Returns cmd_id."""
        if pb is None:
            raise ValueError("Protobuf module not available")

        # Safety check
        allowed, reason = await self._safety.check(cmd_type, params)
        if not allowed:
            raise ValueError(f"Safety gate rejected: {reason}")

        cmd_id = str(uuid.uuid4())
        cmd = pb.RoverCommand()
        cmd.cmd_id = cmd_id
        cmd.cmd_type = CMD_TYPE_MAP.get(cmd_type, 0)
        cmd.issued_by = issued_by
        cmd.timestamp = int(time.time() * 1000)

        # Populate type-specific fields
        if cmd_type == "nav_goal":
            cmd.nav_goal.latitude = params.get("latitude", 0.0)
            cmd.nav_goal.longitude = params.get("longitude", 0.0)
            cmd.nav_goal.altitude = params.get("altitude", 0.0)
            cmd.nav_goal.yaw_deg = params.get("yaw_deg", 0.0)
        elif cmd_type == "set_mode":
            cmd.set_mode.mode_name = params.get("mode_name", "")
        elif cmd_type == "set_param":
            cmd.set_param.param_name = params.get("param_name", "")
            cmd.set_param.param_value = params.get("param_value", "")
        elif cmd_type == "drive":
            speed = float(params.get("speed_ms", 0.0))
            steering = float(params.get("steering", 0.0))
            if not -10.0 <= speed <= 10.0:
                raise ValueError(f"speed_ms out of range [-10, 10]: {speed}")
            if not -1.0 <= steering <= 1.0:
                raise ValueError(f"steering out of range [-1, 1]: {steering}")
            cmd.drive.speed_ms = speed
            cmd.drive.steering = steering
        elif cmd_type == "record":
            action = str(params.get("action", "toggle")).lower()
            if action not in ("start", "stop", "toggle"):
                raise ValueError(f"record action must be start|stop|toggle, got {action!r}")
            cmd.record.action = action
        elif cmd_type == "register_mode":
            cmd.register_mode.mode_name = params.get("mode_name", "")
        elif cmd_type == "unregister_mode":
            cmd.unregister_mode.mode_name = params.get("mode_name", "")

        # Serialize and publish
        payload = cmd.SerializeToString()
        topic = self._topics.get(cmd_type, f"rover/cmd/{cmd_type}")
        qos = 0 if cmd_type == "drive" else self._qos
        self._mqtt.publish(topic, payload, qos=qos)

        # Drive is high-frequency fire-and-forget — skip logging and ACK tracking
        if cmd_type == "drive":
            return cmd_id

        logger.info("Command sent: cmd_id=%s type=%s topic=%s by=%s",
                     cmd_id, cmd_type, topic, issued_by)

        # Log for history
        entry = {
            "cmd_id": cmd_id,
            "cmd_type": cmd_type,
            "issued_by": issued_by,
            "timestamp": cmd.timestamp,
            "params": params,
            "status": "sent",
        }
        self._command_log.append(entry)

        # Notify ACK tracker
        await self._bus.emit("evt.cmd_sent", cmd_id=cmd_id, cmd_type=cmd_type,
                             timestamp=cmd.timestamp)

        return cmd_id

    def get_log(self) -> list[dict[str, Any]]:
        return list(self._command_log)
