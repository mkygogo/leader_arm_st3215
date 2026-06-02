"""Remote follower arm server for StereoSpatial leader-arm teleoperation.

The server connects to scene-relay as a publisher in ``robot-scene`` and:

- receives ``leader_joint_command`` / teleop safety commands from viewers,
- sends safe actions to one or two MKRobot follower arms,
- publishes ``remote_joint_state`` and ``robot_server_status`` feedback.

Run with ``--mock`` when hardware is unavailable; mock mode exercises the
WebSocket protocol and safety state without opening DM CAN serial ports.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import websockets
from websockets.exceptions import ConnectionClosed


LOGGER = logging.getLogger("robot_server")
DEFAULT_CONFIG_PATH = "robot_server_config.json"
DEFAULT_CONFIG = {
    "relay_url": "ws://127.0.0.1:8190",
    "room": "robot-scene",
    "display_name": "RobotServer",
    "control_hz": 50,
    "status_hz": 20,
    "command_timeout_ms": 250,
    "arms": {
        "left": {
            "enabled": False,
            "port": "",
            "device_id": "1-9.1:1.0",
            "direction": [1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0],
            "gripper_open_deg": 50.0,
            "gripper_close_deg": 0.0,
            "joint_velocity_scaling": 0.15,
        },
        "right": {
            "enabled": True,
            "port": "",
            "device_id": "1-7:1.0",
            "direction": [1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0],
            "gripper_open_deg": 50.0,
            "gripper_close_deg": 0.0,
            "joint_velocity_scaling": 0.15,
        },
    },
}


def now_s() -> float:
    return time.time()


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def find_port(device_id: str) -> str | None:
    """Resolve a serial port by device path, serial number, or USB location."""
    if not device_id:
        return None
    candidate = Path(device_id)
    if candidate.exists():
        return str(candidate)
    import serial.tools.list_ports

    for port in serial.tools.list_ports.comports():
        if device_id in (port.device, port.serial_number or "", port.location or ""):
            return port.device
    return None


@dataclass
class ArmConfig:
    enabled: bool = False
    port: str = ""
    device_id: str = ""
    direction: list[float] = field(default_factory=lambda: [1.0] * 7)
    gripper_open_deg: float = 50.0
    gripper_close_deg: float = 0.0
    joint_velocity_scaling: float = 0.15

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ArmConfig":
        direction = value.get("direction", [1.0] * 7)
        if not isinstance(direction, list) or len(direction) != 7:
            direction = [1.0] * 7
        return cls(
            enabled=bool(value.get("enabled", False)),
            port=str(value.get("port", "") or ""),
            device_id=str(value.get("device_id", "") or ""),
            direction=[float(v) for v in direction],
            gripper_open_deg=float(value.get("gripper_open_deg", 50.0)),
            gripper_close_deg=float(value.get("gripper_close_deg", 0.0)),
            joint_velocity_scaling=float(value.get("joint_velocity_scaling", 0.15)),
        )


@dataclass
class ServerConfig:
    relay_url: str = "ws://127.0.0.1:8190"
    room: str = "robot-scene"
    display_name: str = "RobotServer"
    control_hz: float = 50.0
    status_hz: float = 20.0
    command_timeout_ms: int = 250
    arms: dict[str, ArmConfig] = field(default_factory=dict)

    @classmethod
    def load(cls, path: str) -> "ServerConfig":
        data: dict[str, Any] = {}
        if Path(path).exists():
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        arms_data = data.get("arms", {})
        arms = {
            "left": ArmConfig.from_dict(arms_data.get("left", {})),
            "right": ArmConfig.from_dict(arms_data.get("right", {})),
        }
        return cls(
            relay_url=str(data.get("relay_url", cls.relay_url)),
            room=str(data.get("room", cls.room)),
            display_name=str(data.get("display_name", cls.display_name)),
            control_hz=float(data.get("control_hz", cls.control_hz)),
            status_hz=float(data.get("status_hz", cls.status_hz)),
            command_timeout_ms=int(data.get("command_timeout_ms", cls.command_timeout_ms)),
            arms=arms,
        )


class FollowerArm:
    def __init__(self, side: str, config: ArmConfig, mock: bool = False) -> None:
        self.side = side
        self.config = config
        self.mock = mock
        self.robot: Any | None = None
        self.connected = False
        self.error = ""
        self._mock_state = [0.0] * 7

    def connect(self) -> None:
        if not self.config.enabled:
            self.error = "disabled in config"
            return
        if self.mock:
            self.connected = True
            self.error = ""
            LOGGER.info("[%s] mock follower enabled", self.side)
            return
        port = self.config.port or find_port(self.config.device_id)
        if not port:
            self.error = f"port not found for device_id={self.config.device_id!r}"
            LOGGER.warning("[%s] %s", self.side, self.error)
            return
        try:
            from mk_driver import MKRobotStandalone

            self.robot = MKRobotStandalone(port=port, joint_velocity_scaling=self.config.joint_velocity_scaling)
            self.robot.connect()
            self.connected = True
            self.error = ""
            LOGGER.info("[%s] follower connected on %s", self.side, port)
        except Exception as exc:
            self.error = str(exc)
            self.connected = False
            LOGGER.exception("[%s] follower connect failed", self.side)

    def close(self) -> None:
        if self.robot is not None:
            try:
                self.robot.close()
            except Exception:
                LOGGER.exception("[%s] follower close failed", self.side)
        self.robot = None
        self.connected = False

    def command_from_degrees(self, joints_deg: list[float]) -> list[float]:
        if len(joints_deg) != 7:
            raise ValueError(f"{self.side} command must contain 7 joints")
        joints = [0.0] * 7
        for idx in range(6):
            joints[idx] = math.radians(float(joints_deg[idx]))
        span = self.config.gripper_close_deg - self.config.gripper_open_deg
        if abs(span) < 1e-6:
            joints[6] = 0.0
        else:
            joints[6] = clamp((float(joints_deg[6]) - self.config.gripper_open_deg) / span, 0.0, 1.0)
        return [float(value) * float(self.config.direction[idx]) for idx, value in enumerate(joints)]

    def send_action(self, action: list[float]) -> None:
        if not self.connected:
            return
        if self.mock:
            self._mock_state = [float(v) for v in action]
            return
        assert self.robot is not None
        self.robot.send_action(action)

    def read_state(self) -> list[float] | None:
        if not self.connected:
            return None
        if self.mock:
            return [round(float(v), 5) for v in self._mock_state]
        assert self.robot is not None
        obs = self.robot.get_observation()
        state = obs.get("state") if isinstance(obs, dict) else None
        if state is None:
            return None
        return [round(float(v), 5) for v in state]


class RobotServer:
    def __init__(self, config: ServerConfig, mock: bool = False) -> None:
        self.config = config
        self.mock = mock
        self.arms = {side: FollowerArm(side, arm_config, mock=mock) for side, arm_config in config.arms.items()}
        self.teleop_enabled = False
        self.estopped = False
        self.mode = "idle"
        self.error = ""
        self.last_command_at = 0.0
        self.last_command: dict[str, Any] | None = None

    def connect_arms(self) -> None:
        for arm in self.arms.values():
            arm.connect()

    def close(self) -> None:
        for arm in self.arms.values():
            arm.close()

    def handle_message(self, message: dict[str, Any]) -> None:
        msg_type = message.get("type")
        if msg_type == "leader_joint_command":
            self.last_command = message
            self.last_command_at = now_s()
            return
        if msg_type == "teleop_enable":
            if self.estopped:
                self.error = "cannot enable while emergency stop is active"
                return
            self.teleop_enabled = True
            self.mode = "teleop"
            self.error = ""
            return
        if msg_type == "teleop_disable":
            self.teleop_enabled = False
            self.mode = "idle"
            return
        if msg_type == "emergency_stop":
            self.teleop_enabled = False
            self.estopped = True
            self.mode = "estop"
            self.error = "emergency stop"
            return
        if msg_type == "clear_estop":
            self.estopped = False
            self.mode = "idle"
            self.error = ""

    def command_is_fresh(self) -> bool:
        if self.last_command_at <= 0:
            return False
        return (now_s() - self.last_command_at) * 1000.0 <= self.config.command_timeout_ms

    def apply_latest_command(self) -> None:
        if not self.teleop_enabled or self.estopped or self.last_command is None:
            return
        if not self.command_is_fresh():
            self.mode = "stale"
            return
        self.mode = "teleop"
        self.error = ""
        for side in ("left", "right"):
            values = self.last_command.get(side)
            if not isinstance(values, list):
                continue
            arm = self.arms.get(side)
            if arm is None or not arm.connected:
                continue
            try:
                arm.send_action(arm.command_from_degrees(values))
            except Exception as exc:
                self.error = f"{side}: {exc}"
                LOGGER.exception("failed to apply %s command", side)

    def joint_state_payload(self) -> dict[str, Any]:
        return {
            "type": "remote_joint_state",
            "ts": now_s(),
            "left": self.arms["left"].read_state() if "left" in self.arms else None,
            "right": self.arms["right"].read_state() if "right" in self.arms else None,
            "units": "rad",
            "enabled": self.teleop_enabled and not self.estopped,
        }

    def status_payload(self) -> dict[str, Any]:
        connected = any(arm.connected for arm in self.arms.values())
        arm_errors = {side: arm.error for side, arm in self.arms.items() if arm.error}
        error = self.error or "; ".join(f"{side}: {err}" for side, err in arm_errors.items())
        return {
            "type": "robot_server_status",
            "ts": now_s(),
            "connected": connected,
            "enabled": self.teleop_enabled and not self.estopped,
            "mode": self.mode,
            "error": error,
            "command_fresh": self.command_is_fresh(),
            "arms": {
                side: {"connected": arm.connected, "error": arm.error}
                for side, arm in self.arms.items()
            },
        }

    async def run(self) -> None:
        self.connect_arms()
        control_interval = 1.0 / max(self.config.control_hz, 1.0)
        status_interval = 1.0 / max(self.config.status_hz, 1.0)
        while True:
            try:
                async with websockets.connect(self.config.relay_url, max_size=None, ping_interval=20, ping_timeout=20) as ws:
                    await ws.send(json.dumps({
                        "room": self.config.room,
                        "role": "publisher",
                        "displayName": self.config.display_name,
                    }))
                    LOGGER.info("connected to scene-relay %s room=%s", self.config.relay_url, self.config.room)
                    await self._run_connected(ws, control_interval, status_interval)
            except (ConnectionClosed, OSError, ConnectionRefusedError) as exc:
                LOGGER.warning("relay connection lost (%s), reconnecting in 2s", exc)
                await asyncio.sleep(2.0)

    async def _run_connected(self, ws: Any, control_interval: float, status_interval: float) -> None:
        last_control = 0.0
        last_status = 0.0
        while True:
            now = time.monotonic()
            if now - last_control >= control_interval:
                self.apply_latest_command()
                last_control = now
            if now - last_status >= status_interval:
                await ws.send(json.dumps(self.joint_state_payload(), separators=(",", ":")))
                await ws.send(json.dumps(self.status_payload(), separators=(",", ":")))
                last_status = now
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=0.001)
            except asyncio.TimeoutError:
                await asyncio.sleep(0.001)
                continue
            if isinstance(raw, bytes):
                continue
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if isinstance(msg, dict):
                self.handle_message(msg)


def create_default_config(path: str) -> None:
    target = Path(path)
    if target.exists():
        return
    target.write_text(json.dumps(DEFAULT_CONFIG, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Remote MKRobot follower server for scene-relay teleoperation")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--relay-url", default=None)
    parser.add_argument("--room", default=None)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--init", action="store_true", help="Create a default config and exit")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if args.init:
        create_default_config(args.config)
        print(f"created {args.config}")
        return
    config = ServerConfig.load(args.config)
    if args.relay_url:
        config.relay_url = args.relay_url
    if args.room:
        config.room = args.room
    server = RobotServer(config, mock=args.mock)
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        LOGGER.info("shutting down")
    finally:
        server.close()


if __name__ == "__main__":
    main()
