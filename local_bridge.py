"""ST3215 Leader Arm WebSocket Bridge.

Reads joint angles from dual STS3215 servo arms via serial port and
broadcasts them over WebSocket at 50Hz. Designed for Windows + Linux.

Usage:
    pip install pyserial websockets
    python local_bridge.py
    python local_bridge.py --config bridge_config.json --port 7000
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import struct
import sys
import threading
import time
from typing import Any

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

try:
    import websockets
    from websockets.server import serve as ws_serve
except ImportError:
    print("ERROR: websockets is required. Install with: pip install websockets")
    sys.exit(1)


# ─── STS3215 Minimal Driver (embedded, no external dependency) ───────────────

STS_PRESENT_POSITION = 56
STS_TORQUE_ENABLE = 40
STS_INST_READ = 2
STS_INST_WRITE = 3
STS_RESOLUTION = 4096
STS_DEGREE_RANGE = 360.0


def _calc_checksum(packet: list[int]) -> int:
    return (~sum(packet)) & 0xFF


def _build_packet(servo_id: int, instruction: int, params: list[int]) -> bytes:
    length = len(params) + 2
    payload = [servo_id, length, instruction] + params
    checksum = _calc_checksum(payload)
    return bytes([0xFF, 0xFF] + payload + [checksum])


def _read_response(ser: serial.Serial, servo_id: int, timeout: float = 0.02) -> bytes | None:
    """Read and parse STS response packet. Returns param bytes or None."""
    header_count = 0
    start = time.perf_counter()
    while time.perf_counter() - start < timeout:
        byte = ser.read(1)
        if not byte:
            continue
        if byte[0] == 0xFF:
            header_count += 1
        else:
            header_count = 0
        if header_count >= 2:
            break
    else:
        return None

    meta = ser.read(2)
    if len(meta) < 2:
        return None
    resp_id, resp_len = meta[0], meta[1]
    if resp_id != servo_id or resp_len < 2:
        return None
    body = ser.read(resp_len)
    if len(body) != resp_len:
        return None
    # body = [error, param1..paramN, checksum]
    return bytes(body[1:-1])  # skip error byte and checksum


def sts_get_position(ser: serial.Serial, servo_id: int) -> int:
    """Read current position (0-4095) or -1 on failure."""
    ser.flushInput()
    packet = _build_packet(servo_id, STS_INST_READ, [STS_PRESENT_POSITION, 2])
    ser.write(packet)
    resp = _read_response(ser, servo_id)
    if resp and len(resp) == 2:
        return resp[0] | (resp[1] << 8)
    return -1


def sts_set_torque(ser: serial.Serial, servo_id: int, enable: bool) -> None:
    """Enable or disable torque."""
    packet = _build_packet(servo_id, STS_INST_WRITE, [STS_TORQUE_ENABLE, 1 if enable else 0])
    ser.write(packet)


# ─── Port Scanner ────────────────────────────────────────────────────────────

def find_port_by_serial(serial_number: str) -> str | None:
    """Find COM port by USB device serial number (works on Windows + Linux)."""
    for p in serial.tools.list_ports.comports():
        if p.serial_number == serial_number:
            return p.device
        # Fallback: match by location (for devices with duplicate serials)
        if p.location and serial_number == p.location:
            return p.device
    return None


def scan_available_ports() -> list[dict]:
    """List all serial ports with their info (for debugging)."""
    result = []
    for p in serial.tools.list_ports.comports():
        result.append({
            "device": p.device,
            "serial_number": p.serial_number or "",
            "location": p.location or "",
            "description": p.description or "",
        })
    return result


# ─── Arm Reader Thread ───────────────────────────────────────────────────────

class ArmReader:
    """Reads joint angles from a single leader arm in a background thread."""

    def __init__(self, side: str, config: dict):
        self.side = side
        self.serial_number: str = config.get("serial_number", "")
        self.servo_ids: list[int] = config.get("servo_ids", [1, 2, 3, 4, 5, 6, 7])
        self.home_offsets: dict[int, int] = {int(k): v for k, v in config.get("home_offsets", {}).items()}
        self.directions: dict[int, int] = {int(k): v for k, v in config.get("directions", {}).items()}
        self.baudrate: int = config.get("baudrate", 1000000)

        # Fill defaults
        for sid in self.servo_ids:
            self.home_offsets.setdefault(sid, 2048)
            self.directions.setdefault(sid, 1)

        self._ser: serial.Serial | None = None
        self._connected = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest_angles: list[float] | None = None
        self._thread: threading.Thread | None = None
        self._last_connect_attempt = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def latest_angles(self) -> list[float] | None:
        with self._lock:
            return self._latest_angles

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name=f"arm-{self.side}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._close_serial()

    def calibrate_home(self) -> bool:
        """Set current position as zero for all servos."""
        if not self._connected or not self._ser:
            return False
        with self._lock:
            for sid in self.servo_ids:
                pos = sts_get_position(self._ser, sid)
                if pos >= 0:
                    self.home_offsets[sid] = pos
            self._latest_angles = [0.0] * len(self.servo_ids)
        print(f"[{self.side}] Calibrated home: {self.home_offsets}")
        return True

    def set_torque(self, enable: bool) -> bool:
        if not self._connected or not self._ser:
            return False
        for sid in self.servo_ids:
            sts_set_torque(self._ser, sid, enable)
            time.sleep(0.003)
        print(f"[{self.side}] Torque {'ON' if enable else 'OFF'}")
        return True

    def get_config_dict(self) -> dict:
        """Return current config for saving."""
        return {
            "serial_number": self.serial_number,
            "servo_ids": self.servo_ids,
            "baudrate": self.baudrate,
            "home_offsets": {str(k): v for k, v in self.home_offsets.items()},
            "directions": {str(k): v for k, v in self.directions.items()},
        }

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self._connected:
                self._try_connect()
                if not self._connected:
                    self._stop.wait(3.0)
                    continue

            angles = self._read_angles()
            if angles is not None:
                with self._lock:
                    self._latest_angles = angles
            else:
                # Read failed — connection may be lost
                self._connected = False
                self._close_serial()
                print(f"[{self.side}] Connection lost, will retry...")

            # ~50Hz per arm (7 servos * ~2ms each ≈ 14ms + small sleep)
            time.sleep(0.002)

    def _try_connect(self) -> None:
        now = time.time()
        if now - self._last_connect_attempt < 3.0:
            return
        self._last_connect_attempt = now

        port = find_port_by_serial(self.serial_number)
        if not port:
            return

        try:
            self._ser = serial.Serial(port, self.baudrate, timeout=0.02)
            self._ser.flushInput()
            # Verify: try reading first servo
            pos = sts_get_position(self._ser, self.servo_ids[0])
            if pos < 0:
                self._close_serial()
                return
            self._connected = True
            # Disable torque by default (free-move for leader)
            for sid in self.servo_ids:
                sts_set_torque(self._ser, sid, False)
                time.sleep(0.003)
            print(f"[{self.side}] Connected: {port} (SN: {self.serial_number})")
        except Exception as e:
            print(f"[{self.side}] Connect failed ({port}): {e}")
            self._close_serial()

    def _read_angles(self) -> list[float] | None:
        """Read all servo positions and convert to calibrated degrees."""
        if not self._ser:
            return None
        angles = []
        for sid in self.servo_ids:
            raw = sts_get_position(self._ser, sid)
            if raw < 0:
                return None  # Any read failure → full retry
            offset = self.home_offsets.get(sid, 2048)
            delta = raw - offset
            # Wrap-around correction (STS3215 is 0-4095 continuous)
            if delta > 2048:
                delta -= 4096
            elif delta < -2048:
                delta += 4096
            deg = delta * (STS_DEGREE_RANGE / STS_RESOLUTION)
            direction = self.directions.get(sid, 1)
            angles.append(round(deg * direction, 2))
        return angles

    def _close_serial(self) -> None:
        self._connected = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


# ─── WebSocket Bridge Server ─────────────────────────────────────────────────

class BridgeServer:
    """Async WebSocket server that broadcasts joint states and accepts commands."""

    def __init__(self, arms: dict[str, ArmReader], port: int = 7000, hz: float = 50.0):
        self.arms = arms
        self.port = port
        self.hz = hz
        self._clients: set = set()
        self._config_path: str = ""

    async def run(self, config_path: str = "") -> None:
        self._config_path = config_path
        async with ws_serve(self._handle_client, "0.0.0.0", self.port, max_size=1024 * 1024):
            print(f"[bridge] WebSocket server listening on ws://0.0.0.0:{self.port}")
            print(f"[bridge] Connect from browser: ws://localhost:{self.port}")
            await self._broadcast_loop()

    async def _handle_client(self, ws) -> None:
        self._clients.add(ws)
        remote = ws.remote_address
        print(f"[bridge] Client connected: {remote}")
        try:
            async for message in ws:
                await self._handle_command(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.discard(ws)
            print(f"[bridge] Client disconnected: {remote}")

    async def _handle_command(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        cmd = msg.get("cmd")
        side = msg.get("side")

        if cmd == "calibrate":
            sides = [side] if side in self.arms else list(self.arms.keys())
            for s in sides:
                self.arms[s].calibrate_home()
            self._save_config()
        elif cmd == "torque":
            enable = bool(msg.get("enable", False))
            if side in self.arms:
                self.arms[side].set_torque(enable)
        elif cmd == "status":
            # Return current status to requester
            pass

    async def _broadcast_loop(self) -> None:
        interval = 1.0 / self.hz
        while True:
            start = time.perf_counter()
            state = self._build_state()
            if self._clients:
                payload = json.dumps(state, separators=(",", ":"))
                stale = []
                for ws in list(self._clients):
                    try:
                        await ws.send(payload)
                    except Exception:
                        stale.append(ws)
                for ws in stale:
                    self._clients.discard(ws)
            elapsed = time.perf_counter() - start
            sleep_time = max(0.001, interval - elapsed)
            await asyncio.sleep(sleep_time)

    def _build_state(self) -> dict:
        state: dict[str, Any] = {
            "type": "joint_state",
            "ts": round(time.time(), 3),
        }
        for side, arm in self.arms.items():
            state[side] = arm.latest_angles  # None if disconnected
        return state

    def _save_config(self) -> None:
        if not self._config_path:
            return
        config = load_config(self._config_path)
        for side, arm in self.arms.items():
            if side in config.get("arms", {}):
                config["arms"][side].update(arm.get_config_dict())
        try:
            with open(self._config_path, "w", encoding="utf-8") as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
            print(f"[bridge] Config saved to {self._config_path}")
        except Exception as e:
            print(f"[bridge] Failed to save config: {e}")


# ─── Config ──────────────────────────────────────────────────────────────────

DEFAULT_CONFIG = {
    "ws_port": 7000,
    "poll_hz": 50,
    "arms": {
        "left": {
            "serial_number": "",
            "servo_ids": [1, 2, 3, 4, 5, 6, 7],
            "baudrate": 1000000,
            "home_offsets": {},
            "directions": {},
        },
        "right": {
            "serial_number": "",
            "servo_ids": [1, 2, 3, 4, 5, 6, 7],
            "baudrate": 1000000,
            "home_offsets": {},
            "directions": {},
        },
    },
}


def load_config(path: str) -> dict:
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"[bridge] Failed to load config ({path}): {e}")
    return dict(DEFAULT_CONFIG)


def create_default_config(path: str) -> None:
    """Create a template config file with available ports listed."""
    config = dict(DEFAULT_CONFIG)
    ports = scan_available_ports()
    if ports:
        print("\n[bridge] Available serial ports:")
        for p in ports:
            print(f"  {p['device']}  SN={p['serial_number']}  Loc={p['location']}  ({p['description']})")
        print()
        # Auto-fill first two ports if they have serial numbers
        sn_ports = [p for p in ports if p["serial_number"]]
        if len(sn_ports) >= 1:
            config["arms"]["right"]["serial_number"] = sn_ports[0]["serial_number"]
        if len(sn_ports) >= 2:
            config["arms"]["left"]["serial_number"] = sn_ports[1]["serial_number"]
    with open(path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    print(f"[bridge] Created config template: {path}")
    print(f"[bridge] Edit it to set your arm serial numbers, then restart.")


# ─── Main ────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="ST3215 Leader Arm WebSocket Bridge")
    parser.add_argument("--config", default="bridge_config.json", help="Config file path")
    parser.add_argument("--port", type=int, default=0, help="WebSocket port (overrides config)")
    parser.add_argument("--init", action="store_true", help="Create default config and exit")
    parser.add_argument("--scan", action="store_true", help="Scan serial ports and exit")
    args = parser.parse_args()

    if args.scan:
        ports = scan_available_ports()
        print(f"\nFound {len(ports)} serial port(s):\n")
        for p in ports:
            print(f"  {p['device']:12s}  SN={p['serial_number']:16s}  Loc={p['location']:12s}  {p['description']}")
        return

    config_path = args.config
    if args.init or not os.path.exists(config_path):
        create_default_config(config_path)
        if args.init:
            return

    config = load_config(config_path)
    ws_port = args.port or config.get("ws_port", 7000)
    poll_hz = config.get("poll_hz", 50)

    print("=" * 60)
    print("  ST3215 Leader Arm WebSocket Bridge")
    print("=" * 60)

    # Create arm readers
    arms: dict[str, ArmReader] = {}
    for side, arm_cfg in config.get("arms", {}).items():
        sn = arm_cfg.get("serial_number", "")
        if not sn:
            print(f"[{side}] No serial_number configured, skipping")
            continue
        arms[side] = ArmReader(side, arm_cfg)

    if not arms:
        print("\n[bridge] No arms configured! Run with --init to create config.")
        print("[bridge] Or run with --scan to list available ports.")
        return

    # Start arm reader threads
    for arm in arms.values():
        arm.start()

    # Run WebSocket server
    server = BridgeServer(arms, port=ws_port, hz=poll_hz)
    try:
        asyncio.run(server.run(config_path=config_path))
    except KeyboardInterrupt:
        print("\n[bridge] Shutting down...")
    finally:
        for arm in arms.values():
            arm.stop()


if __name__ == "__main__":
    main()
