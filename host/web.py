"""
FastHTML bridge between a browser UI and the tmc2209-fw firmware.

Usage:
    pixi run web [PORT]

Defaults serial to /dev/ttyACM0 and listens on http://127.0.0.1:8000.
The browser UI carries all the controls the TUI has (jog, enable, TMC
config, set-position, servo target+config, vmax/amax, port reopen) plus
an optional URDF-based IK panel that calls into host/ik.py.
"""

from __future__ import annotations

import os
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial
import uvicorn
from fasthtml.common import FastHTML, HTMLResponse, JSONResponse, Request

DEFAULT_PORT = "/dev/ttyACM0"
HTTP_HOST = "127.0.0.1"
HTTP_PORT = 8000
NUM_JOINTS = 5
NUM_SERVOS = 2

# Joints that have UART-controllable TMC2209 drivers. With two UART buses
# (UART1 for J0..J3, UART0 for J4) all five joints are addressable.
TMC_UART_JOINTS = (0, 1, 2, 3, 4)

# StateReport size: 1 tag + N_J * i32 + N_S * u16 + 1 flags.
STATE_LEN = 1 + NUM_JOINTS * 4 + NUM_SERVOS * 2 + 1

# Servo flag bits (must match src/servo.rs).
SERVO_FLAG_HOMING = 1 << 0

INDEX_PATH = Path(__file__).with_name("index.html")


# == protocol helpers ======================================================

def cmd_enable(mask: int) -> bytes:
    return struct.pack("<BB", 0x02, mask & 0xFF)


def cmd_set_target(joint: int, position: int, vmax: int, amax: int) -> bytes:
    return struct.pack("<BBiII", 0x01, joint, position, vmax, amax)


def cmd_get_state() -> bytes:
    return b"\x04"


def cmd_set_tmc_config(joint: int, flags: int) -> bytes:
    return struct.pack("<BBB", 0x05, joint, flags & 0xFF)


def cmd_set_position(joint: int, position: int) -> bytes:
    return struct.pack("<BBi", 0x06, joint, position)


def cmd_get_tmc_config(joint: int) -> bytes:
    return struct.pack("<BB", 0x07, joint)


def cmd_get_version() -> bytes:
    return b"\x08"


def cmd_set_servo_target(servo: int, target_us: int) -> bytes:
    return struct.pack("<BBH", 0x09, servo, target_us & 0xFFFF)


def cmd_set_servo_config(servo: int, cfg: "ServoConfig") -> bytes:
    return struct.pack(
        "<BBHHHHHB", 0x0A, servo,
        cfg.min_us, cfg.max_us, cfg.deadzone_us, cfg.speed_us_per_s,
        cfg.home_us, cfg.flags & 0xFF,
    )


VERSION_LEN = 18
CHIP_NAMES = {0x40: "rp2040", 0x50: "rp2350"}


def parse_version(data: bytes):
    if len(data) < VERSION_LEN or data[0] != 0x84:
        return None
    chip = data[1]
    (epoch,) = struct.unpack_from("<I", data, 2)
    git = data[6:18].split(b"\x00", 1)[0].decode("ascii", errors="replace")
    return {
        "chip": chip,
        "chip_name": CHIP_NAMES.get(chip, f"unknown(0x{chip:02x})"),
        "epoch": epoch,
        "git": git,
    }


TMC_RX_TRACE_LEN = 16
TMC_CONFIG_LEN = 12 + TMC_RX_TRACE_LEN
TMC_STATUS_NAMES = {
    0: "ok", 1: "bad_joint", 2: "uart_error", 3: "timeout", 4: "bad_register",
}


def parse_tmc_config(data: bytes):
    if len(data) < TMC_CONFIG_LEN or data[0] != 0x83:
        return None
    joint = data[1]
    status = data[2]
    gconf, chopconf = struct.unpack_from("<II", data, 3)
    rx_count = data[11]
    rx_trace = list(data[12:12 + TMC_RX_TRACE_LEN])
    rx_hex = " ".join(f"{b:02x}" for b in rx_trace[: min(rx_count, TMC_RX_TRACE_LEN)])
    return {
        "joint": joint,
        "ok": status == 0,
        "status": status,
        "status_name": TMC_STATUS_NAMES.get(status, f"unknown({status})"),
        "gconf": gconf,
        "chopconf": chopconf,
        "rx_count": rx_count,
        "rx_hex": rx_hex,
    }


def parse_state(data: bytes):
    if len(data) < STATE_LEN or data[0] != 0x81:
        return None
    positions = struct.unpack_from(f"<{NUM_JOINTS}i", data, 1)
    servos_off = 1 + NUM_JOINTS * 4
    servos = struct.unpack_from(f"<{NUM_SERVOS}H", data, servos_off)
    flags = data[servos_off + NUM_SERVOS * 2]
    return positions, servos, flags


# == data ==================================================================

@dataclass
class ServoConfig:
    min_us: int = 500
    max_us: int = 2_500
    deadzone_us: int = 5
    speed_us_per_s: int = 1_000
    home_us: int = 1_500
    flags: int = SERVO_FLAG_HOMING

    @classmethod
    def from_dict(cls, d: dict) -> "ServoConfig":
        return cls(
            min_us=int(d.get("min_us", 500)),
            max_us=int(d.get("max_us", 2500)),
            deadzone_us=int(d.get("deadzone_us", 5)),
            speed_us_per_s=int(d.get("speed_us_per_s", 1000)),
            home_us=int(d.get("home_us", 1500)),
            flags=int(d.get("flags", SERVO_FLAG_HOMING)),
        )


# == serial bridge =========================================================

@dataclass
class Bridge:
    port_name: str
    port: serial.Serial | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)
    positions: list[int] = field(default_factory=lambda: [0] * NUM_JOINTS)
    targets: list[int] = field(default_factory=lambda: [0] * NUM_JOINTS)
    servo_positions: list[int] = field(default_factory=lambda: [1500] * NUM_SERVOS)
    servo_targets: list[int] = field(default_factory=lambda: [1500] * NUM_SERVOS)
    servo_configs: list[ServoConfig] = field(
        default_factory=lambda: [ServoConfig() for _ in range(NUM_SERVOS)]
    )
    flags: int = 0
    vmax: int = 10_000
    amax: int = 300
    enable_mask: int = 0
    connected: bool = False
    error: str = ""
    version: dict | None = None

    def open(self) -> None:
        self._close_port()
        self.version = None
        try:
            self.port = serial.Serial(self.port_name, timeout=0.2)
            self.connected = True
            self.error = ""
            self._write(cmd_enable(0))
            for i in range(NUM_JOINTS):
                self._write(cmd_set_target(i, 0, self.vmax, self.amax))
            # Push servo configs; firmware does not persist them across resets.
            for s in range(NUM_SERVOS):
                self._write(cmd_set_servo_config(s, self.servo_configs[s]))
            self._fetch_version_locked()
        except (serial.SerialException, OSError, ValueError) as e:
            self.port = None
            self.connected = False
            self.error = str(e)

    def _fetch_version_locked(self) -> None:
        if self.port is None or not self.port.is_open:
            return
        try:
            self.port.reset_input_buffer()
            self.port.write(cmd_get_version())
            resp = self.port.read(VERSION_LEN)
        except (serial.SerialException, OSError):
            return
        self.version = parse_version(resp)

    def reopen(self, port_name: str | None = None) -> None:
        with self.lock:
            if port_name:
                self.port_name = port_name
            self.open()

    def _close_port(self) -> None:
        if self.port is not None:
            try:
                if self.port.is_open:
                    self.port.close()
            except Exception:
                pass
            self.port = None

    def _write(self, frame: bytes) -> None:
        if self.port is None or not self.port.is_open:
            return
        try:
            self.port.write(frame)
        except (serial.SerialException, OSError) as e:
            self.connected = False
            self.error = str(e)
            self._close_port()

    def set_target(self, joint: int, position: int) -> None:
        if not 0 <= joint < NUM_JOINTS:
            return
        with self.lock:
            self.targets[joint] = position
            self._write(cmd_set_target(joint, position, self.vmax, self.amax))

    def set_enable(self, mask: int) -> None:
        with self.lock:
            self.enable_mask = mask & ((1 << NUM_JOINTS) - 1)
            self._write(cmd_enable(self.enable_mask))

    def set_speeds(self, vmax: int | None, amax: int | None) -> None:
        with self.lock:
            if vmax is not None:
                self.vmax = max(1, int(vmax))
            if amax is not None:
                self.amax = max(1, int(amax))
            for i, t in enumerate(self.targets):
                self._write(cmd_set_target(i, t, self.vmax, self.amax))

    def set_tmc(self, joint: int, flags: int) -> None:
        with self.lock:
            self._write(cmd_set_tmc_config(joint, flags))

    def get_tmc_config(self, joint: int) -> dict:
        if joint not in TMC_UART_JOINTS:
            return {"ok": False, "error": "joint has no UART driver"}
        with self.lock:
            if self.port is None or not self.port.is_open:
                return {"ok": False, "error": "disconnected"}
            try:
                self.port.reset_input_buffer()
                self.port.write(cmd_get_tmc_config(joint))
                resp = self.port.read(TMC_CONFIG_LEN)
            except (serial.SerialException, OSError) as e:
                self.connected = False
                self.error = str(e)
                self._close_port()
                return {"ok": False, "error": str(e)}
        parsed = parse_tmc_config(resp)
        if parsed is None:
            return {
                "ok": False,
                "reason": "no_reply",
                "got_bytes": len(resp),
                "hint": "firmware likely outdated - reflash with the new UF2",
            }
        parsed["reason"] = "ok" if parsed["ok"] else "uart_no_answer"
        return parsed

    def set_position(self, joint: int, position: int) -> None:
        if not 0 <= joint < NUM_JOINTS:
            return
        with self.lock:
            self.targets[joint] = position
            self.positions[joint] = position
            self._write(cmd_set_position(joint, position))

    def set_servo_target(self, servo: int, target_us: int) -> None:
        if not 0 <= servo < NUM_SERVOS:
            return
        with self.lock:
            cfg = self.servo_configs[servo]
            target_us = max(cfg.min_us, min(cfg.max_us, int(target_us)))
            self.servo_targets[servo] = target_us
            self._write(cmd_set_servo_target(servo, target_us))

    def set_servo_config(self, servo: int, cfg: ServoConfig) -> None:
        if not 0 <= servo < NUM_SERVOS:
            return
        with self.lock:
            self.servo_configs[servo] = cfg
            self._write(cmd_set_servo_config(servo, cfg))

    def poll_loop(self) -> None:
        while True:
            if self.port is None or not self.port.is_open:
                time.sleep(1.0)
                with self.lock:
                    self.open()
                continue
            try:
                with self.lock:
                    self.port.reset_input_buffer()
                    self.port.write(cmd_get_state())
                    resp = self.port.read(STATE_LEN)
            except (serial.SerialException, OSError) as e:
                self.connected = False
                self.error = str(e)
                self._close_port()
                time.sleep(0.3)
                continue
            result = parse_state(resp)
            if result is not None:
                positions, servos, flags = result
                self.positions = list(positions)
                self.servo_positions = list(servos)
                self.flags = flags
            time.sleep(0.04)

    def snapshot(self) -> dict:
        return {
            "connected": self.connected,
            "error": self.error,
            "port": self.port_name,
            "positions": list(self.positions),
            "targets": list(self.targets),
            "servo_positions": list(self.servo_positions),
            "servo_targets": list(self.servo_targets),
            "servo_configs": [c.__dict__ for c in self.servo_configs],
            "flags": self.flags,
            "vmax": self.vmax,
            "amax": self.amax,
            "enable_mask": self.enable_mask,
            "num_joints": NUM_JOINTS,
            "num_servos": NUM_SERVOS,
            "tmc_uart_joints": list(TMC_UART_JOINTS),
            "version": self.version,
        }


# == optional IK (lazy) ====================================================

class IKAdapter:
    """Lazy URDF + ikpy wrapper. We don't import ikpy at module load so a
    missing dep doesn't block the rest of the web UI from starting."""

    def __init__(self) -> None:
        self.robot = None
        self.error: str | None = None

    def load(self, urdf: str | None = None, motor_map: str | None = None) -> dict:
        try:
            import ik  # local module
        except ImportError as e:
            self.error = f"ikpy/numpy not installed: {e}"
            self.robot = None
            return {"ok": False, "error": self.error}
        urdf_path = Path(urdf) if urdf else ik.DEFAULT_URDF
        map_path = Path(motor_map) if motor_map else ik.DEFAULT_MOTOR_MAP
        try:
            self.robot = ik.Robot.load(urdf_path, map_path)
            self.error = None
        except Exception as e:
            self.robot = None
            self.error = str(e)
        return {
            "ok": self.robot is not None,
            "error": self.error,
            "urdf": str(urdf_path),
            "motor_map": str(map_path),
        }

    def solve(self, x: float, y: float, z: float) -> dict:
        if self.robot is None:
            return {"ok": False, "error": self.error or "URDF not loaded"}
        try:
            jv = self.robot.solve_ik([float(x), float(y), float(z)])
            ee = self.robot.forward_kinematics(jv)
            targets = self.robot.to_motor_targets(jv)
            specs = self.robot.joint_specs()
            return {
                "ok": True,
                "achieved": ee[:3, 3].tolist(),
                "targets": [
                    {"motor": m, "position": int(t),
                     "vmax": specs[m].vmax, "amax": specs[m].amax}
                    for m, t in sorted(targets.items())
                ],
            }
        except Exception as e:
            return {"ok": False, "error": str(e)}


# == fasthtml app ==========================================================

def build_app(bridge: Bridge) -> FastHTML:
    app = FastHTML()
    ik = IKAdapter()

    @app.get("/")
    def index():
        return HTMLResponse(INDEX_PATH.read_text())

    @app.get("/state")
    def get_state():
        snap = bridge.snapshot()
        snap["ik"] = {"loaded": ik.robot is not None, "error": ik.error}
        return JSONResponse(snap)

    @app.post("/target")
    async def post_target(req: Request):
        data = await req.json()
        bridge.set_target(int(data.get("joint", -1)), int(data.get("position", 0)))
        return JSONResponse({"ok": True})

    @app.post("/targets")
    async def post_targets(req: Request):
        data = await req.json()
        for t in data.get("targets", []):
            bridge.set_target(int(t["joint"]), int(t["position"]))
        return JSONResponse({"ok": True})

    @app.post("/enable")
    async def post_enable(req: Request):
        data = await req.json()
        bridge.set_enable(int(data.get("mask", 0)))
        return JSONResponse({"ok": True})

    @app.post("/speeds")
    async def post_speeds(req: Request):
        data = await req.json()
        bridge.set_speeds(data.get("vmax"), data.get("amax"))
        return JSONResponse({"ok": True})

    @app.post("/tmc")
    async def post_tmc(req: Request):
        data = await req.json()
        bridge.set_tmc(int(data.get("joint", 0)), int(data.get("flags", 0)))
        return JSONResponse({"ok": True})

    @app.post("/set_position")
    async def post_set_position(req: Request):
        data = await req.json()
        if isinstance(data.get("targets"), list):
            for t in data["targets"]:
                bridge.set_position(int(t["joint"]), int(t["position"]))
        else:
            bridge.set_position(int(data.get("joint", -1)),
                                int(data.get("position", 0)))
        return JSONResponse({"ok": True})

    @app.post("/verify_tmc")
    async def post_verify_tmc(req: Request):
        data = await req.json()
        return JSONResponse(bridge.get_tmc_config(int(data.get("joint", -1))))

    @app.post("/servo/target")
    async def post_servo_target(req: Request):
        data = await req.json()
        bridge.set_servo_target(int(data.get("servo", -1)),
                                int(data.get("target_us", 1500)))
        return JSONResponse({"ok": True})

    @app.post("/servo/config")
    async def post_servo_config(req: Request):
        data = await req.json()
        servo = int(data.get("servo", -1))
        cfg = ServoConfig.from_dict(data.get("config") or data)
        bridge.set_servo_config(servo, cfg)
        return JSONResponse({"ok": True, "config": cfg.__dict__})

    @app.post("/reopen")
    async def post_reopen(req: Request):
        try:
            data = await req.json()
        except Exception:
            data = {}
        port_name = data.get("port") if isinstance(data, dict) else None
        bridge.reopen(port_name)
        return JSONResponse({
            "ok": bridge.connected,
            "port": bridge.port_name,
            "error": bridge.error,
        })

    @app.post("/ik/load")
    async def post_ik_load(req: Request):
        try:
            data = await req.json()
        except Exception:
            data = {}
        return JSONResponse(ik.load(data.get("urdf"), data.get("motor_map")))

    @app.post("/ik/solve")
    async def post_ik_solve(req: Request):
        data = await req.json()
        result = ik.solve(data.get("x", 0.0), data.get("y", 0.0), data.get("z", 0.0))
        if result.get("ok") and data.get("send"):
            for t in result["targets"]:
                bridge.set_target(int(t["motor"]), int(t["position"]))
            result["sent"] = True
        return JSONResponse(result)

    return app


def main() -> None:
    port_name = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("TMC_PORT", DEFAULT_PORT)
    bridge = Bridge(port_name=port_name)
    bridge.open()
    threading.Thread(target=bridge.poll_loop, daemon=True).start()

    app = build_app(bridge)
    print(f"serving http://{HTTP_HOST}:{HTTP_PORT}  (serial {port_name})")
    try:
        uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT, log_level="warning")
    except KeyboardInterrupt:
        print("\nstopping")
        bridge.set_enable(0)
        bridge._close_port()


if __name__ == "__main__":
    main()
