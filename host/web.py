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

import array
import fcntl
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

# Joints with UART-addressable TMC2209 drivers. Matches the firmware's
# `JointId::ALL` (J0..J3 on UART1 addresses 0..=3, J4 alone on UART0).
# With `--no-default-features` (uart off) reads will surface as uart_error
# from the firmware - the host still asks, the answer is just "no driver bus".
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

    def jog(self, joint: int, sign: int, vmax: int) -> None:
        # Velocity-mode jog. sign in {-1, 0, +1}; vmax is steps/s magnitude.
        # sign == 0 commands a stop at the current position; otherwise we send
        # a far target so the firmware cruises at `vmax` until we re-command.
        if not 0 <= joint < NUM_JOINTS:
            return
        with self.lock:
            cur = int(self.positions[joint])
            self.targets[joint] = cur
            if sign == 0:
                self._write(cmd_set_target(joint, cur, self.vmax, self.amax))
            else:
                far = cur + (1 if sign > 0 else -1) * 100_000_000
                self._write(cmd_set_target(joint, far, max(1, int(vmax)), self.amax))

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

    def jog_servo(self, servo: int, sign: int) -> None:
        # Velocity-mode jog for servos. Speed is set by ServoConfig.speed_us_per_s;
        # we just aim at min/max us (or freeze at current us on release).
        if not 0 <= servo < NUM_SERVOS:
            return
        with self.lock:
            cfg = self.servo_configs[servo]
            cur = int(self.servo_positions[servo])
            cur = max(cfg.min_us, min(cfg.max_us, cur))
            if sign == 0:
                self.servo_targets[servo] = cur
                self._write(cmd_set_servo_target(servo, cur))
            elif sign > 0:
                self.servo_targets[servo] = cfg.max_us
                self._write(cmd_set_servo_target(servo, cfg.max_us))
            else:
                self.servo_targets[servo] = cfg.min_us
                self._write(cmd_set_servo_target(servo, cfg.min_us))

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


# == joystick (Linux /dev/input/jsN) =======================================

JS_EVENT_FMT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_MAX = 32767.0
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12

def _JSIOCGNAME(length: int) -> int:
    return 0x80006A13 + (length << 16)


def _js_device_info(fd: int) -> tuple[str, int, int]:
    a = array.array("B", [0]); fcntl.ioctl(fd, JSIOCGAXES, a)
    b = array.array("B", [0]); fcntl.ioctl(fd, JSIOCGBUTTONS, b)
    name = array.array("B", [0] * 128); fcntl.ioctl(fd, _JSIOCGNAME(128), name)
    return bytes(name).rstrip(b"\x00").decode(errors="replace"), a[0], b[0]


# axis index -> (default target, default sensitivity units/sec)
# Defaults from /home/tmpk/usb-controller-stick/README.md (Saitek Cyborg):
#   0 L&R tilt, 1 F&B tilt, 3 pivot rotation; 4/5 top stick (0/1 hat)
DEFAULT_AXIS_CFG: list[dict] = [
    {"target": "joint:3", "sensitivity": 4000.0, "deadzone": 0.1, "invert": False},
    {"target": "joint:1", "sensitivity": 4000.0, "deadzone": 0.1, "invert": False},
    {"target": "none",    "sensitivity": 0.0,    "deadzone": 0.1, "invert": False},
    {"target": "joint:0", "sensitivity": 4000.0, "deadzone": 0.1, "invert": False},
    {"target": "joint:4", "sensitivity": 2000.0, "deadzone": 0.1, "invert": False},
    {"target": "joint:2", "sensitivity": 4000.0, "deadzone": 0.1, "invert": False},
]
DEFAULT_BTN_CFG: list[dict] = [
    {"target": "none", "sensitivity": 100.0} for _ in range(12)
]


@dataclass
class Joystick:
    bridge: "Bridge"
    device: str = "/dev/input/js0"
    enabled: bool = False
    axes_cfg: list[dict] = field(default_factory=lambda: [dict(c) for c in DEFAULT_AXIS_CFG])
    buttons_cfg: list[dict] = field(default_factory=lambda: [dict(c) for c in DEFAULT_BTN_CFG])
    axis_state: list[int] = field(default_factory=lambda: [0] * 8)
    button_state: list[int] = field(default_factory=lambda: [0] * 16)
    device_name: str = ""
    connected: bool = False
    error: str = ""
    n_axes: int = 0
    n_buttons: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _acc_joint: list[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    _acc_servo: list[float] = field(default_factory=lambda: [0.0] * NUM_SERVOS)
    _last_joint: list[int] = field(default_factory=lambda: [0] * NUM_JOINTS)
    _last_servo: list[int] = field(default_factory=lambda: [0] * NUM_SERVOS)
    _primed: bool = False
    # Last commanded jog per (kind, idx). For joints: signed steps/s (bucketed);
    # for servos: -1/0/+1. Used to avoid re-sending identical commands.
    _axis_cmd: dict[tuple[str, int], int] = field(default_factory=dict)

    def snapshot(self) -> dict:
        return {
            "enabled": self.enabled,
            "device": self.device,
            "device_name": self.device_name,
            "connected": self.connected,
            "error": self.error,
            "n_axes": self.n_axes,
            "n_buttons": self.n_buttons,
            "axis_state": list(self.axis_state),
            "button_state": list(self.button_state),
            "axes_cfg": [dict(c) for c in self.axes_cfg],
            "buttons_cfg": [dict(c) for c in self.buttons_cfg],
        }

    def configure(self, data: dict) -> None:
        with self._lock:
            if "device" in data and isinstance(data["device"], str):
                new_dev = data["device"].strip()
                if new_dev and new_dev != self.device:
                    self.device = new_dev
                    self.connected = False  # reader_loop will reopen
            if "enabled" in data:
                self.enabled = bool(data["enabled"])
                if self.enabled:
                    self._primed = False  # re-prime accumulator on enable
            if isinstance(data.get("axes_cfg"), list):
                for i, c in enumerate(data["axes_cfg"]):
                    if i >= len(self.axes_cfg):
                        self.axes_cfg.append({})
                    self.axes_cfg[i].update({
                        "target":      str(c.get("target", "none")),
                        "sensitivity": float(c.get("sensitivity", 0.0)),
                        "deadzone":    max(0.0, min(0.95, float(c.get("deadzone", 0.0)))),
                        "invert":      bool(c.get("invert", False)),
                    })
            if isinstance(data.get("buttons_cfg"), list):
                for i, c in enumerate(data["buttons_cfg"]):
                    if i >= len(self.buttons_cfg):
                        self.buttons_cfg.append({})
                    self.buttons_cfg[i].update({
                        "target":      str(c.get("target", "none")),
                        "sensitivity": float(c.get("sensitivity", 0.0)),
                    })

    def _prime_locked(self) -> None:
        with self.bridge.lock:
            self._acc_joint = [float(t) for t in self.bridge.targets]
            self._acc_servo = [float(t) for t in self.bridge.servo_targets]
        self._last_joint = [int(round(v)) for v in self._acc_joint]
        self._last_servo = [int(round(v)) for v in self._acc_servo]
        self._primed = True

    def _apply_delta(self, target: str, delta: float) -> None:
        kind, _, idx_s = target.partition(":")
        try:
            idx = int(idx_s)
        except ValueError:
            return
        if kind == "joint" and 0 <= idx < NUM_JOINTS:
            self._acc_joint[idx] += delta
            new_v = int(round(self._acc_joint[idx]))
            if new_v != self._last_joint[idx]:
                self._last_joint[idx] = new_v
                self.bridge.set_target(idx, new_v)
        elif kind == "servo" and 0 <= idx < NUM_SERVOS:
            self._acc_servo[idx] += delta
            new_v = int(round(self._acc_servo[idx]))
            if new_v != self._last_servo[idx]:
                self._last_servo[idx] = new_v
                self.bridge.set_servo_target(idx, new_v)

    def _on_button_press(self, idx: int) -> None:
        if idx >= len(self.buttons_cfg):
            return
        cfg = self.buttons_cfg[idx]
        target = cfg.get("target", "none")
        if target == "none":
            return
        if not self._primed:
            self._prime_locked()
        self._apply_delta(target, float(cfg.get("sensitivity", 0.0)))

    def reader_loop(self) -> None:
        fd = -1
        while True:
            if not self.enabled:
                if fd >= 0:
                    try: os.close(fd)
                    except OSError: pass
                    fd = -1
                self.connected = False
                time.sleep(0.2)
                continue
            if fd < 0:
                try:
                    fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
                    name, n_axes, n_btns = _js_device_info(fd)
                    self.device_name = name
                    self.n_axes = n_axes
                    self.n_buttons = n_btns
                    self.axis_state = [0] * max(n_axes, 8)
                    self.button_state = [0] * max(n_btns, 16)
                    self.connected = True
                    self.error = ""
                except OSError as e:
                    self.error = str(e)
                    self.connected = False
                    time.sleep(1.0)
                    continue
            try:
                data = os.read(fd, JS_EVENT_SIZE)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            except OSError as e:
                self.error = str(e)
                try: os.close(fd)
                except OSError: pass
                fd = -1
                self.connected = False
                time.sleep(0.5)
                continue
            if not data or len(data) < JS_EVENT_SIZE:
                time.sleep(0.005)
                continue
            _, value, ev_type, number = struct.unpack(JS_EVENT_FMT, data)
            is_init = bool(ev_type & JS_EVENT_INIT)
            ev_type &= ~JS_EVENT_INIT
            if ev_type == JS_EVENT_AXIS and number < len(self.axis_state):
                self.axis_state[number] = value
            elif ev_type == JS_EVENT_BUTTON and number < len(self.button_state):
                prev = self.button_state[number]
                self.button_state[number] = value
                if value and not prev and not is_init:
                    self._on_button_press(number)

    def _apply_axis_velocity(self, target: str, signed_velocity: float) -> None:
        kind, _, idx_s = target.partition(":")
        try:
            idx = int(idx_s)
        except ValueError:
            return
        key = (kind, idx)
        if kind == "joint" and 0 <= idx < NUM_JOINTS:
            # Bucket vmax to nearest 25 steps/s to suppress chatter when the
            # stick drifts; below 1 step/s we treat it as a stop.
            if abs(signed_velocity) < 1.0:
                new_cmd = 0
            else:
                sign = 1 if signed_velocity > 0 else -1
                mag = max(25, int(round(abs(signed_velocity) / 25.0)) * 25)
                new_cmd = sign * mag
            if self._axis_cmd.get(key) == new_cmd:
                return
            self._axis_cmd[key] = new_cmd
            if new_cmd == 0:
                self.bridge.jog(idx, 0, 0)
            else:
                self.bridge.jog(idx, 1 if new_cmd > 0 else -1, abs(new_cmd))
        elif kind == "servo" and 0 <= idx < NUM_SERVOS:
            new_cmd = 0 if abs(signed_velocity) < 1.0 else (1 if signed_velocity > 0 else -1)
            if self._axis_cmd.get(key) == new_cmd:
                return
            self._axis_cmd[key] = new_cmd
            self.bridge.jog_servo(idx, new_cmd)

    def _stop_all_axes(self) -> None:
        for (kind, idx), cmd in list(self._axis_cmd.items()):
            if cmd == 0:
                continue
            if kind == "joint":
                self.bridge.jog(idx, 0, 0)
            elif kind == "servo":
                self.bridge.jog_servo(idx, 0)
        self._axis_cmd.clear()

    def integrate_loop(self) -> None:
        was_active = False
        while True:
            time.sleep(0.025)
            active = self.enabled and self.connected
            if not active:
                if was_active:
                    self._stop_all_axes()
                self._primed = False
                was_active = False
                continue
            was_active = True
            for i, cfg in enumerate(self.axes_cfg):
                if i >= len(self.axis_state):
                    continue
                target = cfg.get("target", "none")
                if target == "none":
                    continue
                raw = self.axis_state[i] / JS_MAX
                if cfg.get("invert"):
                    raw = -raw
                dz = float(cfg.get("deadzone", 0.0))
                ar = abs(raw)
                if ar < dz:
                    signed_v = 0.0
                else:
                    norm = ((ar - dz) / (1.0 - dz)) * (1.0 if raw > 0 else -1.0)
                    signed_v = norm * float(cfg.get("sensitivity", 0.0))
                self._apply_axis_velocity(target, signed_v)


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

def build_app(bridge: Bridge, joystick: "Joystick") -> FastHTML:
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

    @app.get("/joystick")
    def get_joystick():
        return JSONResponse(joystick.snapshot())

    @app.post("/joystick/config")
    async def post_joystick_config(req: Request):
        data = await req.json()
        joystick.configure(data)
        return JSONResponse({"ok": True, "state": joystick.snapshot()})

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

    joystick = Joystick(bridge=bridge)
    threading.Thread(target=joystick.reader_loop, daemon=True).start()
    threading.Thread(target=joystick.integrate_loop, daemon=True).start()

    app = build_app(bridge, joystick)
    print(f"serving http://{HTTP_HOST}:{HTTP_PORT}  (serial {port_name})")
    try:
        uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT, log_level="warning")
    except KeyboardInterrupt:
        print("\nstopping")
        bridge.set_enable(0)
        bridge._close_port()


if __name__ == "__main__":
    main()
