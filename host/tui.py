"""
Textual TUI for the tmc2209-fw firmware.

Usage:
    pixi run tui [PORT]

Defaults to /dev/ttyACM0. Click the jog buttons (or use the keyboard - see
the help/key-bind modal) to move each joint by the selected step size; toggle
the enable switches to power drivers on/off. Two servos drive the end effector
gripper. Live positions stream from the firmware's GetState report.

Keyboard bindings are configurable (press `?` to view, `k` to rebind) and
persist across runs in `~/.config/tmc2209-tui/keys.json`. The user can also
edit that JSON file directly.
"""

from __future__ import annotations

import json
import os
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import serial
from textual import on, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Grid, Horizontal, Vertical
from textual.reactive import reactive
from textual.screen import ModalScreen
from textual.widgets import (
    Button,
    Footer,
    Header,
    Input,
    Label,
    RadioButton,
    RadioSet,
    Static,
    Switch,
)

DEFAULT_PORT = "/dev/ttyACM0"
NUM_JOINTS = 5
NUM_SERVOS = 2

# Joints 0..2 = main linkages, 3 = base rotation, 4 = linear stage.
JOINT_LABELS = ["J0 link", "J1 link", "J2 link", "J3 base", "J4 lin"]

DEFAULT_VMAX = 10_000
DEFAULT_AMAX = 300
JOG_SIZES = [10, 100, 1_000, 3_200]  # microsteps; 3200 = 1 full rev at 16x

# TMC config flag bits - must match src/protocol.rs TMC_FLAG_* constants.
TMC_FLAG_STEALTHCHOP = 1 << 0
TMC_FLAG_INTERPOLATE = 1 << 1
TMC_FLAG_SHAFT_INVERT = 1 << 2

# Servo flag bits - must match src/servo.rs SERVO_FLAG_* constants.
SERVO_FLAG_HOMING = 1 << 0

# Joints with UART-controlled TMC2209 drivers. The firmware uses two UART
# buses (UART1 for J0..J3, UART0 for J4) so all five joints are addressable.
TMC_UART_JOINTS = {0, 1, 2, 3, 4}

# Each toggle: (id_suffix, label, flag_bit, default_on)
# Defaults mirror DriverBus::apply_default_config in the firmware.
TMC_TOGGLES = [
    ("sc", "stealthChop", TMC_FLAG_STEALTHCHOP, True),
    ("ip", "interpolate", TMC_FLAG_INTERPOLATE, True),
    ("sh", "invert dir", TMC_FLAG_SHAFT_INVERT, False),
]

CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "tmc2209-tui"
KEYS_PATH = CONFIG_DIR / "keys.json"

# Action ids and human descriptions. Each action takes no arguments - "jog
# joint 0 negative" is a single action, parameterised by the current jog-size
# setting. Keep these stable; the keys.json file references them.
ACTIONS: dict[str, str] = {
    "quit": "Quit",
    "stop_all": "Stop all",
    "toggle_all": "Enable/disable all",
    "rebind": "Open key-binding editor",
    "help": "Show key bindings",
    **{f"j{j}_neg": f"Jog {JOINT_LABELS[j]} -" for j in range(NUM_JOINTS)},
    **{f"j{j}_pos": f"Jog {JOINT_LABELS[j]} +" for j in range(NUM_JOINTS)},
    **{f"j{j}_home": f"Home {JOINT_LABELS[j]}" for j in range(NUM_JOINTS)},
    **{f"s{s}_neg": f"Servo {s} -100us" for s in range(NUM_SERVOS)},
    **{f"s{s}_pos": f"Servo {s} +100us" for s in range(NUM_SERVOS)},
    **{f"s{s}_home": f"Servo {s} home" for s in range(NUM_SERVOS)},
}

DEFAULT_KEYS: dict[str, str] = {
    "quit": "q",
    "stop_all": "space",
    "toggle_all": "e",
    "rebind": "k",
    "help": "question_mark",
    # Linkages on QWE/ASD (negative/positive).
    "j0_neg": "a", "j0_pos": "q",
    "j1_neg": "s", "j1_pos": "w",
    "j2_neg": "d", "j2_pos": "f",
    # Base + linear stage on F/R and C/V.
    "j3_neg": "z", "j3_pos": "x",
    "j4_neg": "c", "j4_pos": "v",
    # Home joints with Shift+number (Textual reports as upper digit).
    "j0_home": "1", "j1_home": "2", "j2_home": "3", "j3_home": "4", "j4_home": "5",
    # Servos: open/close gripper, plus home.
    "s0_neg": "left_square_bracket", "s0_pos": "right_square_bracket", "s0_home": "minus",
    "s1_neg": "semicolon", "s1_pos": "apostrophe", "s1_home": "equals_sign",
}


def load_keys() -> dict[str, str]:
    if not KEYS_PATH.exists():
        return dict(DEFAULT_KEYS)
    try:
        data = json.loads(KEYS_PATH.read_text())
    except (OSError, json.JSONDecodeError):
        return dict(DEFAULT_KEYS)
    out = dict(DEFAULT_KEYS)
    for action, key in data.items():
        if action in ACTIONS and isinstance(key, str):
            out[action] = key
    return out


def save_keys(keys: dict[str, str]) -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    KEYS_PATH.write_text(json.dumps(keys, indent=2, sort_keys=True))


# == protocol helpers (mirror test.py) ====================================

def cmd_enable(mask: int) -> bytes:
    return struct.pack("<BB", 0x02, mask & 0xFF)


def cmd_set_target(joint: int, position: int, vmax: int, amax: int) -> bytes:
    return struct.pack("<BBiII", 0x01, joint, position, vmax, amax)


def cmd_get_state() -> bytes:
    return b"\x04"


def cmd_set_tmc_config(joint: int, flags: int) -> bytes:
    return struct.pack("<BBB", 0x05, joint, flags & 0xFF)


def cmd_set_servo_target(servo: int, target_us: int) -> bytes:
    return struct.pack("<BBH", 0x09, servo, target_us & 0xFFFF)


def cmd_set_servo_config(servo: int, cfg: "ServoConfig") -> bytes:
    return struct.pack(
        "<BBHHHHHB", 0x0A, servo,
        cfg.min_us, cfg.max_us, cfg.deadzone_us, cfg.speed_us_per_s, cfg.home_us,
        cfg.flags & 0xFF,
    )


# StateReport size = 1 tag + NUM_JOINTS*4 + NUM_SERVOS*2 + 1 flags = 26 for 5/2.
STATE_SIZE = 1 + NUM_JOINTS * 4 + NUM_SERVOS * 2 + 1


def parse_state(data: bytes):
    if len(data) < STATE_SIZE or data[0] != 0x81:
        return None
    positions = struct.unpack_from(f"<{NUM_JOINTS}i", data, 1)
    servos_off = 1 + NUM_JOINTS * 4
    servos = struct.unpack_from(f"<{NUM_SERVOS}H", data, servos_off)
    flags = data[servos_off + NUM_SERVOS * 2]
    return positions, servos, flags


# == data classes =========================================================

@dataclass
class JointState:
    target: int = 0
    position: int = 0
    enabled: bool = False
    tmc_flags: int = 0


@dataclass
class ServoConfig:
    min_us: int = 500
    max_us: int = 2_500
    deadzone_us: int = 5
    speed_us_per_s: int = 1_000
    home_us: int = 1_500
    flags: int = SERVO_FLAG_HOMING


@dataclass
class ServoState:
    config: ServoConfig = field(default_factory=ServoConfig)
    target_us: int = 1_500
    current_us: int = 1_500


# == widgets ==============================================================

class JointPanel(Static):
    """One row of jog controls for a single joint."""

    DEFAULT_CSS = """
    JointPanel {
        border: round $accent;
        padding: 0 1;
        margin: 0 0 1 0;
        height: auto;
    }
    JointPanel .title {
        text-style: bold;
        color: $accent;
    }
    JointPanel .pos {
        color: $success;
        text-style: bold;
    }
    JointPanel .row {
        height: 3;
        align: left middle;
    }
    JointPanel Button {
        min-width: 9;
        margin: 0 1 0 0;
    }
    JointPanel .stop {
        background: $error 30%;
    }
    JointPanel .home {
        background: $warning 30%;
    }
    """

    def __init__(self, joint: int) -> None:
        super().__init__()
        self.joint = joint
        default_flags = 0
        for _, _, bit, on in TMC_TOGGLES:
            if on:
                default_flags |= bit
        self.state = JointState(tmc_flags=default_flags)

    def compose(self) -> ComposeResult:
        with Horizontal(classes="row"):
            yield Label(JOINT_LABELS[self.joint], classes="title")
            yield Switch(value=False, id=f"en{self.joint}")
            yield Label("disabled", id=f"st{self.joint}")
            yield Label("", id=f"pos{self.joint}", classes="pos")
        with Horizontal(classes="row"):
            for size in JOG_SIZES:
                yield Button(f"-{size}", id=f"j{self.joint}_neg_{size}")
            for size in reversed(JOG_SIZES):
                yield Button(f"+{size}", id=f"j{self.joint}_pos_{size}")
        with Horizontal(classes="row"):
            yield Button("home (0)", id=f"home{self.joint}", classes="home")
            yield Button("stop", id=f"stop{self.joint}", classes="stop")
            yield Label("target:")
            yield Input(value="0", id=f"tgt{self.joint}", restrict=r"-?\d*")
            yield Button("go", id=f"go{self.joint}")
        if self.joint in TMC_UART_JOINTS:
            with Horizontal(classes="row"):
                for sfx, label, bit, _ in TMC_TOGGLES:
                    yield Switch(
                        value=bool(self.state.tmc_flags & bit),
                        id=f"tmc{self.joint}_{sfx}",
                    )
                    yield Label(label)
        else:
            with Horizontal(classes="row"):
                yield Label("(standalone - no UART config)")

    def update_position(self, pos: int) -> None:
        self.state.position = pos
        self.query_one(f"#pos{self.joint}", Label).update(f"pos {pos:>+8d}  ->  {self.state.target:>+8d}")

    def update_target(self, tgt: int) -> None:
        self.state.target = tgt
        self.query_one(f"#pos{self.joint}", Label).update(f"pos {self.state.position:>+8d}  ->  {tgt:>+8d}")

    def update_enabled(self, en: bool) -> None:
        self.state.enabled = en
        self.query_one(f"#st{self.joint}", Label).update("ENABLED" if en else "disabled")


class ServoPanel(Static):
    """Servo control + config row."""

    DEFAULT_CSS = """
    ServoPanel {
        border: round $primary;
        padding: 0 1;
        margin: 0 0 1 0;
        height: auto;
    }
    ServoPanel .title {
        text-style: bold;
        color: $primary;
    }
    ServoPanel .row {
        height: 3;
        align: left middle;
    }
    ServoPanel Input {
        width: 10;
    }
    ServoPanel Label {
        margin: 0 1 0 0;
    }
    """

    def __init__(self, servo: int) -> None:
        super().__init__()
        self.servo = servo
        self.state = ServoState()

    def compose(self) -> ComposeResult:
        with Horizontal(classes="row"):
            yield Label(f"S{self.servo} servo", classes="title")
            yield Label("", id=f"spos{self.servo}")
        with Horizontal(classes="row"):
            yield Button(f"-100", id=f"s{self.servo}_neg_100")
            yield Button(f"-10", id=f"s{self.servo}_neg_10")
            yield Button(f"+10", id=f"s{self.servo}_pos_10")
            yield Button(f"+100", id=f"s{self.servo}_pos_100")
            yield Button("home", id=f"s{self.servo}_home")
            yield Label("target us:")
            yield Input(value=str(self.state.target_us), id=f"stgt{self.servo}", restrict=r"\d*")
            yield Button("go", id=f"s{self.servo}_go")
        with Horizontal(classes="row"):
            yield Label("min:")
            yield Input(value=str(self.state.config.min_us), id=f"smin{self.servo}", restrict=r"\d*")
            yield Label("max:")
            yield Input(value=str(self.state.config.max_us), id=f"smax{self.servo}", restrict=r"\d*")
            yield Label("deadzone:")
            yield Input(value=str(self.state.config.deadzone_us), id=f"sdz{self.servo}", restrict=r"\d*")
        with Horizontal(classes="row"):
            yield Label("speed us/s:")
            yield Input(value=str(self.state.config.speed_us_per_s), id=f"sspd{self.servo}", restrict=r"\d*")
            yield Label("home us:")
            yield Input(value=str(self.state.config.home_us), id=f"shm{self.servo}", restrict=r"\d*")
            yield Switch(
                value=bool(self.state.config.flags & SERVO_FLAG_HOMING),
                id=f"shome{self.servo}",
            )
            yield Label("home on enable")
            yield Button("apply config", id=f"s{self.servo}_apply")

    def update_position(self, pos_us: int) -> None:
        self.state.current_us = pos_us
        self.query_one(f"#spos{self.servo}", Label).update(
            f"pos {pos_us:>4d}us  ->  {self.state.target_us:>4d}us"
        )

    def update_target(self, tgt_us: int) -> None:
        self.state.target_us = tgt_us
        self.query_one(f"#spos{self.servo}", Label).update(
            f"pos {self.state.current_us:>4d}us  ->  {tgt_us:>4d}us"
        )


# == key-binding modal ====================================================

class KeyCaptureInput(Input):
    """Single-key capture - swallow the next key press and report it."""

    class Captured(Input.Changed):
        pass

    def __init__(self, action: str, current: str) -> None:
        super().__init__(value=current, id=f"kb_{action}")
        self.action_id = action
        self.capturing = False

    async def _on_key(self, event):
        if self.capturing:
            event.prevent_default()
            event.stop()
            self.value = event.key
            self.capturing = False
            self.refresh()


class RebindScreen(ModalScreen[dict[str, str]]):
    """Modal for editing key bindings. Press a row's button, then a key."""

    CSS = """
    RebindScreen {
        align: center middle;
    }
    #rebind-box {
        background: $surface;
        border: thick $accent;
        padding: 1 2;
        width: 80%;
        height: 80%;
    }
    #rebind-grid {
        grid-size: 3;
        grid-columns: 2fr 1fr 1fr;
        grid-gutter: 0 1;
        height: 1fr;
        overflow-y: auto;
    }
    #rebind-grid Label {
        padding: 0 1;
        height: 3;
    }
    #rebind-grid Input {
        height: 3;
    }
    #rebind-grid Button {
        height: 3;
    }
    #rebind-footer {
        height: auto;
        align: right middle;
    }
    """

    BINDINGS = [Binding("escape", "cancel", "Cancel")]

    def __init__(self, keys: dict[str, str]) -> None:
        super().__init__()
        self.keys = dict(keys)

    def compose(self) -> ComposeResult:
        with Vertical(id="rebind-box"):
            yield Static(
                "Click [capture] then press the desired key. Press Save when done.",
                id="rebind-help",
            )
            with Grid(id="rebind-grid"):
                for action, desc in ACTIONS.items():
                    yield Label(desc)
                    yield Input(value=self.keys.get(action, ""), id=f"kb_{action}")
                    yield Button("capture", id=f"cap_{action}")
            with Horizontal(id="rebind-footer"):
                yield Button("Reset defaults", id="rebind-reset")
                yield Button("Cancel", id="rebind-cancel")
                yield Button("Save", id="rebind-save", variant="primary")

    @on(Button.Pressed)
    def on_button(self, event: Button.Pressed) -> None:
        bid = event.button.id or ""
        if bid == "rebind-cancel":
            self.dismiss(None)
        elif bid == "rebind-save":
            for action in ACTIONS:
                inp = self.query_one(f"#kb_{action}", Input)
                self.keys[action] = inp.value.strip() or DEFAULT_KEYS.get(action, "")
            self.dismiss(self.keys)
        elif bid == "rebind-reset":
            for action, key in DEFAULT_KEYS.items():
                self.query_one(f"#kb_{action}", Input).value = key
        elif bid.startswith("cap_"):
            action = bid[4:]
            inp = self.query_one(f"#kb_{action}", Input)
            inp.value = ""
            inp.placeholder = "press a key..."
            inp.focus()
            self._capturing_action = action

    async def on_key(self, event) -> None:
        # If a capture is active and focus is on its Input, treat the next key
        # as the new binding.
        target = getattr(self, "_capturing_action", None)
        if not target:
            return
        focused = self.focused
        if not focused or focused.id != f"kb_{target}":
            return
        event.prevent_default()
        event.stop()
        if event.key in ("escape",):
            self._capturing_action = None
            return
        inp = self.query_one(f"#kb_{target}", Input)
        inp.value = event.key
        inp.placeholder = ""
        self._capturing_action = None

    def action_cancel(self) -> None:
        self.dismiss(None)


class HelpScreen(ModalScreen[None]):
    CSS = """
    HelpScreen {
        align: center middle;
    }
    #help-box {
        background: $surface;
        border: thick $accent;
        padding: 1 2;
        width: 70%;
        height: 80%;
        overflow-y: auto;
    }
    """

    BINDINGS = [Binding("escape", "close", "Close"),
                Binding("question_mark", "close", "Close")]

    def __init__(self, keys: dict[str, str]) -> None:
        super().__init__()
        self.keys = keys

    def compose(self) -> ComposeResult:
        rows = ["[b]Key bindings[/b]", ""]
        for action, desc in ACTIONS.items():
            key = self.keys.get(action, "")
            rows.append(f"  [b]{key:<22}[/b]  {desc}")
        rows.append("")
        rows.append("Press [b]k[/b] (or the action you bound to 'rebind') to edit.")
        rows.append("Press [b]escape[/b] or [b]?[/b] to close.")
        yield Static("\n".join(rows), id="help-box")

    def action_close(self) -> None:
        self.dismiss(None)


# == main app =============================================================

class TMCApp(App):
    CSS = """
    Screen {
        layers: base;
    }
    #status {
        dock: top;
        height: 1;
        background: $boost;
        padding: 0 1;
    }
    #speed {
        height: auto;
        border: round $primary;
        padding: 0 1;
        margin: 0 0 1 0;
    }
    #speed .row {
        height: 3;
        align: left middle;
    }
    #speed Label {
        margin: 0 1 0 0;
    }
    #speed Input {
        width: 12;
    }
    #speed RadioSet {
        height: auto;
    }
    """

    poll_pos = reactive(True)

    def __init__(self, port_name: str) -> None:
        super().__init__()
        self.port_name = port_name
        self.port: serial.Serial | None = None
        self.joint_panels = [JointPanel(i) for i in range(NUM_JOINTS)]
        self.servo_panels = [ServoPanel(i) for i in range(NUM_SERVOS)]
        self.vmax = DEFAULT_VMAX
        self.amax = DEFAULT_AMAX
        self.jog_size = JOG_SIZES[1]
        self.keys = load_keys()

    # Bindings are installed dynamically in `_install_bindings`, not via the
    # class-level BINDINGS list, so the user's key-bind edits take effect
    # without restarting the app.

    def compose(self) -> ComposeResult:
        yield Header(show_clock=False)
        yield Static(f"port {self.port_name} (connecting...) - press ? for keys",
                     id="status")
        with Vertical():
            with Vertical(id="speed"):
                with Horizontal(classes="row"):
                    yield Label("vmax (usteps/s):")
                    yield Input(value=str(self.vmax), id="vmax", restrict=r"\d*")
                    yield Label("amax (usteps/s^2):")
                    yield Input(value=str(self.amax), id="amax", restrict=r"\d*")
                with Horizontal(classes="row"):
                    yield Label("default jog size:")
                    with RadioSet(id="jog_size"):
                        for size in JOG_SIZES:
                            yield RadioButton(str(size), value=(size == self.jog_size), id=f"sz_{size}")
            for p in self.joint_panels:
                yield p
            for s in self.servo_panels:
                yield s
        yield Footer()

    # -- lifecycle --------------------------------------------------------

    def on_mount(self) -> None:
        self.title = "TMC2209 Controller"
        self._install_bindings()
        try:
            self.port = serial.Serial(self.port_name, timeout=0.2)
        except serial.SerialException as e:
            self.set_status(f"port {self.port_name} ERROR: {e}", error=True)
            return
        self.set_status(f"port {self.port_name} connected")
        # Sync firmware state on connect: drivers off, all joints at 0 target,
        # servo configs pushed (firmware doesn't persist them).
        self.write(cmd_enable(0))
        for i in range(NUM_JOINTS):
            self.write(cmd_set_target(i, 0, self.vmax, self.amax))
        for s in range(NUM_SERVOS):
            self.write(cmd_set_servo_config(s, self.servo_panels[s].state.config))
        self.poll_loop()

    def on_unmount(self) -> None:
        if self.port and self.port.is_open:
            try:
                self.port.write(cmd_enable(0))
            except Exception:
                pass
            self.port.close()

    def _install_bindings(self) -> None:
        # `self.bind(key, action, ...)` overwrites any prior binding for the
        # same key. We don't try to drop bindings for keys removed by a rebind
        # (Textual's public API has no `unbind`); they linger but harmlessly
        # call missing actions, which Textual ignores.
        for action in ACTIONS:
            key = self.keys.get(action)
            if not key:
                continue
            self.bind(key, action, description=ACTIONS[action], show=False)

    # -- helpers ----------------------------------------------------------

    def set_status(self, msg: str, *, error: bool = False) -> None:
        s = self.query_one("#status", Static)
        s.update(("[red]" if error else "") + msg)

    def write(self, frame: bytes) -> None:
        if self.port is None or not self.port.is_open:
            return
        try:
            self.port.write(frame)
        except serial.SerialException as e:
            self.set_status(f"write failed: {e}", error=True)

    def enable_mask(self) -> int:
        mask = 0
        for i, p in enumerate(self.joint_panels):
            if p.state.enabled:
                mask |= 1 << i
        return mask

    def push_target(self, joint: int, target: int) -> None:
        self.joint_panels[joint].update_target(target)
        self.write(cmd_set_target(joint, target, self.vmax, self.amax))

    def push_servo_target(self, servo: int, target_us: int) -> None:
        cfg = self.servo_panels[servo].state.config
        target_us = max(cfg.min_us, min(cfg.max_us, target_us))
        self.servo_panels[servo].update_target(target_us)
        self.write(cmd_set_servo_target(servo, target_us))

    # -- polling ----------------------------------------------------------

    @work(exclusive=True, thread=True)
    def poll_loop(self) -> None:
        import time
        while self.poll_pos:
            if self.port is None or not self.port.is_open:
                time.sleep(0.2)
                continue
            try:
                self.port.reset_input_buffer()
                self.port.write(cmd_get_state())
                resp = self.port.read(STATE_SIZE)
            except serial.SerialException:
                time.sleep(0.2)
                continue
            result = parse_state(resp)
            if result is None:
                time.sleep(0.1)
                continue
            positions, servos, _flags = result
            self.call_from_thread(self._apply_state, positions, servos)
            time.sleep(0.05)

    def _apply_state(self, positions, servos) -> None:
        for i, pos in enumerate(positions):
            self.joint_panels[i].update_position(pos)
        for i, pos_us in enumerate(servos):
            self.servo_panels[i].update_position(pos_us)

    # -- widget events ----------------------------------------------------

    @on(Switch.Changed)
    def on_switch(self, event: Switch.Changed) -> None:
        sid = event.switch.id or ""
        if sid.startswith("en"):
            joint = int(sid[2:])
            self.joint_panels[joint].update_enabled(event.value)
            self.write(cmd_enable(self.enable_mask()))
        elif sid.startswith("tmc"):
            head, sfx = sid.split("_", 1)
            joint = int(head[3:])
            bit = next((b for s, _, b, _ in TMC_TOGGLES if s == sfx), 0)
            if bit == 0:
                return
            panel = self.joint_panels[joint]
            if event.value:
                panel.state.tmc_flags |= bit
            else:
                panel.state.tmc_flags &= ~bit
            self.write(cmd_set_tmc_config(joint, panel.state.tmc_flags))
        elif sid.startswith("shome"):
            servo = int(sid[5:])
            cfg = self.servo_panels[servo].state.config
            if event.value:
                cfg.flags |= SERVO_FLAG_HOMING
            else:
                cfg.flags &= ~SERVO_FLAG_HOMING

    @on(Input.Changed, "#vmax")
    def on_vmax(self, event: Input.Changed) -> None:
        try:
            self.vmax = max(1, int(event.value or "0"))
        except ValueError:
            pass

    @on(Input.Changed, "#amax")
    def on_amax(self, event: Input.Changed) -> None:
        try:
            self.amax = max(1, int(event.value or "0"))
        except ValueError:
            pass

    @on(RadioSet.Changed, "#jog_size")
    def on_jog_size(self, event: RadioSet.Changed) -> None:
        label = str(event.pressed.label).strip()
        try:
            self.jog_size = int(label)
        except ValueError:
            pass

    @on(Button.Pressed)
    def on_button(self, event: Button.Pressed) -> None:
        bid = event.button.id or ""
        if bid.startswith("home") and bid[4:].isdigit():
            joint = int(bid[4:])
            self.push_target(joint, 0)
        elif bid.startswith("stop"):
            joint = int(bid[4:])
            self.push_target(joint, self.joint_panels[joint].state.position)
        elif bid.startswith("go") and bid[2:].isdigit():
            joint = int(bid[2:])
            inp = self.query_one(f"#tgt{joint}", Input)
            try:
                target = int(inp.value or "0")
            except ValueError:
                return
            self.push_target(joint, target)
        elif bid.startswith("j") and "_" in bid:
            head, sign, size_s = bid.split("_")
            joint = int(head[1:])
            delta = int(size_s) if sign == "pos" else -int(size_s)
            new_target = self.joint_panels[joint].state.target + delta
            self.push_target(joint, new_target)
        elif bid.startswith("s") and bid[1].isdigit():
            self._handle_servo_button(bid)

    def _handle_servo_button(self, bid: str) -> None:
        # ids: s{n}_neg_{step}, s{n}_pos_{step}, s{n}_home, s{n}_go, s{n}_apply
        parts = bid.split("_")
        servo = int(parts[0][1:])
        op = parts[1]
        panel = self.servo_panels[servo]
        if op == "neg" or op == "pos":
            step = int(parts[2])
            delta = step if op == "pos" else -step
            self.push_servo_target(servo, panel.state.target_us + delta)
        elif op == "home":
            self.push_servo_target(servo, panel.state.config.home_us)
        elif op == "go":
            try:
                tgt = int(self.query_one(f"#stgt{servo}", Input).value or "0")
            except ValueError:
                return
            self.push_servo_target(servo, tgt)
        elif op == "apply":
            self._apply_servo_config(servo)

    def _apply_servo_config(self, servo: int) -> None:
        panel = self.servo_panels[servo]
        cfg = panel.state.config
        def _int(s: str, fallback: int) -> int:
            try:
                return int(s)
            except (TypeError, ValueError):
                return fallback
        cfg.min_us = _int(self.query_one(f"#smin{servo}", Input).value, cfg.min_us)
        cfg.max_us = _int(self.query_one(f"#smax{servo}", Input).value, cfg.max_us)
        cfg.deadzone_us = _int(self.query_one(f"#sdz{servo}", Input).value, cfg.deadzone_us)
        cfg.speed_us_per_s = _int(self.query_one(f"#sspd{servo}", Input).value, cfg.speed_us_per_s)
        cfg.home_us = _int(self.query_one(f"#shm{servo}", Input).value, cfg.home_us)
        if self.query_one(f"#shome{servo}", Switch).value:
            cfg.flags |= SERVO_FLAG_HOMING
        else:
            cfg.flags &= ~SERVO_FLAG_HOMING
        self.write(cmd_set_servo_config(servo, cfg))
        self.set_status(f"servo {servo} config applied")

    # -- actions (target of key bindings) --------------------------------
    # `action_quit` is provided by App and is async.

    def action_stop_all(self) -> None:
        for i, p in enumerate(self.joint_panels):
            self.push_target(i, p.state.position)
        self.set_status("STOP ALL")

    def action_toggle_all(self) -> None:
        new = not any(p.state.enabled for p in self.joint_panels)
        for i in range(NUM_JOINTS):
            self.query_one(f"#en{i}", Switch).value = new

    def action_help(self) -> None:
        self.push_screen(HelpScreen(self.keys))

    def action_rebind(self) -> None:
        def _on_close(new_keys: dict[str, str] | None) -> None:
            if not new_keys:
                return
            self.keys = new_keys
            save_keys(new_keys)
            self._install_bindings()
            self.set_status(f"keys saved to {KEYS_PATH}")
        self.push_screen(RebindScreen(self.keys), _on_close)

    # Per-joint jog/home actions ------------------------------------------

    def _jog(self, joint: int, sign: int) -> None:
        panel = self.joint_panels[joint]
        self.push_target(joint, panel.state.target + sign * self.jog_size)

    # Generated below: action_j{n}_neg/pos/home, action_s{n}_neg/pos/home.


def _attach_dynamic_actions() -> None:
    """Bolt per-joint and per-servo action methods onto the TMCApp class."""
    for j in range(NUM_JOINTS):
        def make(jj: int):
            def neg(self): self._jog(jj, -1)
            def pos(self): self._jog(jj, +1)
            def home(self): self.push_target(jj, 0)
            return neg, pos, home
        neg, pos, home = make(j)
        setattr(TMCApp, f"action_j{j}_neg", neg)
        setattr(TMCApp, f"action_j{j}_pos", pos)
        setattr(TMCApp, f"action_j{j}_home", home)
    for s in range(NUM_SERVOS):
        def make_s(ss: int):
            def neg(self): self.push_servo_target(ss, self.servo_panels[ss].state.target_us - 100)
            def pos(self): self.push_servo_target(ss, self.servo_panels[ss].state.target_us + 100)
            def home(self): self.push_servo_target(ss, self.servo_panels[ss].state.config.home_us)
            return neg, pos, home
        neg, pos, home = make_s(s)
        setattr(TMCApp, f"action_s{s}_neg", neg)
        setattr(TMCApp, f"action_s{s}_pos", pos)
        setattr(TMCApp, f"action_s{s}_home", home)


_attach_dynamic_actions()


def main() -> None:
    port_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    TMCApp(port_name).run()


if __name__ == "__main__":
    main()
