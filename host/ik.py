"""
URDF-driven inverse kinematics for the tmc2209-fw arm.

Workflow:
    1. Model the arm in Onshape.
    2. Export to URDF with `onshape-to-robot` (https://github.com/Rhoban/onshape-to-robot):
           pixi run onshape-to-robot /path/to/your/cad
       This writes `robot.urdf` plus mesh files into the cad directory.
    3. Drop the .urdf (and optionally meshes) into `host/urdf/robot.urdf`.
    4. Edit `host/urdf/motor_map.json` so the IK joint names line up with the
       firmware's motor indices (J0..J4) and units (microsteps per radian for
       revolute joints, microsteps per meter for the prismatic linear stage).
    5. `pixi run ik X Y Z` solves IK and prints / streams motor targets.

Library entry points:
    Robot.load(urdf_path, motor_map_path)
    Robot.solve_ik(xyz, initial_position=None, orientation=None) -> joint values
    Robot.to_motor_targets(joint_values) -> {joint_idx: microsteps}
    Robot.send(port, joint_values) -> writes SetTarget frames over serial

The library only depends on ikpy and numpy at runtime. ikpy reads URDF
directly so there is no separate parser to maintain.
"""

from __future__ import annotations

import json
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

try:
    import numpy as np
    from ikpy.chain import Chain
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        f"ik.py needs ikpy and numpy: pixi add --pypi ikpy numpy  ({e})"
    )


DEFAULT_URDF = Path(__file__).parent / "urdf" / "robot.urdf"
DEFAULT_MOTOR_MAP = Path(__file__).parent / "urdf" / "motor_map.json"

# Tags that mirror src/protocol.rs.
TAG_SET_TARGET = 0x01


@dataclass
class JointMap:
    """How one URDF joint maps to one firmware motor.

    `scale` converts the URDF joint value (radians for revolute, metres for
    prismatic) to microsteps. For a stepper with 200 steps/rev at 16x
    microstepping coupled 1:1, that is `200*16 / (2*pi) ~= 509.296` for a
    revolute joint. Sign flips here are how you tell the firmware to invert
    direction without changing wiring (cf. TMC_FLAG_SHAFT_INVERT).
    """

    motor: int
    scale: float
    offset_us: int = 0
    """Microsteps added after scaling - moves the joint's zero off the URDF
    zero (useful when the mechanical home isn't at the modelled zero pose)."""
    vmax: int = 10_000
    amax: int = 300


@dataclass
class Robot:
    chain: Any
    motor_map: dict[str, JointMap]
    active_links_mask: list[bool] = field(default_factory=list)

    @classmethod
    def load(cls, urdf_path: Path = DEFAULT_URDF,
             motor_map_path: Path = DEFAULT_MOTOR_MAP) -> "Robot":
        if not urdf_path.exists():
            raise FileNotFoundError(
                f"URDF not found: {urdf_path}. Export from Onshape with "
                "onshape-to-robot and drop the file there."
            )
        if not motor_map_path.exists():
            raise FileNotFoundError(
                f"motor_map.json not found: {motor_map_path}. See ik.py "
                "docstring for the expected format."
            )
        raw_map = json.loads(motor_map_path.read_text())
        motor_map = {
            name: JointMap(**spec) for name, spec in raw_map["joints"].items()
        }
        # Build an "active links mask" so ikpy only treats the URDF joints we
        # actually drive as IK degrees of freedom. Inactive fixed joints (e.g.
        # the gripper) stay locked at zero.
        base_elements = raw_map.get("base_elements")
        chain = Chain.from_urdf_file(str(urdf_path),
                                     base_elements=base_elements) \
            if base_elements else Chain.from_urdf_file(str(urdf_path))
        active_mask = [
            (link.name in motor_map) if hasattr(link, "name") else False
            for link in chain.links
        ]
        # ikpy requires the very first link (the root "Base link") to be
        # inactive, regardless of its name.
        if active_mask:
            active_mask[0] = False
        chain.active_links_mask = active_mask
        return cls(chain=chain, motor_map=motor_map, active_links_mask=active_mask)

    @property
    def n_links(self) -> int:
        return len(self.chain.links)

    def home_position(self) -> "np.ndarray":
        return np.zeros(self.n_links)

    def solve_ik(self, xyz: Iterable[float],
                 initial_position: "np.ndarray | None" = None,
                 orientation: "np.ndarray | None" = None,
                 orientation_mode: str | None = None) -> "np.ndarray":
        """Inverse-kinematics solve. Returns a vector indexed by URDF link
        (length = n_links). Use `to_motor_targets` to convert to firmware
        microsteps for the motors you actually drive."""
        target = np.asarray(list(xyz), dtype=float)
        init = initial_position if initial_position is not None else self.home_position()
        if orientation is not None:
            return self.chain.inverse_kinematics(
                target_position=target,
                target_orientation=np.asarray(orientation, dtype=float),
                orientation_mode=orientation_mode or "all",
                initial_position=init,
            )
        return self.chain.inverse_kinematics(target_position=target,
                                             initial_position=init)

    def forward_kinematics(self, joint_values: "np.ndarray") -> "np.ndarray":
        """Convenience: returns the 4x4 end-effector pose for visualisation /
        sanity checks. Useful to verify the URDF zero pose is what you expect
        before plumbing the IK targets through to the motors."""
        return self.chain.forward_kinematics(joint_values)

    def to_motor_targets(self, joint_values: "np.ndarray") -> dict[int, int]:
        """Map URDF joint values to firmware motor microstep targets via the
        motor_map. Missing joints (gripper, fixed) are skipped."""
        out: dict[int, int] = {}
        for idx, link in enumerate(self.chain.links):
            spec = self.motor_map.get(getattr(link, "name", ""))
            if spec is None:
                continue
            steps = int(round(spec.scale * float(joint_values[idx]))) + spec.offset_us
            out[spec.motor] = steps
        return out

    def joint_specs(self) -> dict[int, JointMap]:
        """motor index -> JointMap (so callers can read vmax/amax per motor)."""
        return {spec.motor: spec for spec in self.motor_map.values()}

    def send(self, port, joint_values: "np.ndarray") -> None:
        """Push SetTarget frames for every mapped motor over the firmware's
        USB serial. `port` is a pyserial.Serial-like writer."""
        for motor, steps in self.to_motor_targets(joint_values).items():
            spec = self.joint_specs()[motor]
            frame = struct.pack("<BBiII", TAG_SET_TARGET, motor,
                                steps, spec.vmax, spec.amax)
            port.write(frame)


# == CLI demo =============================================================
#
# Usage:
#   pixi run ik X Y Z                 # solve and print motor commands
#   pixi run ik --port /dev/ttyACM0 X Y Z   # solve and send over USB

def _main(argv: list[str]) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Solve IK for an end-effector "
                                             "target and emit motor commands.")
    ap.add_argument("xyz", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="Target position in metres, robot base frame.")
    ap.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    ap.add_argument("--map", type=Path, default=DEFAULT_MOTOR_MAP, dest="map_path")
    ap.add_argument("--port", type=str, default=None,
                    help="If given, stream motor commands to this serial port.")
    args = ap.parse_args(argv)

    robot = Robot.load(args.urdf, args.map_path)
    joint_values = robot.solve_ik(args.xyz)
    ee = robot.forward_kinematics(joint_values)
    print(f"target xyz      : {args.xyz}")
    print(f"achieved xyz    : {ee[:3, 3].tolist()}")
    targets = robot.to_motor_targets(joint_values)
    for motor in sorted(targets):
        print(f"  motor {motor} -> {targets[motor]:+d} usteps")

    if args.port:
        import serial
        with serial.Serial(args.port, timeout=0.2) as ser:
            robot.send(ser, joint_values)
        print(f"sent to {args.port}")
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
