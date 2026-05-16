"""
Minimal host-side test for the tmc2209-fw firmware.

Usage:
    pixi run test [PORT]

Defaults to /dev/ttyACM0.  Loops J0 back and forth between 0 and 3200
µsteps until Ctrl+C.
"""

import struct
import sys
import time

import serial

DEFAULT_PORT = "/dev/ttyACM0"

VMAX = 10000
AMAX = 20000  # microsteps/s^2; needs to be high enough to reach VMAX in <1s
STEPS_PER_REV = 3200  # 200 full-steps x 16 ustep
NUM_JOINTS = 5
NUM_SERVOS = 2
# Frame: 0x81 + i32*NUM_JOINTS + u16*NUM_SERVOS + u8 flags
STATE_LEN = 1 + 4 * NUM_JOINTS + 2 * NUM_SERVOS + 1


# ── protocol helpers ─────────────────────────────────────────────────────

def cmd_enable(mask: int) -> bytes:
    """Enable drivers by bitmask (0b001 = J0, 0b010 = J1, 0b100 = J2)."""
    return struct.pack("<BB", 0x02, mask)


def cmd_set_target(joint: int, position: int, max_velocity: int, max_accel: int) -> bytes:
    """Command a joint to an absolute microstep position."""
    return struct.pack("<BBiII", 0x01, joint, position, max_velocity, max_accel)


def cmd_get_state() -> bytes:
    return b"\x04"


def parse_state(data: bytes):
    """Parse a StateReport: tag 0x81 + 5 i32 positions + 2 u16 servos + u8 flags."""
    if len(data) < STATE_LEN or data[0] != 0x81:
        return None
    positions = struct.unpack_from("<5i", data, 1)
    servos = struct.unpack_from("<2H", data, 1 + 4 * NUM_JOINTS)
    flags = data[1 + 4 * NUM_JOINTS + 2 * NUM_SERVOS]
    return positions, servos, flags


def wait_for_position(port, joint: int, target: int, timeout: float = 10.0):
    """Poll until joint reaches target position or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.1)
        port.reset_input_buffer()
        port.write(cmd_get_state())
        resp = port.read(STATE_LEN)
        result = parse_state(resp)
        if result is None:
            continue
        positions, servos, flags = result
        print(f"\r  pos={positions}  servos={servos}  flags=0x{flags:02x}",
              end="", flush=True)
        if positions[joint] == target:
            print("  ✓")
            return True
    print("  timeout!")
    return False


# ── main ─────────────────────────────────────────────────────────────────

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    print(f"opening {port_name}")
    port = serial.Serial(port_name, timeout=1)
    time.sleep(0.5)

    print("enabling J0")
    port.write(cmd_enable(0b001))
    time.sleep(0.2)

    cycle = 0
    try:
        while True:
            cycle += 1

            target = STEPS_PER_REV *10
            print(f"[{cycle}] → {target}")
            port.write(cmd_set_target(0, target, VMAX, AMAX))
            wait_for_position(port, 0, target)

            target = 0
            print(f"[{cycle}] → {target}")
            port.write(cmd_set_target(0, target, VMAX, AMAX))
            wait_for_position(port, 0, target)

    except KeyboardInterrupt:
        print("\nstopping")

    print("disabling drivers")
    port.write(cmd_enable(0x00))
    port.close()


if __name__ == "__main__":
    main()
