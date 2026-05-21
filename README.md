# TMC2209 arm firmware

![TMC2209](imgs/tmc2209.png)

Firmware for a 5-axis arm + 2-servo gripper:

- **J0..J2** three main linkages (TMC2209 over UART1, PIO0 SM0..SM2)
- **J3**    base rotation pivot (TMC2209 over UART1, PIO0 SM3)
- **J4**    linear stage (TMC2209 over UART0, PIO1 SM0)
- **S0/S1** end-effector servos (PWM slice 7, 50 Hz)

Two TMC UART buses are needed because the TMC2209's MS1/MS2 strap pins only
encode 4 addresses. UART1 carries J0..J3 (addresses 0..3); UART0 carries J4
alone (address 0).

Builds for the **Raspberry Pi Pico 2 (RP2350)** and the original **Pico
(RP2040)** from the same source. Pin assignments live in
[src/board.rs](src/board.rs) - change them there and nothing else.

## Build & flash

```sh
cargo build-pico2 --release   # RP2350, default
cargo build-pico  --release   # RP2040 (original Pico, 40-pin)
cargo build-zero  --release   # RP2040-Zero (only GPIO 0..=15, 26..=29 exposed)
```

`cargo run-*` invokes `elf2uf2-rs` to produce `firmware.uf2`. Hold BOOTSEL
when plugging in the board, then:

```sh
cp firmware.uf2      /media/$USER/RP2350/    # Pico 2
cp firmware-pico.uf2 /media/$USER/RPI-RP2/   # Pico
```

The device re-enumerates as `16c0:27dd` (`tmc2209-fw`) - usually
`/dev/ttyACM0`. With an SWD probe, uncomment the `probe-rs run` runner in
`.cargo/config.toml` to flash + stream defmt directly.

One-time setup:

```sh
rustup target add thumbv8m.main-none-eabihf  # RP2350
rustup target add thumbv6m-none-eabi         # RP2040
cargo install elf2uf2-rs
```

## Pin map

Pico / Pico 2 share the 40-pin footprint and use the same assignments. The
Waveshare RP2040-Zero only breaks out GP0..=15 and GP26..=29 (GP16 drives the
onboard WS2812), so two pins are remapped under `--features rp2040-zero`.

| Signal          | Pico / Pico 2 | Pico Zero (`rp2040-zero`) | Notes |
|-----------------|---------------|---------------------------|-------|
| STEP0/1/2       | GP2/3/4       | GP2/3/4                   | PIO0 SM0..SM2 (linkages) |
| STEP3           | GP11          | GP11                      | PIO0 SM3 (base rotation) |
| STEP4           | GP12          | GP12                      | PIO1 SM0 (linear stage)  |
| DIR0/1/2        | GP5/6/7       | GP5/6/7                   | |
| DIR3            | GP13          | GP13                      | |
| DIR4            | GP16          | **GP26**                  | GP16 is the WS2812 on Pico Zero, so DIR4 moves to GP26 |
| TMC UART1 TX/RX | GP8/9         | GP8/9                     | primary bus (J0..J3); on bare-chip wiring tie via 1k for single-wire, BTT-style breakouts expose split TX/RX so wire MCU-TX -> breakout-RX, MCU-RX -> breakout-TX |
| TMC UART0 TX/RX | GP0/1         | GP0/1                     | secondary bus (J4); same wiring rules as primary |
| EN (shared)     | GP10          | GP10                      | active-low, starts disabled; for bringup you can tie EN to GND directly to keep all drivers permanently enabled |
| Servo0/Servo1   | GP14/15       | GP14/15                   | PWM slice 7 A/B, 50 Hz |
| LED             | GP25          | **GP27**                  | GP25 isn't broken out on Pico Zero; GP27 is exposed as an external-indicator pin |
| L298N IN1/IN2   | GP17/18       | -                         | direction (only with `--features l298n`; Pico Zero doesn't expose these) |
| L298N ENA       | GP19          | -                         | PWM slice 1 ch B, ~10 kHz speed input |

## TMC2209 single-wire UART buses

Two physical buses, identically wired. On each bus the MCU TX and RX meet
through a 1k series resistor; an optional 10k pull-up to VIO keeps the bus
idle-high.

```
  Primary (J0..J3):
    GP8 --+-- 1k --+--> PDN_UART (J0, J1, J2, J3 all tied)
    GP9 --+        +

  Secondary (J4):
    GP0 --+-- 1k --+--> PDN_UART (J4)
    GP1 --+        +
```

The TMC2209's MS1/MS2 straps fix the UART address at power-up. On the
primary bus J0..J3 take addresses 0..3; on the secondary bus J4 takes
address 0 (it's alone).

| MS2 | MS1 | UART address | Primary bus | Secondary bus |
|-----|-----|--------------|-------------|---------------|
| GND | GND | 0            | J0          | J4            |
| GND | VIO | 1            | J1          | -             |
| VIO | GND | 2            | J2          | -             |
| VIO | VIO | 3            | J3          | -             |

Tie each strap directly to GND or VIO (no pull resistor needed). The same
pins also set the microstep resolution in STEP/DIR mode, but in UART mode
microsteps are programmed over the bus so the straps are address-only.

Baud is 115200 on both buses. The firmware tolerates the echo of its own TX
bytes transparently via `tmc2209::Reader`.

## Optional brushed DC motor on an L298N (`--features l298n`)

Alongside the TMC2209 steppers the firmware can drive one brushed DC motor
through an external L298N dual H-bridge. Half of the L298N (channel A) is
used; the other half is free for a second motor later.

Wiring (one motor on the A side of the L298N):

```
  GP17 -> IN1
  GP18 -> IN2
  GP19 -> ENA          (PWM speed, ~10 kHz)
  GND  -> L298N GND    (must share ground with the Pico)
  +VM  -> L298N +12V   (motor supply, separate from logic 5V)
  OUT1/OUT2 -> motor terminals
```

Leave the ENA jumper off if your L298N has one. Tie L298N's logic +5V to a
5 V supply (USB 5 V on the Pico carrier works for most clones, but check
your board's silkscreen - some need an external 5 V). The Pico's ground
**must** be common with the motor PSU ground or PWM edges won't be seen.

Host commands the motor with `SetDcMotor { id: 0, duty: i16 }` where
`duty` is signed milli-units in `-1000..=+1000`. Sign sets direction,
magnitude sets PWM duty (0 = coast). Builds without `--features l298n`
still decode the frame so the host protocol is uniform; they just drop it.

Not available on `rp2040-zero` - GP17-GP19 are not broken out on its
castellated edge, and enabling the feature there is a compile error.

## Motor power

Per driver: VM = 4.75-29 V (12-24 V typical), 100 uF cap VM<->GND, A1/A2 and
B1/B2 to the motor coils. Set `IRUN` / `IHOLD` for your sense resistor and
motor in [src/driver.rs](src/driver.rs::apply_default_config) (currently
`IRUN=16, IHOLD=8`).

## Servo wiring

Both servos take signal from GP14/GP15, +5 V from the servo PSU (**not** the
Pico's 3V3), and common ground with the Pico. The firmware drives 50 Hz / 20
ms PWM with the divider arranged so one PWM count equals one microsecond -
the host commands a pulse width in microseconds directly.

## Host tools

Python tools live in [host/](host/) and use `pixi`:

```sh
pixi run tui [/dev/ttyACM0]   # Textual jog + servo + TMC config TUI
pixi run web [/dev/ttyACM0]   # FastHTML + p5.js browser UI on :8000
pixi run ik  X Y Z            # solve URDF-based IK and print motor targets
                              # (add --port to also stream over serial)
```

The TUI's keyboard shortcuts are configurable: press `?` to see them, `k` to
open the rebind modal. The bindings persist in
`~/.config/tmc2209-tui/keys.json`.

## URDF / IK

Export the arm from Onshape with
[`onshape-to-robot`](https://github.com/Rhoban/onshape-to-robot), drop
`robot.urdf` into [host/urdf/](host/urdf/), and edit
[host/urdf/motor_map.json](host/urdf/motor_map.json) so URDF joint names map
to firmware motor indices with the right unit scales (microsteps per radian /
per metre). `pixi run ik X Y Z` then solves IK with ikpy and emits motor
targets; pass `--port /dev/ttyACM0` to stream them as `SetTarget` frames.

The two gripper servos are outside the IK chain - drive them directly from
the TUI.

## Host protocol (USB-CDC, binary, little-endian)

| tag  | direction | payload                                                              |
|------|-----------|----------------------------------------------------------------------|
| 0x01 | host->fw  | SetTarget { joint:u8, pos:i32, vmax:u32, amax:u32 }                  |
| 0x02 | host->fw  | Enable { mask:u8 } (bit i = joint i, 5 bits)                         |
| 0x03 | host->fw  | Home { joint:u8 }                                                    |
| 0x04 | host->fw  | GetState -> 0x81                                                     |
| 0x05 | host->fw  | SetTmcConfig { joint:u8, flags:u8 } (stealthChop/interp/invert)      |
| 0x06 | host->fw  | SetPosition { joint:u8, pos:i32 } (relabel origin)                   |
| 0x07 | host->fw  | GetTmcConfig { joint:u8 } -> 0x83                                    |
| 0x08 | host->fw  | GetVersion -> 0x84                                                   |
| 0x09 | host->fw  | SetServoTarget { id:u8, target_us:u16 }                              |
| 0x0A | host->fw  | SetServoConfig { id, min, max, deadzone, speed, home_us, flags }     |
| 0x0B | host->fw  | SetDcMotor { id:u8, duty:i16 } (-1000..=+1000; L298N option)        |
| 0x81 | fw->host  | StateReport { positions:[i32;5], servos:[u16;2], flags:u8 }          |
| 0x83 | fw->host  | TmcConfigReport (GCONF + CHOPCONF + rx trace for diagnostics)        |
| 0x84 | fw->host  | VersionReport (chip id, build epoch, git short rev)                  |

Servo config is owned by the host - the firmware does not persist it, so the
TUI pushes `SetServoConfig` on every connect.

## Design notes

Non-obvious choices, gotchas, and the rationale behind the protocol/pinout
live in [NOTES.md](NOTES.md).


