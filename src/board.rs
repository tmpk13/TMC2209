//! Pin map. Identical for the Pico 2 (RP2350A) and the original Pico (RP2040)
//! - both share the same 40-pin Pico footprint, so the GPIO numbers below
//! work on either board. The chip is selected at compile time via the
//! `rp2040` / `rp2350` cargo features.
//!
//! Change these type aliases here and everything else follows. If you move to
//! a custom PCB, update this module and leave the rest of the firmware alone.

use embassy_rp::peripherals;

// TMC2209 half-duplex UART buses. Each shared single-wire bus supports up to
// 4 TMC2209 drivers (the chip's MS1/MS2 straps only encode 2 address bits),
// so 5 joints need 2 buses. Primary (UART1) carries J0..=J3, secondary
// (UART0) carries J4 alone.
pub type DriverUartPrimary = peripherals::UART1;
pub type DriverUartPrimaryTx = peripherals::PIN_8;
pub type DriverUartPrimaryRx = peripherals::PIN_9;
pub type DriverUartSecondary = peripherals::UART0;
pub type DriverUartSecondaryTx = peripherals::PIN_0;
pub type DriverUartSecondaryRx = peripherals::PIN_1;

// Step-gen PIO blocks. Joints 0..=3 live on PIO0 (one state machine each).
// Joint 4 lives on PIO1 SM0 because PIO0 only has 4 state machines.
pub type StepPio0 = peripherals::PIO0;
pub type StepPio1 = peripherals::PIO1;
pub type Step0Pin = peripherals::PIN_2;
pub type Step1Pin = peripherals::PIN_3;
pub type Step2Pin = peripherals::PIN_4;
pub type Step3Pin = peripherals::PIN_11;
pub type Step4Pin = peripherals::PIN_13;

// DIR pins - plain GPIO outputs.
pub type Dir0Pin = peripherals::PIN_5;
pub type Dir1Pin = peripherals::PIN_6;
pub type Dir2Pin = peripherals::PIN_7;
pub type Dir3Pin = peripherals::PIN_12;
pub type Dir4Pin = peripherals::PIN_18;

// Shared driver enable (active-low).
pub type EnablePin = peripherals::PIN_10;

// Servos. Both share PWM slice 7 (channels A=GP14, B=GP15) so they run off
// the same 50 Hz period clock; compare values are set independently.
pub type ServoPwm = peripherals::PWM_SLICE7;
pub type Servo0Pin = peripherals::PIN_14;
pub type Servo1Pin = peripherals::PIN_15;

// Onboard LED (heartbeat).
pub type LedPin = peripherals::PIN_25;

// USB.
pub type Usb = peripherals::USB;

/// TMC2209 UART baud rate. The TMC2209 is tolerant of a wide range; 115200 is
/// a safe, jitter-friendly choice on a 133 MHz RP2350 core.
pub const TMC_BAUD: u32 = 115_200;

/// Servo PWM tick rate in microseconds per tick.
///
/// We pick a divider that makes one PWM count equal one microsecond, so the
/// `compare` register reads directly in microseconds of pulse width. The RP2040
/// core clock is 125 MHz and the RP2350 default is 150 MHz - both close enough
/// that a constant divider works (the small jitter doesn't matter for hobby
/// servos). With `top = 19_999` the period is 20 ms (50 Hz).
#[cfg(chip_rp2040)]
pub const SERVO_DIV: u8 = 125;
#[cfg(chip_rp2350)]
pub const SERVO_DIV: u8 = 150;
pub const SERVO_TOP: u16 = 19_999;
