//! Pin map. Identical for the Pico 2 (RP2350A) and the original Pico (RP2040)
//! - both share the same 40-pin Pico footprint, so the GPIO numbers below
//! work on either board. The chip is selected at compile time via the
//! `rp2040` / `rp2350` cargo features.
//!
//! 5 joints + 2 servos. PIO0 hosts J0..J3 (SM0..SM3), PIO1 hosts J4 (SM0).
//! Two TMC2209 UART buses: UART1 carries J0..J3 (addresses 0..3 via MS1/MS2
//! straps); UART0 carries J4 alone (address 0). PWM slice 7 drives the two
//! servos on its A/B channels (one slice, shared 50 Hz period).
//!
//! Change these type aliases here and everything else follows. If you move to
//! a custom PCB, update this module and leave the rest of the firmware alone.

use embassy_rp::peripherals;

// TMC2209 UART buses.
pub type DriverUart0 = peripherals::UART0; // J4 alone, addr 0
pub type DriverUart0TxPin = peripherals::PIN_0;
pub type DriverUart0RxPin = peripherals::PIN_1;
pub type DriverUart1 = peripherals::UART1; // J0..J3, addr 0..3 via MS1/MS2
pub type DriverUart1TxPin = peripherals::PIN_8;
pub type DriverUart1RxPin = peripherals::PIN_9;

// Step-gen PIOs. PIO0 SM0..SM3 = J0..J3; PIO1 SM0 = J4.
pub type StepPio0 = peripherals::PIO0;
pub type StepPio1 = peripherals::PIO1;
pub type Step0Pin = peripherals::PIN_2;
pub type Step1Pin = peripherals::PIN_3;
pub type Step2Pin = peripherals::PIN_4;
pub type Step3Pin = peripherals::PIN_11;
pub type Step4Pin = peripherals::PIN_12;

// DIR pins - plain GPIO outputs.
pub type Dir0Pin = peripherals::PIN_5;
pub type Dir1Pin = peripherals::PIN_6;
pub type Dir2Pin = peripherals::PIN_7;
pub type Dir3Pin = peripherals::PIN_13;
// GP16 is the WS2812 on the Waveshare RP2040-Zero, so on that variant DIR J4
// is routed to GP26 (which is broken out on the castellated edge) instead.
#[cfg(not(feature = "rp2040-zero"))]
pub type Dir4Pin = peripherals::PIN_16;
#[cfg(feature = "rp2040-zero")]
pub type Dir4Pin = peripherals::PIN_26;

// Shared driver enable (active-low) for all 5 joints.
pub type EnablePin = peripherals::PIN_10;

// Servo PWM outputs. GP14 = PWM7 A, GP15 = PWM7 B - one slice, two channels.
pub type ServoSlice = peripherals::PWM_SLICE7;
pub type Servo0Pin = peripherals::PIN_14;
pub type Servo1Pin = peripherals::PIN_15;

// Heartbeat / status LED. On Pico/Pico 2 the onboard LED is GP25. On the
// RP2040-Zero there is no plain LED (GP16 drives the onboard WS2812 and
// isn't usable here), so we expose GP27 as an external-indicator pin.
#[cfg(not(feature = "rp2040-zero"))]
pub type LedPin = peripherals::PIN_25;
#[cfg(feature = "rp2040-zero")]
pub type LedPin = peripherals::PIN_27;

// Optional L298N H-bridge driving one brushed DC motor. IN1/IN2 set
// direction (and brake/coast), ENA gates motor current via PWM. ENA is on
// PWM slice 1 channel B (GP19); the servos already own slice 7. Not wired
// on rp2040-zero - GP17..GP19 are not broken out on its castellated edge.
#[cfg(all(feature = "l298n", not(feature = "rp2040-zero")))]
pub type L298nIn1Pin = peripherals::PIN_17;
#[cfg(all(feature = "l298n", not(feature = "rp2040-zero")))]
pub type L298nIn2Pin = peripherals::PIN_18;
#[cfg(all(feature = "l298n", not(feature = "rp2040-zero")))]
pub type L298nEnaPin = peripherals::PIN_19;
#[cfg(all(feature = "l298n", not(feature = "rp2040-zero")))]
pub type L298nPwmSlice = peripherals::PWM_SLICE1;
#[cfg(all(feature = "l298n", feature = "rp2040-zero"))]
compile_error!("feature `l298n` is not supported on rp2040-zero (GP17-GP19 are not broken out)");

// USB.
pub type Usb = peripherals::USB;

/// TMC2209 UART baud rate. The TMC2209 is tolerant of a wide range.
pub const TMC_BAUD: u32 = 9_600;
