//! Pin map. Identical for the Pico 2 (RP2350A) and the original Pico (RP2040)
//! — both share the same 40-pin Pico footprint, so the GPIO numbers below
//! work on either board. The chip is selected at compile time via the
//! `rp2040` / `rp2350` cargo features.
//!
//! Change these type aliases here and everything else follows. If you move to
//! a custom PCB, update this module and leave the rest of the firmware alone.

use embassy_rp::peripherals;

// Shared TMC2209 half-duplex UART (all 3 drivers on the same wire, addressed
// 0..=2 via MS1/MS2 straps).
pub type DriverUart = peripherals::UART1;
pub type DriverUartTxPin = peripherals::PIN_8;
pub type DriverUartRxPin = peripherals::PIN_9;

// Step-gen PIO block.
pub type StepPio = peripherals::PIO0;
pub type Step0Pin = peripherals::PIN_2;
pub type Step1Pin = peripherals::PIN_3;
pub type Step2Pin = peripherals::PIN_4;

// DIR pins — plain GPIO outputs.
pub type Dir0Pin = peripherals::PIN_5;
pub type Dir1Pin = peripherals::PIN_6;
pub type Dir2Pin = peripherals::PIN_7;

// Shared driver enable (active-low).
pub type EnablePin = peripherals::PIN_10;

// Onboard LED (heartbeat). On Pico/Pico 2 this is GP25; on the RP2040-Zero
// GP25 isn't broken out (and GP16 drives the onboard WS2812), so route the
// heartbeat to GP26 which sits in the castellated-edge range.
#[cfg(not(feature = "rp2040-zero"))]
pub type LedPin = peripherals::PIN_25;
#[cfg(feature = "rp2040-zero")]
pub type LedPin = peripherals::PIN_26;

// USB.
pub type Usb = peripherals::USB;

/// TMC2209 UART baud rate. The TMC2209 is tolerant of a wide range.
pub const TMC_BAUD: u32 = 9_600;
