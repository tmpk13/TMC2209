//! Pin map for the Pico 2 (RP2350A) dev board.
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

// Onboard LED (heartbeat).
pub type LedPin = peripherals::PIN_25;

// USB.
pub type Usb = peripherals::USB;

/// TMC2209 UART baud rate. The TMC2209 is tolerant of a wide range; 115200 is
/// a safe, jitter-friendly choice on a 133 MHz RP2350 core.
pub const TMC_BAUD: u32 = 115_200;
