#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1, UART0, UART1, USB};
use embassy_rp::pio::{InterruptHandler as PioIrq, Pio};
use embassy_rp::uart::{BufferedInterruptHandler as UartIrq, BufferedUart, Config as UartConfig};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbIrq};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use static_cell::StaticCell;
use {defmt as _, embassy_rp as _};

mod board;
mod driver;
mod motion;
mod protocol;
mod servo;
mod usb_cdc;

use crate::driver::{DriverBus, Drivers, JointId};
use crate::motion::MotionController;
use crate::protocol::Command;
use crate::servo::ServoController;

// RP2350 boots from a signed image block in flash; RP2040 boots from a
// stage-2 bootloader supplied by `embassy-rp` via the `boot2-*` feature.
#[cfg(chip_rp2350)]
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbIrq<USB>;
    PIO0_IRQ_0 => PioIrq<PIO0>;
    PIO1_IRQ_0 => PioIrq<PIO1>;
    UART1_IRQ => UartIrq<UART1>;
    UART0_IRQ => UartIrq<UART0>;
});

static COMMANDS: Channel<CriticalSectionRawMutex, Command, 8> = Channel::new();
static DRIVERS: StaticCell<Drivers> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("tmc2209-fw booting");

    // -- TMC2209 UART buses (two single-wire half-duplex buses). The chip's
    //    MS1/MS2 address straps only encode 4 addresses, so 5 drivers need
    //    2 buses. Primary (UART1, GP8/9) carries J0..=J3; secondary (UART0,
    //    GP0/1) carries J4 alone.
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = board::TMC_BAUD;

    static TX_BUF_P: StaticCell<[u8; 64]> = StaticCell::new();
    static RX_BUF_P: StaticCell<[u8; 64]> = StaticCell::new();
    let uart_primary = BufferedUart::new(
        p.UART1,
        p.PIN_8,
        p.PIN_9,
        Irqs,
        TX_BUF_P.init([0; 64]),
        RX_BUF_P.init([0; 64]),
        uart_cfg.clone(),
    );

    static TX_BUF_S: StaticCell<[u8; 64]> = StaticCell::new();
    static RX_BUF_S: StaticCell<[u8; 64]> = StaticCell::new();
    let uart_secondary = BufferedUart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        TX_BUF_S.init([0; 64]),
        RX_BUF_S.init([0; 64]),
        uart_cfg,
    );

    let drivers: &'static Drivers = DRIVERS.init(Drivers::new(
        DriverBus::new(uart_primary),
        DriverBus::new(uart_secondary),
    ));

    // -- PIO step-gen + motion controller. PIO0 carries J0..=J3 (4 SMs); PIO1
    //    SM0 carries J4. Both PIOs run the same step program.
    let pio0 = Pio::new(p.PIO0, Irqs);
    let pio1 = Pio::new(p.PIO1, Irqs);

    // -- Servos. PWM slice 7 drives GP14/GP15 at 50 Hz / 20 ms period.
    let servos = ServoController::new(p.PWM_SLICE7, p.PIN_14, p.PIN_15);

    let mut motion = MotionController::new(
        pio0, pio1, p.PIN_2, p.PIN_3, p.PIN_4, p.PIN_11, p.PIN_13, p.PIN_5, p.PIN_6, p.PIN_7,
        p.PIN_12, p.PIN_18, p.PIN_10, servos,
    );

    // -- USB CDC --
    let usb_driver = UsbDriver::new(p.USB, Irqs);

    // -- Spawn tasks --
    spawner.must_spawn(driver_init_task(drivers));
    spawner.must_spawn(usb_task(usb_driver, drivers));

    // Main task runs the motion control loop forever.
    motion.run(COMMANDS.receiver()).await
}

#[embassy_executor::task]
async fn driver_init_task(drivers: &'static Drivers) {
    for j in JointId::ALL {
        match drivers.apply_default_config(j).await {
            Ok(()) => defmt::info!("joint {} configured", j as u8),
            Err(e) => defmt::error!("joint {} config failed: {:?}", j as u8, e),
        }
    }
}

#[embassy_executor::task]
async fn usb_task(driver: UsbDriver<'static, USB>, drivers: &'static Drivers) {
    usb_cdc::run(driver, COMMANDS.sender(), drivers).await;
}
