#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
#[cfg(feature = "uart")]
use embassy_rp::peripherals::{UART0, UART1};
use embassy_rp::peripherals::{PIO0, PIO1, USB};
use embassy_rp::pio::{InterruptHandler as PioIrq, Pio};
#[cfg(feature = "uart")]
use embassy_rp::uart::{BufferedInterruptHandler as UartIrq, BufferedUart, Config as UartConfig};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbIrq};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use static_cell::StaticCell;
use {defmt as _, embassy_rp as _};

mod board;
mod driver;
#[cfg(feature = "l298n")]
mod l298n;
mod motion;
mod protocol;
mod servo;
mod usb_cdc;

use crate::driver::DriverBus;
#[cfg(feature = "uart")]
use crate::driver::JointId;
use crate::motion::MotionController;
use crate::protocol::Command;

// RP2350 boots from a signed image block in flash; RP2040 boots from a
// stage-2 bootloader supplied by `embassy-rp` via the `boot2-*` feature.
#[cfg(chip_rp2350)]
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

#[cfg(feature = "uart")]
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbIrq<USB>;
    PIO0_IRQ_0 => PioIrq<PIO0>;
    PIO1_IRQ_0 => PioIrq<PIO1>;
    UART0_IRQ => UartIrq<UART0>;
    UART1_IRQ => UartIrq<UART1>;
});

#[cfg(not(feature = "uart"))]
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbIrq<USB>;
    PIO0_IRQ_0 => PioIrq<PIO0>;
    PIO1_IRQ_0 => PioIrq<PIO1>;
});

static COMMANDS: Channel<CriticalSectionRawMutex, Command, 8> = Channel::new();
static DRIVER_BUS: StaticCell<DriverBus> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("tmc2209-fw booting");

    // -- TMC2209 UART buses ------------------------------------------------
    // UART1 carries J0..J3 (addresses 0..=3); UART0 carries J4 alone.
    #[cfg(feature = "uart")]
    let bus: &'static DriverBus = {
        static TX0_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        static RX0_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        static TX1_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        static RX1_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let mut uart_cfg = UartConfig::default();
        uart_cfg.baudrate = board::TMC_BAUD;
        let uart0 = BufferedUart::new(
            p.UART0,
            p.PIN_0,
            p.PIN_1,
            Irqs,
            TX0_BUF.init([0; 64]),
            RX0_BUF.init([0; 64]),
            uart_cfg,
        );
        let uart1 = BufferedUart::new(
            p.UART1,
            p.PIN_8,
            p.PIN_9,
            Irqs,
            TX1_BUF.init([0; 64]),
            RX1_BUF.init([0; 64]),
            uart_cfg,
        );
        DRIVER_BUS.init(DriverBus::new(uart0, uart1))
    };
    #[cfg(not(feature = "uart"))]
    let bus: &'static DriverBus = DRIVER_BUS.init(DriverBus::new());

    // -- PIO step-gen + motion controller ----------------------------------
    // PIO0 hosts J0..J3 on SM0..SM3; PIO1 hosts J4 on SM0.
    let pio0 = Pio::new(p.PIO0, Irqs);
    let pio1 = Pio::new(p.PIO1, Irqs);
    #[cfg(not(feature = "rp2040-zero"))]
    let dir4 = p.PIN_16;
    #[cfg(feature = "rp2040-zero")]
    let dir4 = p.PIN_26;
    let mut motion = MotionController::new(
        pio0, pio1,
        p.PIN_2, p.PIN_3, p.PIN_4, p.PIN_11, p.PIN_12,
        p.PIN_5, p.PIN_6, p.PIN_7, p.PIN_13, dir4,
        p.PIN_10,
    );

    // -- Servo PWM (slice 7, GP14 ch A, GP15 ch B) ------------------------
    servo::spawn(spawner, p.PWM_SLICE7, p.PIN_14, p.PIN_15);

    // -- Optional L298N brushed DC motor (slice 1 ch B, GP17/18 dir) ------
    #[cfg(feature = "l298n")]
    l298n::spawn(spawner, p.PWM_SLICE1, p.PIN_19, p.PIN_17, p.PIN_18);

    // -- USB CDC ----------------------------------------------------------
    let usb_driver = UsbDriver::new(p.USB, Irqs);

    // -- Spawn tasks ------------------------------------------------------
    #[cfg(feature = "uart")]
    spawner.must_spawn(driver_init_task(bus));
    spawner.must_spawn(usb_task(usb_driver, bus));

    // Main task runs the motion control loop forever.
    motion.run(COMMANDS.receiver()).await
}

#[cfg(feature = "uart")]
#[embassy_executor::task]
async fn driver_init_task(bus: &'static DriverBus) {
    for j in JointId::ALL {
        match bus.apply_default_config(j).await {
            Ok(()) => defmt::info!("joint {} configured", j as u8),
            Err(e) => defmt::error!("joint {} config failed: {:?}", j as u8, e),
        }
    }
}

#[embassy_executor::task]
async fn usb_task(driver: UsbDriver<'static, USB>, bus: &'static DriverBus) {
    usb_cdc::run(driver, COMMANDS.sender(), bus).await;
}
