#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, UART1, USB};
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
mod usb_cdc;

use crate::driver::{DriverBus, JointId};
use crate::motion::MotionController;
use crate::protocol::Command;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbIrq<USB>;
    PIO0_IRQ_0 => PioIrq<PIO0>;
    UART1_IRQ => UartIrq<UART1>;
});

static COMMANDS: Channel<CriticalSectionRawMutex, Command, 8> = Channel::new();
static DRIVER_BUS: StaticCell<DriverBus> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("tmc2209-fw booting");

    // ── TMC2209 UART bus (shared half-duplex) ─────────────────────────────
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = board::TMC_BAUD;
    let uart = BufferedUart::new(
        p.UART1,
        p.PIN_8,
        p.PIN_9,
        Irqs,
        TX_BUF.init([0; 64]),
        RX_BUF.init([0; 64]),
        uart_cfg,
    );
    let bus: &'static DriverBus = DRIVER_BUS.init(DriverBus::new(uart));

    // ── PIO step-gen + motion controller ──────────────────────────────────
    let pio = Pio::new(p.PIO0, Irqs);
    let mut motion = MotionController::new(
        pio, p.PIN_2, p.PIN_3, p.PIN_4, p.PIN_5, p.PIN_6, p.PIN_7, p.PIN_10,
    );

    // ── USB CDC ───────────────────────────────────────────────────────────
    let usb_driver = UsbDriver::new(p.USB, Irqs);

    // ── Spawn tasks ───────────────────────────────────────────────────────
    spawner.must_spawn(driver_init_task(bus));
    spawner.must_spawn(usb_task(usb_driver));

    // Main task runs the motion control loop forever.
    motion.run(COMMANDS.receiver()).await
}

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
async fn usb_task(driver: UsbDriver<'static, USB>) {
    usb_cdc::run(driver, COMMANDS.sender()).await;
}
