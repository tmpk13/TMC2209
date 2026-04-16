//! USB CDC-ACM command interface.
//!
//! The RP2350 enumerates as a CDC serial device. The host (Isaac Sim bridge)
//! sends fixed-size command records; the firmware replies with `StateReport`
//! frames on demand.

use core::sync::atomic::Ordering;

use embassy_futures::join::join;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

use crate::motion::{NUM_JOINTS, POSITIONS};
use crate::protocol::{self, encode_state, Command, StateReport};

pub type CommandSender = Sender<'static, CriticalSectionRawMutex, Command, 8>;

/// Entry point for the USB task. Owns the `Driver` (USB peripheral) and the
/// outgoing command channel to the motion task.
pub async fn run(usb_driver: Driver<'static, USB>, commands: CommandSender) -> ! {
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let mut config = Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("tmpk");
    config.product = Some("tmc2209-fw");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        usb_driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let mut class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    let mut usb = builder.build();

    let usb_fut = usb.run();
    let cmd_fut = async {
        loop {
            class.wait_connection().await;
            defmt::info!("usb connected");
            let _ = read_loop(&mut class, commands).await;
            defmt::info!("usb disconnected");
        }
    };

    join(usb_fut, cmd_fut).await;
    unreachable!()
}

async fn read_loop(
    class: &mut CdcAcmClass<'static, Driver<'static, USB>>,
    commands: CommandSender,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut rx = [0u8; 64];
    let mut tx = [0u8; 32];
    loop {
        let n = class.read_packet(&mut rx).await?;
        if n == 0 {
            continue;
        }
        match protocol::decode(&rx[..n]) {
            Ok(Command::GetState) => {
                let report = current_state();
                let len = encode_state(&report, &mut tx);
                class.write_packet(&tx[..len]).await?;
            }
            Ok(cmd) => {
                // Non-blocking send — drop if the motion task is backed up.
                let _ = commands.try_send(cmd);
            }
            Err(e) => defmt::warn!("bad frame: {:?}", defmt::Debug2Format(&e)),
        }
    }
}

fn current_state() -> StateReport {
    let mut positions = [0i32; NUM_JOINTS];
    for (i, slot) in positions.iter_mut().enumerate() {
        *slot = POSITIONS[i].load(Ordering::Relaxed);
    }
    StateReport { positions, flags: 0 }
}
