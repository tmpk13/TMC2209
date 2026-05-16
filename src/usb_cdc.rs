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

use tmc2209::reg;

use crate::driver::{Drivers, Error as DriverError, JointId};
use crate::motion::{NUM_JOINTS, NUM_SERVOS, POSITIONS};
use crate::protocol::{
    self, encode_state, encode_tmc_config, encode_version, Command, StateReport, TmcConfigReport,
    TmcConfigStatus, VersionReport, RX_TRACE_LEN,
};
#[cfg(chip_rp2040)]
use crate::protocol::CHIP_ID_RP2040;
#[cfg(chip_rp2350)]
use crate::protocol::CHIP_ID_RP2350;
use crate::servo::SERVO_POSITIONS;

/// Compile-time chip id, picked by the same `chip_*` cfg the linker uses.
#[cfg(chip_rp2040)]
const CHIP_ID: u8 = CHIP_ID_RP2040;
#[cfg(chip_rp2350)]
const CHIP_ID: u8 = CHIP_ID_RP2350;

/// Build epoch (Unix seconds) from build.rs. `parse::<u32>()` is `const`-safe
/// in newer toolchains but not const here, so we parse at startup the first
/// time it's needed; the value is fixed for the binary's lifetime.
const BUILD_EPOCH_STR: &str = env!("BUILD_EPOCH");
const GIT_SHORT: &str = env!("GIT_SHORT");

fn version_report() -> VersionReport {
    let epoch = BUILD_EPOCH_STR.parse::<u32>().unwrap_or(0);
    let mut git = [0u8; 12];
    let bytes = GIT_SHORT.as_bytes();
    let n = bytes.len().min(git.len());
    git[..n].copy_from_slice(&bytes[..n]);
    VersionReport { chip: CHIP_ID, epoch, git }
}

pub type CommandSender = Sender<'static, CriticalSectionRawMutex, Command, 8>;

/// Entry point for the USB task. Owns the `Driver` (USB peripheral) and the
/// outgoing command channel to the motion task. The `Drivers` aggregator is
/// needed so `SetTmcConfig` can re-write the TMC2209 registers inline.
pub async fn run(
    usb_driver: Driver<'static, USB>,
    commands: CommandSender,
    drivers: &'static Drivers,
) -> ! {
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
            let _ = read_loop(&mut class, commands, drivers).await;
            defmt::info!("usb disconnected");
        }
    };

    join(usb_fut, cmd_fut).await;
    unreachable!()
}

async fn read_loop(
    class: &mut CdcAcmClass<'static, Driver<'static, USB>>,
    commands: CommandSender,
    drivers: &'static Drivers,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut rx = [0u8; 64];
    let mut tx = [0u8; 64]; // headroom for the 28-byte TmcConfig report
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
            Ok(Command::SetTmcConfig { joint, flags }) => {
                match JointId::from_index(joint) {
                    Some(j) => {
                        if let Err(e) = drivers.apply_tmc_flags(j, flags).await {
                            defmt::warn!("tmc config j{} failed: {:?}", joint, e);
                        }
                    }
                    None => defmt::warn!("tmc config: bad joint {}", joint),
                }
            }
            Ok(Command::GetTmcConfig { joint }) => {
                let report = read_tmc_config(drivers, joint).await;
                let len = encode_tmc_config(&report, &mut tx);
                class.write_packet(&tx[..len]).await?;
            }
            Ok(Command::GetVersion) => {
                let len = encode_version(&version_report(), &mut tx);
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
    let mut servos = [0u16; NUM_SERVOS];
    for (i, slot) in servos.iter_mut().enumerate() {
        *slot = SERVO_POSITIONS[i].load(Ordering::Relaxed);
    }
    StateReport { positions, servos, flags: 0 }
}

/// Read GCONF + CHOPCONF for `joint` over the TMC2209 UART. Always
/// produces a report — `ok=false` with zero fields if the chip didn't
/// answer (bad joint id, timeout, CRC). The host uses this to verify
/// that the bits a SetTmcConfig requested are actually set on the chip.
async fn read_tmc_config(drivers: &'static Drivers, joint: u8) -> TmcConfigReport {
    fn err_status(e: DriverError) -> u8 {
        match e {
            DriverError::Uart => TmcConfigStatus::UartError as u8,
            DriverError::Crc => TmcConfigStatus::Timeout as u8,
            DriverError::Timeout => TmcConfigStatus::Timeout as u8,
            DriverError::BadRegister => TmcConfigStatus::BadRegister as u8,
        }
    }
    let empty_trace = [0u8; RX_TRACE_LEN];
    let Some(j) = JointId::from_index(joint) else {
        defmt::warn!("get tmc config: bad joint {}", joint);
        return TmcConfigReport {
            joint,
            status: TmcConfigStatus::BadJoint as u8,
            gconf: 0,
            chopconf: 0,
            rx_count: 0,
            rx_trace: empty_trace,
        };
    };
    // Use the traced variant for GCONF so the host can see the raw bytes
    // that came back (or didn't). CHOPCONF is read normally — if GCONF
    // already failed we never get here, and the GCONF trace is the
    // diagnostic we care about for "is the bus alive at all?".
    let (g_res, rx_trace, rx_count) = drivers.read_register_traced::<reg::GCONF>(j).await;
    let gconf: u32 = match g_res {
        Ok(g) => g.into(),
        Err(e) => {
            defmt::warn!("read GCONF j{} failed: {:?} (rx_count={})", joint, e, rx_count);
            return TmcConfigReport {
                joint, status: err_status(e), gconf: 0, chopconf: 0, rx_count, rx_trace,
            };
        }
    };
    let chopconf = match drivers.read_register::<reg::CHOPCONF>(j).await {
        Ok(c) => c.into(),
        Err(e) => {
            defmt::warn!("read CHOPCONF j{} failed: {:?}", joint, e);
            return TmcConfigReport {
                joint, status: err_status(e), gconf: 0, chopconf: 0, rx_count, rx_trace,
            };
        }
    };
    TmcConfigReport {
        joint, status: TmcConfigStatus::Ok as u8, gconf, chopconf, rx_count, rx_trace,
    }
}
