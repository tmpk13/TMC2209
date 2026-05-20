//! Host ↔ firmware binary protocol.
//!
//! Framing: COBS-less for now — fixed-size little-endian records, one per
//! USB bulk write. Small and easy to parse from Isaac Sim's host side.
//!
//! Commands (host → fw):
//!   0x01  SetTarget       { joint: u8, pos: i32, vmax: u32, amax: u32 }
//!   0x02  Enable          { mask: u8 }   bits 0..4 = J0..J4
//!   0x03  Home            { joint: u8 }
//!   0x04  GetState        {} → StateReport
//!   0x05  SetTmcConfig    { joint: u8, flags: u8 }
//!         flags bit 0: stealthchop (1=on, 0=spreadCycle)
//!         flags bit 1: interpolate to 256 µsteps
//!         flags bit 2: invert shaft direction
//!   0x06  SetPosition     { joint: u8, pos: i32 }
//!         Re-labels the joint's current microstep count to `pos` and
//!         snaps target=pos. Motors do not move. Use after manual jogging
//!         or homing-by-hand to define a new origin.
//!   0x07  GetTmcConfig    { joint: u8 } → TmcConfig
//!         Reads GCONF and CHOPCONF from one TMC2209 over UART so the
//!         host can verify a SetTmcConfig actually took effect.
//!   0x08  GetVersion      {} → Version
//!         Returns the firmware build epoch + git short rev + chip id
//!         so the host can confirm which binary is actually running.
//!   0x09  SetServoTarget  { servo: u8, target_us: u16 }
//!   0x0A  SetServoConfig  { servo: u8, min_us, max_us, deadzone_us,
//!                            speed_us_per_s, home_us: u16, flags: u8 }
//!         flags bit 0: home-on-enable
//!
//! Reports (fw → host):
//!   0x81  StateReport     { positions: [i32; 5], servos: [u16; 2], flags: u8 }
//!   0x82  DriverStatus    { joint: u8, drv_status: u32 }
//!   0x83  TmcConfig       { joint: u8, status: u8, gconf: u32, chopconf: u32,
//!                           rx_count: u8, rx_trace: [u8; 16] }   = 28 bytes
//!         status: 0=ok, 1=bad_joint, 2=uart_error, 3=timeout, 4=bad_register.
//!         rx_count + rx_trace: raw bytes received on the TMC2209 UART
//!         during the GCONF read attempt (truncated to 16). Empty trace
//!         means the chip is silent (wiring/address); a populated trace
//!         with non-zero status means it replied but the parse failed.
//!   0x84  Version         { chip: u8, epoch: u32, git: [u8; 12] }
//!         chip: 0x40=RP2040, 0x50=RP2350. epoch: Unix seconds at build.
//!         git: ASCII short rev (null-padded; "nogit" if not in a repo).

use crate::motion::{JointTarget, NUM_JOINTS};
use crate::servo::{ServoConfig, NUM_SERVOS};

pub const MAX_FRAME: usize = 32;

#[derive(Clone, Copy, Debug)]
pub enum Command {
    SetTarget { joint: u8, target: JointTarget },
    Enable { mask: u8 },
    Home { joint: u8 },
    GetState,
    SetTmcConfig { joint: u8, flags: u8 },
    SetPosition { joint: u8, position: i32 },
    GetTmcConfig { joint: u8 },
    GetVersion,
    SetServoTarget { servo: u8, target_us: u16 },
    SetServoConfig { servo: u8, config: ServoConfig },
}

#[derive(Clone, Copy, Debug)]
pub struct VersionReport {
    pub chip: u8,
    pub epoch: u32,
    pub git: [u8; 12],
}

pub const CHIP_ID_RP2040: u8 = 0x40;
pub const CHIP_ID_RP2350: u8 = 0x50;

/// Reply to `GetTmcConfig`. `status==0` means both reads succeeded; any
/// non-zero value is a `TmcConfigStatus` discriminant.
///
/// `rx_trace` carries the raw bytes the firmware saw on the TMC2209 UART
/// during the GCONF read attempt (truncated to `RX_TRACE_LEN`). When the
/// chip is silent the trace is empty; when it replies the trace contains
/// the response bytes (and possibly an echo prefix on shared-wire setups).
/// This is the diagnostic that lets the host distinguish "chip never
/// answered" from "chip answered with garbage".
pub const RX_TRACE_LEN: usize = 16;

#[derive(Clone, Copy, Debug)]
pub struct TmcConfigReport {
    pub joint: u8,
    pub status: u8,
    pub gconf: u32,
    pub chopconf: u32,
    pub rx_count: u8,
    pub rx_trace: [u8; RX_TRACE_LEN],
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum TmcConfigStatus {
    Ok = 0,
    BadJoint = 1,
    UartError = 2,
    Timeout = 3,
    BadRegister = 4,
}

// SetTmcConfig flag bits.
pub const TMC_FLAG_STEALTHCHOP: u8 = 1 << 0;
pub const TMC_FLAG_INTERPOLATE: u8 = 1 << 1;
pub const TMC_FLAG_SHAFT_INVERT: u8 = 1 << 2;

// SetServoConfig flag bits (must match host/web.py SERVO_FLAG_*).
pub const SERVO_FLAG_HOMING: u8 = 1 << 0;

#[derive(Clone, Copy, Debug)]
pub struct StateReport {
    pub positions: [i32; NUM_JOINTS],
    pub servos: [u16; NUM_SERVOS],
    pub flags: u8,
}

#[derive(Debug)]
pub enum DecodeError {
    Short,
    BadTag,
}

pub fn decode(bytes: &[u8]) -> Result<Command, DecodeError> {
    let (&tag, rest) = bytes.split_first().ok_or(DecodeError::Short)?;
    match tag {
        0x01 => {
            if rest.len() < 1 + 4 + 4 + 4 {
                return Err(DecodeError::Short);
            }
            let joint = rest[0];
            let pos = i32::from_le_bytes(rest[1..5].try_into().unwrap());
            let vmax = u32::from_le_bytes(rest[5..9].try_into().unwrap());
            let amax = u32::from_le_bytes(rest[9..13].try_into().unwrap());
            Ok(Command::SetTarget {
                joint,
                target: JointTarget {
                    position: pos,
                    max_velocity: vmax,
                    max_accel: amax,
                },
            })
        }
        0x02 => Ok(Command::Enable {
            mask: *rest.first().ok_or(DecodeError::Short)?,
        }),
        0x03 => Ok(Command::Home {
            joint: *rest.first().ok_or(DecodeError::Short)?,
        }),
        0x04 => Ok(Command::GetState),
        0x05 => {
            if rest.len() < 2 {
                return Err(DecodeError::Short);
            }
            Ok(Command::SetTmcConfig {
                joint: rest[0],
                flags: rest[1],
            })
        }
        0x06 => {
            if rest.len() < 1 + 4 {
                return Err(DecodeError::Short);
            }
            let joint = rest[0];
            let pos = i32::from_le_bytes(rest[1..5].try_into().unwrap());
            Ok(Command::SetPosition { joint, position: pos })
        }
        0x07 => Ok(Command::GetTmcConfig {
            joint: *rest.first().ok_or(DecodeError::Short)?,
        }),
        0x08 => Ok(Command::GetVersion),
        0x09 => {
            if rest.len() < 1 + 2 {
                return Err(DecodeError::Short);
            }
            let servo = rest[0];
            let target_us = u16::from_le_bytes(rest[1..3].try_into().unwrap());
            Ok(Command::SetServoTarget { servo, target_us })
        }
        0x0A => {
            // <BB HHHHH B> = 1 + 1 + 5*u16 + u8 = 13 bytes total; we've already
            // peeled the tag, so `rest` is the 12 bytes after it.
            if rest.len() < 1 + 5 * 2 + 1 {
                return Err(DecodeError::Short);
            }
            let servo = rest[0];
            let min_us = u16::from_le_bytes(rest[1..3].try_into().unwrap());
            let max_us = u16::from_le_bytes(rest[3..5].try_into().unwrap());
            let deadzone_us = u16::from_le_bytes(rest[5..7].try_into().unwrap());
            let speed_us_per_s = u16::from_le_bytes(rest[7..9].try_into().unwrap());
            let home_us = u16::from_le_bytes(rest[9..11].try_into().unwrap());
            let flags = rest[11];
            Ok(Command::SetServoConfig {
                servo,
                config: ServoConfig {
                    min_us,
                    max_us,
                    deadzone_us,
                    speed_us_per_s,
                    home_us,
                    flags,
                },
            })
        }
        _ => Err(DecodeError::BadTag),
    }
}

pub fn encode_state(report: &StateReport, out: &mut [u8]) -> usize {
    out[0] = 0x81;
    let mut i = 1;
    for p in report.positions {
        out[i..i + 4].copy_from_slice(&p.to_le_bytes());
        i += 4;
    }
    for s in report.servos {
        out[i..i + 2].copy_from_slice(&s.to_le_bytes());
        i += 2;
    }
    out[i] = report.flags;
    i + 1
}

pub fn encode_tmc_config(report: &TmcConfigReport, out: &mut [u8]) -> usize {
    out[0] = 0x83;
    out[1] = report.joint;
    out[2] = report.status;
    out[3..7].copy_from_slice(&report.gconf.to_le_bytes());
    out[7..11].copy_from_slice(&report.chopconf.to_le_bytes());
    out[11] = report.rx_count;
    out[12..12 + RX_TRACE_LEN].copy_from_slice(&report.rx_trace);
    12 + RX_TRACE_LEN
}

pub fn encode_version(report: &VersionReport, out: &mut [u8]) -> usize {
    out[0] = 0x84;
    out[1] = report.chip;
    out[2..6].copy_from_slice(&report.epoch.to_le_bytes());
    out[6..18].copy_from_slice(&report.git);
    18
}
