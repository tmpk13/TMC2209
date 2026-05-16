//! Host <-> firmware binary protocol.
//!
//! Framing: COBS-less for now - fixed-size little-endian records, one per
//! USB bulk write. Small and easy to parse from Isaac Sim's host side.
//!
//! Commands (host -> fw):
//!   0x01  SetTarget       { joint: u8, pos: i32, vmax: u32, amax: u32 }
//!   0x02  Enable          { mask: u8 }     bit i = joint i (i = 0..NUM_JOINTS)
//!   0x03  Home            { joint: u8 }
//!   0x04  GetState        {} -> StateReport
//!   0x05  SetTmcConfig    { joint: u8, flags: u8 }
//!         flags bit 0: stealthchop (1=on, 0=spreadCycle)
//!         flags bit 1: interpolate to 256 usteps
//!         flags bit 2: invert shaft direction
//!   0x06  SetPosition     { joint: u8, pos: i32 }
//!   0x07  GetTmcConfig    { joint: u8 } -> TmcConfig
//!   0x08  GetVersion      {} -> Version
//!   0x09  SetServoTarget  { id: u8, target_us: u16 }
//!   0x0A  SetServoConfig  { id: u8, min_us: u16, max_us: u16, deadzone_us: u16,
//!                           speed_us_per_s: u16, home_us: u16, flags: u8 }
//!         flags bit 0: homing on enable
//!
//! Servo config is stored host-side - the host pushes SetServoConfig on
//! connect, so no GetServoConfig is needed.
//!
//! Reports (fw -> host):
//!   0x81  StateReport     { positions: [i32; NUM_JOINTS],
//!                            servos: [u16; NUM_SERVOS], flags: u8 }
//!   0x82  DriverStatus    { joint: u8, drv_status: u32 }
//!   0x83  TmcConfig       { joint: u8, status: u8, gconf: u32, chopconf: u32,
//!                           rx_count: u8, rx_trace: [u8; 16] }
//!   0x84  Version         { chip: u8, epoch: u32, git: [u8; 12] }

use crate::motion::{JointTarget, NUM_JOINTS, NUM_SERVOS};
use crate::servo::ServoConfig;

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
    SetServoTarget { id: u8, target_us: u16 },
    SetServoConfig { id: u8, config: ServoConfig },
}

#[derive(Clone, Copy, Debug)]
pub struct VersionReport {
    pub chip: u8,
    pub epoch: u32,
    pub git: [u8; 12],
}

pub const CHIP_ID_RP2040: u8 = 0x40;
pub const CHIP_ID_RP2350: u8 = 0x50;

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
            let id = rest[0];
            let target_us = u16::from_le_bytes(rest[1..3].try_into().unwrap());
            Ok(Command::SetServoTarget { id, target_us })
        }
        0x0A => {
            // id (1) + 5x u16 (10) + flags (1) = 12 bytes
            if rest.len() < 1 + 10 + 1 {
                return Err(DecodeError::Short);
            }
            let id = rest[0];
            let min_us = u16::from_le_bytes(rest[1..3].try_into().unwrap());
            let max_us = u16::from_le_bytes(rest[3..5].try_into().unwrap());
            let deadzone_us = u16::from_le_bytes(rest[5..7].try_into().unwrap());
            let speed_us_per_s = u16::from_le_bytes(rest[7..9].try_into().unwrap());
            let home_us = u16::from_le_bytes(rest[9..11].try_into().unwrap());
            let flags = rest[11];
            Ok(Command::SetServoConfig {
                id,
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

