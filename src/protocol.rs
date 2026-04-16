//! Host ↔ firmware binary protocol.
//!
//! Framing: COBS-less for now — fixed-size little-endian records, one per
//! USB bulk write. Small and easy to parse from Isaac Sim's host side.
//!
//! Commands (host → fw):
//!   0x01  SetTarget       { joint: u8, pos: i32, vmax: u32, amax: u32 }
//!   0x02  Enable          { mask: u8 }
//!   0x03  Home            { joint: u8 }
//!   0x04  GetState        {} → StateReport
//!
//! Reports (fw → host):
//!   0x81  StateReport     { positions: [i32; 3], flags: u8 }
//!   0x82  DriverStatus    { joint: u8, drv_status: u32 }

use crate::motion::{JointTarget, NUM_JOINTS};

pub const MAX_FRAME: usize = 32;

#[derive(Clone, Copy, Debug)]
pub enum Command {
    SetTarget { joint: u8, target: JointTarget },
    Enable { mask: u8 },
    Home { joint: u8 },
    GetState,
}

#[derive(Clone, Copy, Debug)]
pub struct StateReport {
    pub positions: [i32; NUM_JOINTS],
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
    out[i] = report.flags;
    i + 1
}
