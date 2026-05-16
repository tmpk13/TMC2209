//! Hobby-servo control via the RP2040/RP2350 PWM peripheral.
//!
//! Two servos share PWM slice 7 (channels A and B), so they run off the same
//! 50 Hz / 20 ms period clock; only the compare value differs between them.
//! The divider is set so one PWM count equals one microsecond, which means the
//! compare register reads directly in microseconds of pulse width.
//!
//! Each servo carries its own [`ServoConfig`]:
//!   - `min_us` / `max_us`     pulse-width clamp (e.g. 500..2500)
//!   - `deadzone_us`           moves smaller than this are ignored
//!   - `speed_us_per_s`        slew rate; 0 means "snap to target"
//!   - `flags` bit 0           homing on enable
//!   - `home_us`               home pulse used when homing is enabled
//!
//! The controller is ticked from the motion task. On each tick it slews the
//! commanded pulse width toward the target while respecting the deadzone.

use core::sync::atomic::{AtomicU16, Ordering};

use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::Peri;
use fixed::traits::ToFixed;

use crate::board;
use crate::motion::NUM_SERVOS;

/// Latest commanded pulse width per servo, in microseconds. Updated by the
/// motion task; read by the USB task to serve `GetState`.
pub static SERVO_POSITIONS: [AtomicU16; NUM_SERVOS] = [AtomicU16::new(0), AtomicU16::new(0)];

pub const SERVO_FLAG_HOMING: u8 = 1 << 0;

#[derive(Clone, Copy, Debug)]
pub struct ServoConfig {
    pub min_us: u16,
    pub max_us: u16,
    pub deadzone_us: u16,
    pub speed_us_per_s: u16,
    pub home_us: u16,
    pub flags: u8,
}

impl Default for ServoConfig {
    fn default() -> Self {
        Self {
            min_us: 500,
            max_us: 2_500,
            deadzone_us: 5,
            speed_us_per_s: 1_000,
            home_us: 1_500,
            flags: SERVO_FLAG_HOMING,
        }
    }
}

impl ServoConfig {
    pub fn homing_enabled(self) -> bool {
        self.flags & SERVO_FLAG_HOMING != 0
    }
}

struct ServoState {
    config: ServoConfig,
    /// Latest target pulse width in microseconds (clamped to config).
    target_us: f32,
    /// Currently commanded pulse width (slews toward target).
    current_us: f32,
}

impl ServoState {
    fn new(config: ServoConfig) -> Self {
        let home = config.home_us as f32;
        Self {
            config,
            target_us: home,
            current_us: home,
        }
    }

    fn clamp(&self, v: f32) -> f32 {
        v.clamp(self.config.min_us as f32, self.config.max_us as f32)
    }

    fn set_target(&mut self, target_us: u16) {
        self.target_us = self.clamp(target_us as f32);
    }

    fn set_config(&mut self, config: ServoConfig) {
        self.config = config;
        self.target_us = self.clamp(self.target_us);
        self.current_us = self.clamp(self.current_us);
    }

    /// Advance toward the target by one tick. Returns the new compare value
    /// (in microseconds), or `None` if no change was needed.
    fn step(&mut self, dt_us: u32) -> Option<u16> {
        let cfg = self.config;
        let err = self.target_us - self.current_us;
        if libm::fabsf(err) <= cfg.deadzone_us as f32 {
            return None;
        }
        if cfg.speed_us_per_s == 0 {
            self.current_us = self.target_us;
            return Some(self.current_us as u16);
        }
        let dt_s = dt_us as f32 * 1e-6;
        let max_step = cfg.speed_us_per_s as f32 * dt_s;
        let step = err.clamp(-max_step, max_step);
        self.current_us = self.clamp(self.current_us + step);
        Some(self.current_us as u16)
    }
}

pub struct ServoController<'d> {
    pwm: Pwm<'d>,
    servos: [ServoState; NUM_SERVOS],
}

impl<'d> ServoController<'d> {
    pub fn new(
        slice: Peri<'d, board::ServoPwm>,
        pin_a: Peri<'d, board::Servo0Pin>,
        pin_b: Peri<'d, board::Servo1Pin>,
    ) -> Self {
        let mut cfg = PwmConfig::default();
        cfg.divider = (board::SERVO_DIV as u16).to_fixed();
        cfg.top = board::SERVO_TOP;
        let servo_a = ServoState::new(ServoConfig::default());
        let servo_b = ServoState::new(ServoConfig::default());
        cfg.compare_a = servo_a.current_us as u16;
        cfg.compare_b = servo_b.current_us as u16;
        cfg.enable = true;
        let pwm = Pwm::new_output_ab(slice, pin_a, pin_b, cfg);
        Self {
            pwm,
            servos: [servo_a, servo_b],
        }
    }

    pub fn set_target(&mut self, id: u8, target_us: u16) {
        if let Some(s) = self.servos.get_mut(id as usize) {
            s.set_target(target_us);
        } else {
            defmt::warn!("servo set target: bad id {}", id);
        }
    }

    pub fn set_config(&mut self, id: u8, config: ServoConfig) {
        if let Some(s) = self.servos.get_mut(id as usize) {
            s.set_config(config);
            if config.homing_enabled() {
                s.target_us = s.clamp(config.home_us as f32);
            }
        } else {
            defmt::warn!("servo set config: bad id {}", id);
        }
    }

    pub fn config(&self, id: u8) -> Option<ServoConfig> {
        self.servos.get(id as usize).map(|s| s.config)
    }

    pub fn report_positions(&self) -> [u16; NUM_SERVOS] {
        [
            SERVO_POSITIONS[0].load(Ordering::Relaxed),
            SERVO_POSITIONS[1].load(Ordering::Relaxed),
        ]
    }

    pub fn tick(&mut self, dt_us: u32) {
        // We have to read out the current config, modify the compare fields,
        // and write the whole thing back - the embassy-rp PWM API doesn't
        // expose a per-channel compare setter.
        let a_new = self.servos[0].step(dt_us);
        let b_new = self.servos[1].step(dt_us);
        if a_new.is_none() && b_new.is_none() {
            return;
        }
        let a_cmp = a_new.unwrap_or(self.servos[0].current_us as u16);
        let b_cmp = b_new.unwrap_or(self.servos[1].current_us as u16);
        SERVO_POSITIONS[0].store(a_cmp, Ordering::Relaxed);
        SERVO_POSITIONS[1].store(b_cmp, Ordering::Relaxed);
        let mut cfg = PwmConfig::default();
        cfg.divider = (board::SERVO_DIV as u16).to_fixed();
        cfg.top = board::SERVO_TOP;
        cfg.compare_a = a_cmp;
        cfg.compare_b = b_cmp;
        cfg.enable = true;
        self.pwm.set_config(&cfg);
    }
}
