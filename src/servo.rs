//! Hobby-servo PWM output with software slew limiting and home-on-enable.
//!
//! Two servos share PWM slice 7 (channel A on GP14, channel B on GP15) so they
//! run off the same wrap counter at 50 Hz. The slice is clocked at 1 MHz
//! (sysclk / 125) so the compare register reads directly in microseconds:
//! TOP = 19_999 gives a 20 ms period and `compare_x = target_us` produces the
//! corresponding pulse width.
//!
//! The servo task runs at 50 Hz and slews the live pulse width toward the
//! commanded target at `speed_us_per_s`. When a servo is enabled (via the
//! joint enable mask) and `SERVO_FLAG_HOMING` is set, the target snaps to
//! `home_us` on the disabled->enabled transition. `POSITIONS` carries the
//! live pulse width so the host UI can show it.

use core::sync::atomic::{AtomicU16, Ordering};

use embassy_executor::Spawner;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_time::{Duration, Ticker};
use fixed::traits::ToFixed;

use crate::board;
use crate::protocol::SERVO_FLAG_HOMING;

pub const NUM_SERVOS: usize = 2;
const TICK_HZ: u32 = 50;
const PWM_TOP: u16 = 19_999;

#[derive(Clone, Copy, Debug)]
pub enum ServoCommand {
    SetTarget { servo: u8, target_us: u16 },
    SetConfig { servo: u8, config: ServoConfig },
    Enable { mask: u8 },
}

#[derive(Clone, Copy, Debug)]
pub struct ServoConfig {
    pub min_us: u16,
    pub max_us: u16,
    pub deadzone_us: u16,
    pub speed_us_per_s: u16,
    pub home_us: u16,
    pub flags: u8,
}

impl ServoConfig {
    pub const fn boot_default() -> Self {
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

/// Live pulse widths in microseconds, readable from any task without locking.
pub static POSITIONS: [AtomicU16; NUM_SERVOS] =
    [AtomicU16::new(1500), AtomicU16::new(1500)];

pub static COMMANDS: Channel<CriticalSectionRawMutex, ServoCommand, 8> = Channel::new();

pub fn snapshot() -> [u16; NUM_SERVOS] {
    [
        POSITIONS[0].load(Ordering::Relaxed),
        POSITIONS[1].load(Ordering::Relaxed),
    ]
}

struct Servo {
    config: ServoConfig,
    target_us: u16,
    current_us: f32,
    enabled: bool,
}

impl Servo {
    const fn new() -> Self {
        let cfg = ServoConfig::boot_default();
        Self {
            config: cfg,
            target_us: cfg.home_us,
            current_us: cfg.home_us as f32,
            enabled: false,
        }
    }

    fn clamp(&self, us: u16) -> u16 {
        us.clamp(self.config.min_us, self.config.max_us)
    }

    fn set_target(&mut self, target_us: u16) {
        self.target_us = self.clamp(target_us);
    }

    fn set_config(&mut self, cfg: ServoConfig) {
        self.config = cfg;
        // Keep the existing target inside the new range.
        self.target_us = self.clamp(self.target_us);
    }

    fn on_enable(&mut self, on: bool) {
        if on && !self.enabled && self.config.flags & SERVO_FLAG_HOMING != 0 {
            self.target_us = self.clamp(self.config.home_us);
        }
        self.enabled = on;
    }

    /// Advance current_us toward target_us at speed_us_per_s.
    fn tick(&mut self, dt: f32) -> u16 {
        if !self.enabled {
            return self.current_us as u16;
        }
        let err = self.target_us as f32 - self.current_us;
        if libm::fabsf(err) <= self.config.deadzone_us as f32 {
            self.current_us = self.target_us as f32;
        } else {
            let step = self.config.speed_us_per_s as f32 * dt;
            let delta = if err.abs() <= step {
                err
            } else {
                step.copysign(err)
            };
            self.current_us += delta;
        }
        self.current_us as u16
    }
}

#[embassy_executor::task]
async fn servo_task(
    mut pwm: Pwm<'static>,
    mut cfg: PwmConfig,
    commands: Receiver<'static, CriticalSectionRawMutex, ServoCommand, 8>,
) {
    let mut servos = [Servo::new(), Servo::new()];
    let period = Duration::from_hz(TICK_HZ as u64);
    let mut ticker = Ticker::every(period);
    let dt = 1.0 / TICK_HZ as f32;

    cfg.compare_a = servos[0].current_us as u16;
    cfg.compare_b = servos[1].current_us as u16;
    pwm.set_config(&cfg);

    loop {
        while let Ok(cmd) = commands.try_receive() {
            match cmd {
                ServoCommand::SetTarget { servo, target_us } => {
                    if let Some(s) = servos.get_mut(servo as usize) {
                        s.set_target(target_us);
                    } else {
                        defmt::warn!("servo target: bad index {}", servo);
                    }
                }
                ServoCommand::SetConfig { servo, config } => {
                    if let Some(s) = servos.get_mut(servo as usize) {
                        s.set_config(config);
                    } else {
                        defmt::warn!("servo config: bad index {}", servo);
                    }
                }
                ServoCommand::Enable { mask } => {
                    for (i, s) in servos.iter_mut().enumerate() {
                        s.on_enable(mask & (1 << i) != 0);
                    }
                }
            }
        }

        let us0 = servos[0].tick(dt);
        let us1 = servos[1].tick(dt);
        POSITIONS[0].store(us0, Ordering::Relaxed);
        POSITIONS[1].store(us1, Ordering::Relaxed);

        cfg.compare_a = us0;
        cfg.compare_b = us1;
        pwm.set_config(&cfg);

        ticker.next().await;
    }
}

pub fn spawn(
    spawner: Spawner,
    slice: Peri<'static, board::ServoSlice>,
    pin_a: Peri<'static, board::Servo0Pin>,
    pin_b: Peri<'static, board::Servo1Pin>,
) {
    // 1 MHz PWM clock so compare counts read as microseconds. TOP = 19_999
    // gives a 20 ms (50 Hz) period.
    let mut cfg = PwmConfig::default();
    cfg.divider = 125u8.to_fixed();
    cfg.top = PWM_TOP;
    cfg.compare_a = 1500;
    cfg.compare_b = 1500;
    cfg.enable = true;
    let pwm = Pwm::new_output_ab(slice, pin_a, pin_b, cfg.clone());
    spawner.must_spawn(servo_task(pwm, cfg, COMMANDS.receiver()));
}
