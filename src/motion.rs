//! Step/dir generation via PIO + per-joint move queue.
//!
//! Each joint owns one PIO0 state machine running this program:
//!
//! ```text
//! .side_set 1
//! .wrap_target
//!     pull block        side 0   ; read 32-bit pulse count
//!     mov x, osr        side 0
//! loop:
//!     jmp !x done       side 0
//!     nop               side 1 [7]   ; STEP high
//!     nop               side 0 [7]   ; STEP low
//!     jmp x-- loop      side 0
//! done:
//! .wrap
//! ```
//!
//! The PIO clock divider sets the step frequency; each 32-bit FIFO word
//! requests that many pulses. The DIR pin is a plain GPIO toggled from CPU.
//!
//! v0 uses a very simple motion model: constant velocity toward the latest
//! commanded target. The trapezoidal profile lives on the TODO list.

use core::sync::atomic::{AtomicI32, Ordering};

use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    Common, Config as PioConfig, Direction, LoadedProgram, Pio, ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_time::{Duration, Ticker};
use fixed::traits::ToFixed;

use crate::board;
use crate::protocol::{Command, StateReport};

pub const NUM_JOINTS: usize = 3;
const TICK_HZ: u32 = 200;
/// Cycles the PIO program spends per step pulse (1 high delay + 1 low delay + overhead).
const PIO_CYCLES_PER_STEP: u32 = 16;

#[derive(Clone, Copy, Debug)]
pub struct JointTarget {
    /// Absolute target position in microsteps.
    pub position: i32,
    /// Maximum speed in microsteps/second.
    pub max_velocity: u32,
    /// Acceleration (unused in v0; reserved for trapezoidal profile).
    pub max_accel: u32,
}

impl Default for JointTarget {
    fn default() -> Self {
        Self {
            position: 0,
            max_velocity: 2_000,
            max_accel: 5_000,
        }
    }
}

pub type CommandReceiver = Receiver<'static, CriticalSectionRawMutex, Command, 8>;

/// Feedback positions, readable from any task without locking.
pub static POSITIONS: [AtomicI32; NUM_JOINTS] =
    [AtomicI32::new(0), AtomicI32::new(0), AtomicI32::new(0)];

struct Joint<'d, const SM: usize> {
    sm: StateMachine<'d, PIO0, SM>,
    dir: Output<'d>,
    /// Committed position in microsteps (total steps emitted so far).
    position: i32,
    /// Current velocity in microsteps/second, signed.
    velocity: f32,
    /// Fractional-microstep accumulator — carries the sub-step remainder
    /// between ticks so slow moves still make progress.
    frac_accum: f32,
    target: JointTarget,
    enabled: bool,
}

impl<'d, const SM: usize> Joint<'d, SM> {
    fn tick(&mut self, idx: usize, dt_us: u32) {
        if !self.enabled {
            self.velocity = 0.0;
            return;
        }
        let err = self.target.position - self.position;
        if err == 0 && self.velocity == 0.0 {
            return;
        }

        let dt = dt_us as f32 * 1e-6;
        let a = self.target.max_accel.max(1) as f32;
        let v_max = self.target.max_velocity as f32;

        // Max speed we can have right now and still decelerate to zero exactly
        // at the target. Adding one tick's worth of braking room
        // (`+ 0.5 * a * dt`) avoids bouncing around ±1 µstep at the finish.
        let brake_dist = err.unsigned_abs() as f32;
        let v_brake = libm::sqrtf(2.0 * a * brake_dist);
        let sign = if err >= 0 { 1.0 } else { -1.0 };
        let v_target = sign * v_max.min(v_brake);

        // Slew velocity toward v_target, bounded by acceleration.
        let dv_max = a * dt;
        let dv = (v_target - self.velocity).clamp(-dv_max, dv_max);
        self.velocity += dv;

        // Figure out how many integer microsteps that velocity produces over
        // this tick; carry the fractional remainder into the next tick.
        let step_float = self.velocity * dt + self.frac_accum;
        let steps = step_float as i32;
        self.frac_accum = step_float - steps as f32;

        if steps == 0 {
            return;
        }

        let positive = steps > 0;
        self.dir.set_level(if positive { Level::High } else { Level::Low });

        // Space the step burst across the full tick window so the motor hears
        // a steady cadence instead of a machine-gun at the start of each tick.
        let n = steps.unsigned_abs();
        let target_freq = n * TICK_HZ;
        let divider = calculate_pio_clock_divider(target_freq * PIO_CYCLES_PER_STEP);
        self.sm.set_clock_divider(divider);
        self.sm.clkdiv_restart();
        let _ = self.sm.tx().try_push(n);

        self.position += steps;
        POSITIONS[idx].store(self.position, Ordering::Relaxed);
    }
}

pub struct MotionController<'d> {
    j0: Joint<'d, 0>,
    j1: Joint<'d, 1>,
    j2: Joint<'d, 2>,
    enable: Output<'d>,
}

impl<'d> MotionController<'d> {
    pub fn new(
        pio: Pio<'d, PIO0>,
        step0: Peri<'d, board::Step0Pin>,
        step1: Peri<'d, board::Step1Pin>,
        step2: Peri<'d, board::Step2Pin>,
        dir0: Peri<'d, board::Dir0Pin>,
        dir1: Peri<'d, board::Dir1Pin>,
        dir2: Peri<'d, board::Dir2Pin>,
        enable: Peri<'d, board::EnablePin>,
    ) -> Self {
        let Pio {
            mut common,
            sm0,
            sm1,
            sm2,
            ..
        } = pio;

        let prg = pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "    pull block        side 0",
            "    mov x, osr        side 0",
            "loop_:",
            "    jmp !x done       side 0",
            "    nop               side 1 [7]",
            "    nop               side 0 [7]",
            "    jmp x-- loop_     side 0",
            "done:",
            ".wrap",
        );
        let loaded = common.load_program(&prg.program);

        let j0 = setup_sm(&mut common, sm0, &loaded, step0);
        let j1 = setup_sm(&mut common, sm1, &loaded, step1);
        let j2 = setup_sm(&mut common, sm2, &loaded, step2);

        Self {
            j0: Joint {
                sm: j0,
                dir: Output::new(dir0, Level::Low),
                position: 0,
                velocity: 0.0,
                frac_accum: 0.0,
                target: JointTarget::default(),
                enabled: true,
            },
            j1: Joint {
                sm: j1,
                dir: Output::new(dir1, Level::Low),
                position: 0,
                velocity: 0.0,
                frac_accum: 0.0,
                target: JointTarget::default(),
                enabled: true,
            },
            j2: Joint {
                sm: j2,
                dir: Output::new(dir2, Level::Low),
                position: 0,
                velocity: 0.0,
                frac_accum: 0.0,
                target: JointTarget::default(),
                enabled: true,
            },
            enable: Output::new(enable, Level::High), // active-low → disabled on boot
        }
    }

    pub fn set_enable(&mut self, en: bool) {
        self.enable.set_level(if en { Level::Low } else { Level::High });
    }

    pub fn state(&self) -> StateReport {
        StateReport {
            positions: [self.j0.position, self.j1.position, self.j2.position],
            flags: 0,
        }
    }

    fn apply(&mut self, cmd: Command) {
        match cmd {
            Command::SetTarget { joint, target } => match joint {
                0 => self.j0.target = target,
                1 => self.j1.target = target,
                2 => self.j2.target = target,
                _ => defmt::warn!("unknown joint id {}", joint),
            },
            Command::Enable { mask } => {
                let any = mask != 0;
                self.set_enable(any);
                self.j0.enabled = mask & 0b001 != 0;
                self.j1.enabled = mask & 0b010 != 0;
                self.j2.enabled = mask & 0b100 != 0;
            }
            Command::Home { .. } | Command::GetState => {}
        }
    }

    pub async fn run(&mut self, commands: CommandReceiver) -> ! {
        let period = Duration::from_hz(TICK_HZ as u64);
        let mut ticker = Ticker::every(period);
        let dt_us = 1_000_000 / TICK_HZ;
        loop {
            while let Ok(cmd) = commands.try_receive() {
                self.apply(cmd);
            }
            self.j0.tick(0, dt_us);
            self.j1.tick(1, dt_us);
            self.j2.tick(2, dt_us);
            ticker.next().await;
        }
    }
}

fn setup_sm<'d, const SM: usize>(
    common: &mut Common<'d, PIO0>,
    mut sm: StateMachine<'d, PIO0, SM>,
    program: &LoadedProgram<'d, PIO0>,
    step_pin: Peri<'d, impl embassy_rp::pio::PioPin>,
) -> StateMachine<'d, PIO0, SM> {
    let pin = common.make_pio_pin(step_pin);
    sm.set_pin_dirs(Direction::Out, &[&pin]);

    let mut cfg = PioConfig::default();
    cfg.use_program(program, &[&pin]);
    cfg.shift_out.direction = ShiftDirection::Right;
    cfg.shift_out.auto_fill = false;
    cfg.shift_out.threshold = 32;
    cfg.clock_divider = 256u16.to_fixed();
    sm.set_config(&cfg);
    sm.set_enable(true);
    sm
}
