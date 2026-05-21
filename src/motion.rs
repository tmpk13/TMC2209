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
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{
    Common, Config as PioConfig, Direction, Instance as PioInstance, LoadedProgram, Pio,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_time::{Duration, Ticker};
use fixed::traits::ToFixed;

use crate::board;
use crate::servo;
use crate::protocol::{Command, StateReport};

pub const NUM_JOINTS: usize = 5;
const TICK_HZ: u32 = 200;
/// PIO cycles per step pulse: jmp(1) + nop_high(1+7) + nop_low(1+7) + jmp_dec(1) = 18.
const PIO_CYCLES_PER_STEP: u32 = 18;

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
pub static POSITIONS: [AtomicI32; NUM_JOINTS] = [
    AtomicI32::new(0),
    AtomicI32::new(0),
    AtomicI32::new(0),
    AtomicI32::new(0),
    AtomicI32::new(0),
];

struct Joint<'d, P: PioInstance, const SM: usize> {
    sm: StateMachine<'d, P, SM>,
    dir: Output<'d>,
    /// Committed position in microsteps (total steps emitted so far).
    position: i32,
    /// Current velocity in microsteps/second, signed.
    velocity: f32,
    /// Fractional-microstep accumulator - carries the sub-step remainder
    /// between ticks so slow moves still make progress.
    frac_accum: f32,
    target: JointTarget,
    enabled: bool,
}

impl<'d, P: PioInstance, const SM: usize> Joint<'d, P, SM> {
    /// Adopt `position` as the joint's current microstep count without
    /// commanding any motion. Snaps target to match and clears the motion
    /// integrators so the next tick starts from rest at the new origin.
    fn set_position(&mut self, idx: usize, position: i32) {
        self.position = position;
        self.target.position = position;
        self.velocity = 0.0;
        self.frac_accum = 0.0;
        POSITIONS[idx].store(position, Ordering::Relaxed);
    }

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

        if self.sm.tx().try_push(n) {
            self.position += steps;
            POSITIONS[idx].store(self.position, Ordering::Relaxed);
        } else {
            // FIFO full — undo velocity/frac so we retry next tick.
            self.frac_accum += steps as f32;
        }
    }
}

pub struct MotionController<'d> {
    j0: Joint<'d, PIO0, 0>,
    j1: Joint<'d, PIO0, 1>,
    j2: Joint<'d, PIO0, 2>,
    j3: Joint<'d, PIO0, 3>,
    j4: Joint<'d, PIO1, 0>,
    enable: Output<'d>,
}

impl<'d> MotionController<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pio0: Pio<'d, PIO0>,
        pio1: Pio<'d, PIO1>,
        step0: Peri<'d, board::Step0Pin>,
        step1: Peri<'d, board::Step1Pin>,
        step2: Peri<'d, board::Step2Pin>,
        step3: Peri<'d, board::Step3Pin>,
        step4: Peri<'d, board::Step4Pin>,
        dir0: Peri<'d, board::Dir0Pin>,
        dir1: Peri<'d, board::Dir1Pin>,
        dir2: Peri<'d, board::Dir2Pin>,
        dir3: Peri<'d, board::Dir3Pin>,
        dir4: Peri<'d, board::Dir4Pin>,
        enable: Peri<'d, board::EnablePin>,
    ) -> Self {
        let Pio {
            common: mut common0,
            sm0,
            sm1,
            sm2,
            sm3,
            ..
        } = pio0;
        let Pio {
            common: mut common1,
            sm0: sm1_0,
            ..
        } = pio1;

        let prg = pio::pio_asm!(
            ".side_set 1",
            "start:",
            "    pull block        side 0",
            "    mov x, osr        side 0",
            "loop_:",
            "    jmp !x start      side 0",
            "    nop               side 1 [7]",
            "    nop               side 0 [7]",
            "    jmp x-- loop_     side 0",
        );
        // Each PIO has its own instruction memory, so load the program twice.
        let loaded0 = common0.load_program(&prg.program);
        let loaded1 = common1.load_program(&prg.program);

        let j0_sm = setup_sm(&mut common0, sm0, &loaded0, step0);
        let j1_sm = setup_sm(&mut common0, sm1, &loaded0, step1);
        let j2_sm = setup_sm(&mut common0, sm2, &loaded0, step2);
        let j3_sm = setup_sm(&mut common0, sm3, &loaded0, step3);
        let j4_sm = setup_sm(&mut common1, sm1_0, &loaded1, step4);

        Self {
            j0: Joint::new(j0_sm, Output::new(dir0, Level::Low)),
            j1: Joint::new(j1_sm, Output::new(dir1, Level::Low)),
            j2: Joint::new(j2_sm, Output::new(dir2, Level::Low)),
            j3: Joint::new(j3_sm, Output::new(dir3, Level::Low)),
            j4: Joint::new(j4_sm, Output::new(dir4, Level::Low)),
            enable: Output::new(enable, Level::High), // active-low -> disabled on boot
        }
    }

    pub fn set_enable(&mut self, en: bool) {
        self.enable.set_level(if en { Level::Low } else { Level::High });
    }

    pub fn state(&self) -> StateReport {
        StateReport {
            positions: [
                self.j0.position,
                self.j1.position,
                self.j2.position,
                self.j3.position,
                self.j4.position,
            ],
            servos: servo::snapshot(),
            flags: 0,
        }
    }

    fn apply(&mut self, cmd: Command) {
        match cmd {
            Command::SetTarget { joint, target } => match joint {
                0 => self.j0.target = target,
                1 => self.j1.target = target,
                2 => self.j2.target = target,
                3 => self.j3.target = target,
                4 => self.j4.target = target,
                _ => defmt::warn!("unknown joint id {}", joint),
            },
            Command::Enable { mask } => {
                let any = mask != 0;
                self.set_enable(any);
                self.j0.enabled = mask & 0b00001 != 0;
                self.j1.enabled = mask & 0b00010 != 0;
                self.j2.enabled = mask & 0b00100 != 0;
                self.j3.enabled = mask & 0b01000 != 0;
                self.j4.enabled = mask & 0b10000 != 0;
            }
            Command::SetPosition { joint, position } => match joint {
                0 => self.j0.set_position(0, position),
                1 => self.j1.set_position(1, position),
                2 => self.j2.set_position(2, position),
                3 => self.j3.set_position(3, position),
                4 => self.j4.set_position(4, position),
                _ => defmt::warn!("set position: bad joint {}", joint),
            },
            // SetTmcConfig and GetTmcConfig are handled in the USB task
            // (it owns the driver bus). Servo commands route through their
            // own channel into the servo task.
            Command::Home { .. }
            | Command::GetState
            | Command::SetTmcConfig { .. }
            | Command::GetTmcConfig { .. }
            | Command::GetVersion
            | Command::SetServoTarget { .. }
            | Command::SetServoConfig { .. }
            | Command::SetDcMotor { .. } => {}
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
            self.j3.tick(3, dt_us);
            self.j4.tick(4, dt_us);
            ticker.next().await;
        }
    }
}

impl<'d, P: PioInstance, const SM: usize> Joint<'d, P, SM> {
    fn new(sm: StateMachine<'d, P, SM>, dir: Output<'d>) -> Self {
        Self {
            sm,
            dir,
            position: 0,
            velocity: 0.0,
            frac_accum: 0.0,
            target: JointTarget::default(),
            enabled: true,
        }
    }
}

fn setup_sm<'d, P: PioInstance, const SM: usize>(
    common: &mut Common<'d, P>,
    mut sm: StateMachine<'d, P, SM>,
    program: &LoadedProgram<'d, P>,
    step_pin: Peri<'d, impl embassy_rp::pio::PioPin>,
) -> StateMachine<'d, P, SM> {
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
