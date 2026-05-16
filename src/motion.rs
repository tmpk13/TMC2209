//! Step/dir generation via PIO + per-joint move queue.
//!
//! Each joint owns one PIO state machine running this program:
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
//! Joints 0..=3 live on PIO0 (4 state machines). Joint 4 lives on PIO1 SM0
//! because PIO0 is full.
//!
//! v0 uses a very simple motion model: constant velocity toward the latest
//! commanded target. The trapezoidal profile lives on the TODO list.

use core::sync::atomic::{AtomicI32, Ordering};

use embassy_rp::gpio::{Level, Output, Pull};
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
use crate::protocol::{Command, StateReport};
use crate::servo::ServoController;

pub const NUM_JOINTS: usize = 5;
pub const NUM_SERVOS: usize = 2;
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

        let brake_dist = err.unsigned_abs() as f32;
        let v_brake = libm::sqrtf(2.0 * a * brake_dist);
        let sign = if err >= 0 { 1.0 } else { -1.0 };
        let v_target = sign * v_max.min(v_brake);

        let dv_max = a * dt;
        let dv = (v_target - self.velocity).clamp(-dv_max, dv_max);
        self.velocity += dv;

        let step_float = self.velocity * dt + self.frac_accum;
        let steps = step_float as i32;
        self.frac_accum = step_float - steps as f32;

        if steps == 0 {
            return;
        }

        let positive = steps > 0;
        self.dir.set_level(if positive { Level::High } else { Level::Low });

        let n = steps.unsigned_abs();
        let target_freq = n * TICK_HZ;
        let divider = calculate_pio_clock_divider(target_freq * PIO_CYCLES_PER_STEP);
        self.sm.set_clock_divider(divider);
        self.sm.clkdiv_restart();

        if self.sm.tx().try_push(n) {
            self.position += steps;
            POSITIONS[idx].store(self.position, Ordering::Relaxed);
        } else {
            self.frac_accum += steps as f32;
        }
    }
}

pub struct MotionController<'d> {
    j0: Joint<'d, board::StepPio0, 0>,
    j1: Joint<'d, board::StepPio0, 1>,
    j2: Joint<'d, board::StepPio0, 2>,
    j3: Joint<'d, board::StepPio0, 3>,
    j4: Joint<'d, board::StepPio1, 0>,
    enable: Output<'d>,
    pub servos: ServoController<'d>,
}

impl<'d> MotionController<'d> {
    pub fn new(
        pio0: Pio<'d, board::StepPio0>,
        pio1: Pio<'d, board::StepPio1>,
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
        servos: ServoController<'d>,
    ) -> Self {
        // PIO0: joints 0..=3 (one program, 4 state machines).
        let Pio {
            mut common,
            sm0,
            sm1,
            sm2,
            sm3,
            ..
        } = pio0;
        let prg0 = pio::pio_asm!(
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
        let loaded0 = common.load_program(&prg0.program);
        let sm0 = setup_sm(&mut common, sm0, &loaded0, step0);
        let sm1 = setup_sm(&mut common, sm1, &loaded0, step1);
        let sm2 = setup_sm(&mut common, sm2, &loaded0, step2);
        let sm3 = setup_sm(&mut common, sm3, &loaded0, step3);

        // PIO1: joint 4 only (SM0).
        let Pio {
            common: mut common1,
            sm0: sm1_0,
            ..
        } = pio1;
        let prg1 = pio::pio_asm!(
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
        let loaded1 = common1.load_program(&prg1.program);
        let sm4 = setup_sm(&mut common1, sm1_0, &loaded1, step4);

        Self {
            j0: joint(sm0, dir0),
            j1: joint(sm1, dir1),
            j2: joint(sm2, dir2),
            j3: joint(sm3, dir3),
            j4: joint(sm4, dir4),
            enable: Output::new(enable, Level::High), // active-low → disabled on boot
            servos,
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
            servos: self.servos.report_positions(),
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
            Command::SetServoTarget { id, target_us } => {
                self.servos.set_target(id, target_us);
            }
            Command::SetServoConfig { id, config } => {
                self.servos.set_config(id, config);
            }
            // SetTmcConfig and GetTmcConfig are handled in the USB task
            // (it owns the driver bus).
            Command::Home { .. }
            | Command::GetState
            | Command::SetTmcConfig { .. }
            | Command::GetTmcConfig { .. }
            | Command::GetVersion => {}
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
            self.servos.tick(dt_us);
            ticker.next().await;
        }
    }
}

fn joint<'d, P: PioInstance, const SM: usize>(
    sm: StateMachine<'d, P, SM>,
    dir: Peri<'d, impl embassy_rp::gpio::Pin>,
) -> Joint<'d, P, SM> {
    Joint {
        sm,
        dir: Output::new(dir, Level::Low),
        position: 0,
        velocity: 0.0,
        frac_accum: 0.0,
        target: JointTarget::default(),
        enabled: true,
    }
}

fn setup_sm<'d, P: PioInstance, const SM: usize>(
    common: &mut Common<'d, P>,
    mut sm: StateMachine<'d, P, SM>,
    program: &LoadedProgram<'d, P>,
    step_pin: Peri<'d, impl embassy_rp::pio::PioPin>,
) -> StateMachine<'d, P, SM> {
    let mut pin = common.make_pio_pin(step_pin);
    // STEP is a TMC2209 Schmitt-trigger input. Without a pull, the line is
    // high-impedance during boot (before the SM is enabled) and during any
    // reconfig window, and capacitive coupling from a hand near the wire is
    // enough to clock false steps. embassy-rp's make_pio_pin explicitly turns
    // BOTH pulls off, so add the pull-down back. The PIO push-pull driver
    // still wins when it asserts the line; the pull-down only matters when
    // the PIO isn't driving.
    pin.set_pull(Pull::Down);
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
