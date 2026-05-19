//! TMC2209 UART bus — up to 4 drivers addressed 0..=3 on a shared half-duplex line.
//!
//! The TMC2209 exposes a single-wire UART (PDN_UART pin). When multiple drivers
//! share the same wire, each driver gets a unique address via its MS1/MS2 pins
//! at power-up (datasheet §5). A read datagram is a 4-byte request followed by
//! an 8-byte reply; a write datagram is 8 bytes with no reply.
//!
//! On a physically-shared TX/RX line the MCU sees its own transmitted bytes
//! echoed back before any driver reply. `tmc2209::Reader` handles this for us
//! by searching for the response sync byte, so we can feed it every byte we
//! read back without filtering.

#[cfg(feature = "uart")]
use embassy_rp::uart::BufferedUart;
#[cfg(feature = "uart")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "uart")]
use embassy_sync::mutex::Mutex;
#[cfg(feature = "uart")]
use embassy_time::{with_timeout, Duration};
#[cfg(feature = "uart")]
use embedded_io_async::{Read, Write};

#[cfg(feature = "uart")]
use tmc2209::{data::MicroStepResolution, read_request, write_request, ReadResponse, Reader};
use tmc2209::reg;

#[cfg(feature = "uart")]
use crate::protocol::{TMC_FLAG_INTERPOLATE, TMC_FLAG_SHAFT_INVERT, TMC_FLAG_STEALTHCHOP};

/// Joint index → TMC2209 UART address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum JointId {
    J0 = 0,
    J1 = 1,
    J2 = 2,
}

impl JointId {
    pub const ALL: [Self; 3] = [Self::J0, Self::J1, Self::J2];
    pub fn addr(self) -> u8 {
        self as u8
    }
    pub fn from_index(i: u8) -> Option<Self> {
        match i {
            0 => Some(Self::J0),
            1 => Some(Self::J1),
            2 => Some(Self::J2),
            _ => None,
        }
    }
}

/// Defaults used by `apply_default_config`, expressed as protocol flag bits.
/// Keep in sync so the host UI can show the firmware's initial state.
#[cfg(feature = "uart")]
pub const DEFAULT_TMC_FLAGS: u8 = TMC_FLAG_STEALTHCHOP | TMC_FLAG_INTERPOLATE;

pub struct DriverBus {
    #[cfg(feature = "uart")]
    uart: Mutex<CriticalSectionRawMutex, BufferedUart>,
}

impl DriverBus {
    #[cfg(feature = "uart")]
    pub fn new(uart: BufferedUart) -> Self {
        Self {
            uart: Mutex::new(uart),
        }
    }

    #[cfg(not(feature = "uart"))]
    pub fn new() -> Self {
        Self {}
    }

    /// Write a TMC2209 register on one driver. No reply is expected; we wait
    /// long enough to flush the bytes out of the TX buffer.
    #[cfg(feature = "uart")]
    pub async fn write_register<R>(&self, joint: JointId, value: R) -> Result<(), Error>
    where
        R: reg::WritableRegister,
    {
        let req = write_request(joint.addr(), value);
        let mut uart = self.uart.lock().await;
        uart.write_all(req.bytes()).await.map_err(|_| Error::Uart)?;
        // Drain the echoed bytes so the next read sees a clean line.
        let mut echo = [0u8; 8];
        let _ = with_timeout(Duration::from_millis(10), uart.read_exact(&mut echo)).await;
        Ok(())
    }

    #[cfg(not(feature = "uart"))]
    pub async fn write_register<R>(&self, _joint: JointId, _value: R) -> Result<(), Error>
    where
        R: reg::WritableRegister,
    {
        Ok(())
    }

    /// Capacity of the raw RX trace buffer captured by `read_register_traced`.
    pub const RX_TRACE_CAP: usize = 16;

    /// Like [`read_register`] but also returns the raw bytes received on the
    /// wire during the read attempt (truncated to `RX_TRACE_CAP`). Used by
    /// the diagnostic GetTmcConfig path so the host can show hex of what
    /// actually came back, distinguishing "chip silent" (0 bytes) from
    /// "chip replied with garbage" (some bytes, no parse).
    #[cfg(not(feature = "uart"))]
    pub async fn read_register_traced<R>(
        &self,
        _joint: JointId,
    ) -> (Result<R, Error>, [u8; Self::RX_TRACE_CAP], u8)
    where
        R: reg::ReadableRegister,
    {
        (Err(Error::Uart), [0u8; Self::RX_TRACE_CAP], 0)
    }

    #[cfg(feature = "uart")]
    pub async fn read_register_traced<R>(
        &self,
        joint: JointId,
    ) -> (Result<R, Error>, [u8; Self::RX_TRACE_CAP], u8)
    where
        R: reg::ReadableRegister,
    {
        let mut trace = [0u8; Self::RX_TRACE_CAP];
        let mut trace_len: u8 = 0;
        let req = read_request::<R>(joint.addr());
        let mut uart = self.uart.lock().await;

        // Drain stale bytes from prior writes' echoes etc.
        let mut scratch = [0u8; 16];
        while let Ok(Ok(_)) =
            with_timeout(Duration::from_millis(1), uart.read(&mut scratch)).await
        {}

        if uart.write_all(req.bytes()).await.is_err() {
            return (Err(Error::Uart), trace, trace_len);
        }

        let mut reader = Reader::default();
        let mut buf = [0u8; 16];
        let res = with_timeout(Duration::from_millis(50), async {
            loop {
                let n = match uart.read(&mut buf).await {
                    Ok(n) => n,
                    Err(_) => return Err(Error::Uart),
                };
                if n == 0 {
                    continue;
                }
                // Append to trace, capped.
                let room = (Self::RX_TRACE_CAP as u8).saturating_sub(trace_len) as usize;
                let take = n.min(room);
                if take > 0 {
                    let dst = trace_len as usize;
                    trace[dst..dst + take].copy_from_slice(&buf[..take]);
                    trace_len += take as u8;
                }
                if let (_, Some(r)) = reader.read_response(&buf[..n]) {
                    return Ok(r);
                }
            }
        })
        .await;

        let parsed: Result<R, Error> = match res {
            Ok(Ok(resp)) => resp.register::<R>().map_err(|_| Error::BadRegister),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(Error::Timeout),
        };
        (parsed, trace, trace_len)
    }

    /// Read a TMC2209 register from one driver.
    #[cfg(not(feature = "uart"))]
    pub async fn read_register<R>(&self, _joint: JointId) -> Result<R, Error>
    where
        R: reg::ReadableRegister,
    {
        Err(Error::Uart)
    }

    #[cfg(feature = "uart")]
    pub async fn read_register<R>(&self, joint: JointId) -> Result<R, Error>
    where
        R: reg::ReadableRegister,
    {
        let req = read_request::<R>(joint.addr());
        let mut uart = self.uart.lock().await;

        // Drain any stale bytes left in the RX buffer from a previous
        // operation (e.g., partial echo from a prior write_register that
        // straddled the 10ms drain window). Without this, `Reader` may
        // try to interpret junk as a response prefix and give up.
        let mut scratch = [0u8; 16];
        while let Ok(Ok(_)) =
            with_timeout(Duration::from_millis(1), uart.read(&mut scratch)).await
        {}

        uart.write_all(req.bytes()).await.map_err(|_| Error::Uart)?;

        // On a shared single-wire bus we first see a 4-byte echo of our own
        // request and then 8 bytes of reply. On a split TX/RX wiring the
        // echo never arrives — only the 8-byte reply. `Reader` walks the
        // stream and surfaces the response as soon as it sees the sync byte
        // + payload, so we don't need to count bytes. 50ms is generous
        // versus the ~16.7ms wall-clock of 16 bytes at 9600 baud, and
        // forgiving of any UART/IRQ latency in the buffered driver.
        let mut reader = Reader::default();
        let mut buf = [0u8; 16];
        let response: ReadResponse = with_timeout(Duration::from_millis(50), async {
            loop {
                let n = uart.read(&mut buf).await.map_err(|_| Error::Uart)?;
                if n == 0 {
                    continue;
                }
                if let (_, Some(r)) = reader.read_response(&buf[..n]) {
                    return Ok::<_, Error>(r);
                }
            }
        })
        .await
        .map_err(|_| Error::Timeout)??;

        response.register::<R>().map_err(|_| Error::BadRegister)
    }

    /// Baseline configuration shared across every joint:
    /// - UART-controlled current, microstep resolution set via MRES in CHOPCONF
    /// - stealthChop enabled (smooth, quiet low-speed motion)
    /// - internal step source via `VACTUAL` disabled (we drive STEP/DIR pins)
    #[cfg(feature = "uart")]
    pub async fn apply_default_config(&self, joint: JointId) -> Result<(), Error> {
        self.apply_gconf_chopconf(joint, DEFAULT_TMC_FLAGS).await?;

        // Motor current: IRUN=16, IHOLD=8, IHOLDDELAY=8 — conservative defaults.
        // Real current depends on sense-resistor value; tune per board.
        let mut ihold = reg::IHOLD_IRUN::default();
        ihold.set_ihold(8);
        ihold.set_irun(16);
        ihold.set_ihold_delay(8);
        self.write_register(joint, ihold).await?;

        Ok(())
    }

    #[cfg(not(feature = "uart"))]
    pub async fn apply_default_config(&self, _joint: JointId) -> Result<(), Error> {
        Ok(())
    }

    /// Re-write GCONF + CHOPCONF for one joint with the toggleable bits taken
    /// from `flags` (see `protocol::TMC_FLAG_*`). Other fields keep the
    /// firmware's baseline values.
    #[cfg(feature = "uart")]
    pub async fn apply_tmc_flags(&self, joint: JointId, flags: u8) -> Result<(), Error> {
        self.apply_gconf_chopconf(joint, flags).await
    }

    #[cfg(not(feature = "uart"))]
    pub async fn apply_tmc_flags(&self, _joint: JointId, _flags: u8) -> Result<(), Error> {
        Ok(())
    }

    #[cfg(feature = "uart")]
    async fn apply_gconf_chopconf(&self, joint: JointId, flags: u8) -> Result<(), Error> {
        let stealthchop = flags & TMC_FLAG_STEALTHCHOP != 0;
        let interpolate = flags & TMC_FLAG_INTERPOLATE != 0;
        let shaft_invert = flags & TMC_FLAG_SHAFT_INVERT != 0;

        // Enable the driver and tell it we want PDN_UART to control it
        // (pdn_disable = 1) instead of the standalone pin mode.
        let mut gconf = reg::GCONF::default();
        gconf.set_pdn_disable(true);
        gconf.set_mstep_reg_select(true); // MRES comes from CHOPCONF, not MS1/MS2
        gconf.set_en_spread_cycle(!stealthchop);
        gconf.set_shaft(shaft_invert);
        self.write_register(joint, gconf).await?;

        // 16 microsteps, TOFF=3 (driver enabled), TBL=2, HSTRT=5, HEND=2
        let mut chop = reg::CHOPCONF::default();
        chop.set_toff(3);
        chop.set_hstrt(5);
        chop.set_hend(2);
        chop.set_tbl(2);
        chop.set_mres(MicroStepResolution::from_microsteps(16));
        chop.set_intpol(interpolate);
        self.write_register(joint, chop).await?;

        Ok(())
    }
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    Uart,
    Crc,
    Timeout,
    BadRegister,
}
