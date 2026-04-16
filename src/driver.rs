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

use embassy_rp::uart::BufferedUart;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{with_timeout, Duration};
use embedded_io_async::{Read, Write};

use tmc2209::{data::MicroStepResolution, read_request, reg, write_request, ReadResponse, Reader};

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
}

pub struct DriverBus {
    uart: Mutex<CriticalSectionRawMutex, BufferedUart>,
}

impl DriverBus {
    pub fn new(uart: BufferedUart) -> Self {
        Self {
            uart: Mutex::new(uart),
        }
    }

    /// Write a TMC2209 register on one driver. No reply is expected; we wait
    /// long enough to flush the bytes out of the TX buffer.
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

    /// Read a TMC2209 register from one driver.
    pub async fn read_register<R>(&self, joint: JointId) -> Result<R, Error>
    where
        R: reg::ReadableRegister,
    {
        let req = read_request::<R>(joint.addr());
        let mut uart = self.uart.lock().await;
        uart.write_all(req.bytes()).await.map_err(|_| Error::Uart)?;

        // On the shared wire we'll first read the 4-byte echo of our own
        // request, then 8 bytes of reply. `Reader` walks the stream and
        // surfaces the response as soon as it sees the sync byte + payload,
        // so we don't have to count bytes ourselves.
        let mut reader = Reader::default();
        let mut buf = [0u8; 16];
        let response: ReadResponse = with_timeout(Duration::from_millis(20), async {
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
    pub async fn apply_default_config(&self, joint: JointId) -> Result<(), Error> {
        // Enable the driver and tell it we want PDN_UART to control it
        // (pdn_disable = 1) instead of the standalone pin mode.
        let mut gconf = reg::GCONF::default();
        gconf.set_pdn_disable(true);
        gconf.set_mstep_reg_select(true); // MRES comes from CHOPCONF, not MS1/MS2
        gconf.set_en_spread_cycle(false); // stealthChop
        self.write_register(joint, gconf).await?;

        // 16 microsteps, TOFF=3 (driver enabled), TBL=2, HSTRT=5, HEND=2
        let mut chop = reg::CHOPCONF::default();
        chop.set_toff(3);
        chop.set_hstrt(5);
        chop.set_hend(2);
        chop.set_tbl(2);
        chop.set_mres(MicroStepResolution::from_microsteps(16));
        chop.set_intpol(true); // interpolate to 256 µsteps internally
        self.write_register(joint, chop).await?;

        // Motor current: IRUN=16, IHOLD=8, IHOLDDELAY=8 — conservative defaults.
        // Real current depends on sense-resistor value; tune per board.
        let mut ihold = reg::IHOLD_IRUN::default();
        ihold.set_ihold(8);
        ihold.set_irun(16);
        ihold.set_ihold_delay(8);
        self.write_register(joint, ihold).await?;

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
