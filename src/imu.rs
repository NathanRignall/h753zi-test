//! CEVA/Hillcrest BNO08x 9-DoF IMU over I2C, using the Sensor Hub Transport
//! Protocol (SHTP) and SH-2 report format directly — no vendor library.
//!
//! Wire protocol, as far as this driver needs it:
//!
//! * Every transfer starts with a 4-byte header: `len_lo, len_hi, channel,
//!   seq`. `len` counts the header, bit 15 marks a continuation chunk. A
//!   read of just the header does not consume the payload; the next read
//!   returns header + payload from the start.
//! * Channel 1 (executable) carries reset; channel 2 (control) carries
//!   Set Feature / Product ID commands and their responses; channel 3
//!   carries sensor input reports.
//! * An input packet is a `0xFB` time-base report followed by one or more
//!   fixed-length sensor reports, values in little-endian fixed point.
//!
//! No INT pin is available on a Qwiic-only hookup, so the sensor is polled.

use defmt::{debug, info, warn};
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c;
use embassy_time::{Duration, Instant, Timer};
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::radian;
use uom::si::f32::{Acceleration as Accel, Angle};

use crate::{Bus, telemetry};

/// Default 7-bit address (ADR/SA0 pin low). 0x4B if it is pulled high.
pub const ADDRESS: u8 = 0x4A;

const CH_EXECUTABLE: u8 = 1;
const CH_CONTROL: u8 = 2;
const CH_INPUT: u8 = 3;

const REPORT_ACCELEROMETER: u8 = 0x01;
const REPORT_ROTATION_VECTOR: u8 = 0x05;
const REPORT_PRODUCT_ID_REQUEST: u8 = 0xF9;
const REPORT_PRODUCT_ID_RESPONSE: u8 = 0xF8;
const REPORT_SET_FEATURE: u8 = 0xFD;
const REPORT_GET_FEATURE_RESPONSE: u8 = 0xFC;
const REPORT_COMMAND_RESPONSE: u8 = 0xF1;
const REPORT_COMMAND_REQUEST: u8 = 0xF2;
const COMMAND_INITIALIZE: u8 = 0x04;
const REPORT_TIMEBASE: u8 = 0xFB;
const REPORT_TIMESTAMP_REBASE: u8 = 0xFA;

/// How often the sensor is asked for each report, in microseconds.
const REPORT_INTERVAL_US: u32 = 50_000; // 20 Hz
/// Poll cadence when the last read came back empty.
const IDLE_POLL: Duration = Duration::from_millis(10);
/// If no report arrives for this long the sensor is re-initialised.
const SILENCE_RESTART: Duration = Duration::from_secs(3);
/// Largest packet we accept in one go. The post-reset advertisement is the
/// biggest thing the sensor sends (~280 bytes).
const MAX_PACKET: usize = 512;
/// Bytes per I2C read transaction.
const CHUNK: usize = 64;

#[derive(Clone, Copy)]
#[allow(dead_code)] // public data: read through `telemetry::snapshot()`
pub struct Orientation {
    /// Unit quaternion, sensor frame → world frame (dimensionless).
    pub i: f32,
    pub j: f32,
    pub k: f32,
    pub real: f32,
    /// Estimated heading accuracy.
    pub accuracy: Angle,
    /// 0 = unreliable … 3 = high (sensor's own calibration status).
    pub status: u8,
}

/// Tait–Bryan angles: yaw about Z, pitch about Y, roll about X.
#[derive(Clone, Copy)]
pub struct Euler {
    pub yaw: Angle,
    pub pitch: Angle,
    pub roll: Angle,
}

impl Orientation {
    pub fn euler(&self) -> Euler {
        let (x, y, z, w) = (self.i, self.j, self.k, self.real);
        let yaw = libm::atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        let pitch = libm::asinf((2.0 * (w * y - x * z)).clamp(-1.0, 1.0));
        let roll = libm::atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        Euler {
            yaw: Angle::new::<radian>(yaw),
            pitch: Angle::new::<radian>(pitch),
            roll: Angle::new::<radian>(roll),
        }
    }
}

/// Sensor-frame acceleration, gravity included.
#[derive(Clone, Copy)]
#[allow(dead_code)] // public data: read through `telemetry::snapshot()`
pub struct Acceleration {
    pub x: Accel,
    pub y: Accel,
    pub z: Accel,
    pub status: u8,
}

struct Shtp<'a> {
    bus: &'a Bus,
    /// Per-channel outgoing sequence numbers.
    seq: [u8; 6],
    /// Sequence number for SH-2 command requests (separate from SHTP's).
    command_seq: u8,
    buf: [u8; MAX_PACKET],
}

/// A received packet: channel and payload length within `Shtp::buf`.
struct Packet {
    channel: u8,
    len: usize,
}

impl<'a> Shtp<'a> {
    fn new(bus: &'a Bus) -> Self {
        Self { bus, seq: [0; 6], command_seq: 0, buf: [0; MAX_PACKET] }
    }

    fn next_command_seq(&mut self) -> u8 {
        let s = self.command_seq;
        self.command_seq = s.wrapping_add(1);
        s
    }

    fn payload(&self, p: &Packet) -> &[u8] {
        &self.buf[..p.len]
    }

    async fn write(&mut self, channel: u8, payload: &[u8]) -> Result<(), i2c::Error> {
        let len = payload.len() + 4;
        let seq = &mut self.seq[channel as usize];
        let mut frame = [0u8; 32];
        frame[0] = len as u8;
        frame[1] = (len >> 8) as u8;
        frame[2] = channel;
        frame[3] = *seq;
        frame[4..len].copy_from_slice(payload);
        *seq = seq.wrapping_add(1);
        debug!("[imu] tx {:02x}", &frame[..len]);
        self.bus.lock().await.write(ADDRESS, &frame[..len]).await
    }

    /// Reads one packet if the sensor has one. `Ok(None)` means nothing
    /// pending. Oversized packets are consumed and dropped.
    ///
    /// Every I2C read transaction starts with a fresh 4-byte header, and a
    /// read shorter than the packet makes the sensor send the remainder as a
    /// continuation on the next read. So: read one chunk, then keep reading
    /// chunks (each with its own header to skip) until the packet is whole.
    async fn read(&mut self) -> Result<Option<Packet>, i2c::Error> {
        let mut bus = self.bus.lock().await;
        let mut chunk = [0u8; CHUNK];

        bus.read(ADDRESS, &mut chunk).await?;
        let total = (u16::from_le_bytes([chunk[0], chunk[1]]) & 0x7FFF) as usize;
        if total < 4 || total == 0x7FFF {
            return Ok(None);
        }
        let channel = chunk[2];
        let len = total - 4;
        let keep = len <= self.buf.len();
        if !keep {
            warn!("[imu] dropping {}-byte packet on channel {}", total, channel);
        }

        let got = len.min(CHUNK - 4);
        if keep {
            self.buf[..got].copy_from_slice(&chunk[4..4 + got]);
        }
        let mut pos = got;
        while pos < len {
            let n = (len - pos + 4).min(CHUNK);
            bus.read(ADDRESS, &mut chunk[..n]).await?;
            if keep {
                self.buf[pos..pos + n - 4].copy_from_slice(&chunk[4..n]);
            }
            pos += n - 4;
        }
        debug!("[imu] rx ch={} len={} body={:02x}", channel, len, &self.buf[..len.min(8)]);
        Ok(if keep { Some(Packet { channel, len }) } else { None })
    }
}

/// Reads orientation and acceleration forever and publishes them.
///
/// `reset` drives the sensor's RST pin (active low). It is pulsed at every
/// bring-up: the BNO08x's hub can hang if the host vanishes mid-transfer
/// (which is what a reflash does), and in that state it still answers
/// queries but ignores the soft reset, so only RST recovers it.
#[embassy_executor::task]
pub async fn imu_task(bus: &'static Bus, mut reset: Output<'static>) -> ! {
    info!("[imu] BNO08x reader started (I2C addr 0x{:02x})", ADDRESS);
    let mut shtp = Shtp::new(bus);

    loop {
        reset.set_low();
        Timer::after(Duration::from_millis(10)).await;
        reset.set_high();
        Timer::after(Duration::from_millis(100)).await;

        match bring_up(&mut shtp).await {
            Ok(()) => {}
            Err(e) => {
                warn!("[imu] bring-up failed ({:?}) — retrying in 2 s", e);
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        }

        // Steady state: poll, decode, publish. Leave this loop only on a
        // bus error or a spontaneous reset, which restarts bring-up.
        let mut last_report = Instant::now();

        loop {
            match shtp.read().await {
                Ok(Some(p)) => match p.channel {
                    CH_INPUT => {
                        let mut orientation = None;
                        let mut acceleration = None;
                        if decode_input(shtp.payload(&p), &mut orientation, &mut acceleration) > 0 {
                            last_report = Instant::now();
                            telemetry::update(|t| {
                                if orientation.is_some() {
                                    t.orientation = orientation;
                                }
                                if acceleration.is_some() {
                                    t.acceleration = acceleration;
                                }
                                t.imu_updated = Some(last_report);
                            });
                        }
                    }
                    CH_CONTROL => decode_control(shtp.payload(&p)),
                    CH_EXECUTABLE => {
                        if shtp.payload(&p).first() == Some(&1) {
                            warn!("[imu] sensor reset itself — reconfiguring");
                            break;
                        }
                    }
                    _ => {}
                },
                Ok(None) => Timer::after(IDLE_POLL).await,
                Err(e) => {
                    warn!("[imu] read error: {:?} — restarting", e);
                    break;
                }
            }

            if Instant::now() - last_report > SILENCE_RESTART {
                warn!("[imu] no reports for {} s — re-initialising", SILENCE_RESTART.as_secs());
                break;
            }
        }
    }
}

/// Soft-reset the sensor, wait for it to come back, identify it and enable
/// the reports we want, checking each one is acknowledged.
async fn bring_up(shtp: &mut Shtp<'_>) -> Result<(), i2c::Error> {
    // The sensor NAKs while busy; a few attempts are normal right after power-up.
    let mut attempts = 0;
    loop {
        match shtp.write(CH_EXECUTABLE, &[1]).await {
            Ok(()) => break,
            Err(e) if attempts < 10 => {
                attempts += 1;
                if attempts == 1 {
                    info!("[imu] reset write failed ({:?}) — retrying", e);
                }
                Timer::after(Duration::from_millis(100)).await;
            }
            Err(e) => return Err(e),
        }
    }
    Timer::after(Duration::from_millis(300)).await;

    // The sensor announces itself in stages: advertisement, "reset complete"
    // on the executable channel, then an "initialize" command response on
    // the control channel. Commands sent before the last one are dropped.
    let is_init = |ch: u8, p: &[u8]| {
        ch == CH_CONTROL && p.first() == Some(&REPORT_COMMAND_RESPONSE) && p.get(2) == Some(&COMMAND_INITIALIZE)
    };
    if wait_for(shtp, Duration::from_secs(2), |ch, p| ch == CH_EXECUTABLE && p.first() == Some(&1)).await? {
        info!("[imu] reset complete");
        if wait_for(shtp, Duration::from_secs(2), is_init).await? {
            info!("[imu] initialised");
        } else {
            warn!("[imu] no initialise response after reset");
        }
    } else {
        // Fallback if RST is not wired: ask the hub to re-initialise itself.
        warn!("[imu] no reset-complete seen — sending SH-2 reinitialize");
        let mut cmd = [0u8; 12];
        cmd[0] = REPORT_COMMAND_REQUEST;
        cmd[1] = shtp.next_command_seq();
        cmd[2] = COMMAND_INITIALIZE;
        cmd[3] = 1; // P0 = 1: reinitialise the sensor hub
        shtp.write(CH_CONTROL, &cmd).await?;
        if wait_for(shtp, Duration::from_secs(3), is_init).await? {
            info!("[imu] reinitialised");
        } else {
            warn!("[imu] reinitialize not acknowledged either — continuing anyway");
        }
    }

    shtp.write(CH_CONTROL, &[REPORT_PRODUCT_ID_REQUEST, 0]).await?;

    for report in [REPORT_ROTATION_VECTOR, REPORT_ACCELEROMETER] {
        let mut confirmed = false;
        for attempt in 1..=3 {
            set_feature(shtp, report, REPORT_INTERVAL_US).await?;
            let is_ack =
                |ch: u8, p: &[u8]| ch == CH_CONTROL && p.first() == Some(&REPORT_GET_FEATURE_RESPONSE) && p.get(1) == Some(&report);
            if wait_for(shtp, Duration::from_secs(1), is_ack).await? {
                confirmed = true;
                break;
            }
            warn!("[imu] feature 0x{:02x} not acknowledged (attempt {}) — resending", report, attempt);
        }
        if !confirmed {
            warn!("[imu] feature 0x{:02x} never acknowledged", report);
        }
    }
    info!("[imu] rotation vector + accelerometer requested at {} Hz", 1_000_000 / REPORT_INTERVAL_US);
    Ok(())
}

/// Reads packets until `pred` matches one or `timeout` passes. Control
/// packets seen on the way are logged as usual; input reports are dropped
/// (this only runs during bring-up). NAKs are tolerated: the sensor is
/// often still restarting.
async fn wait_for(
    shtp: &mut Shtp<'_>,
    timeout: Duration,
    pred: impl Fn(u8, &[u8]) -> bool,
) -> Result<bool, i2c::Error> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match shtp.read().await {
            Ok(Some(p)) => {
                let payload = shtp.payload(&p);
                if pred(p.channel, payload) {
                    return Ok(true);
                }
                if p.channel == CH_CONTROL {
                    decode_control(payload);
                }
            }
            Ok(None) => Timer::after(Duration::from_millis(10)).await,
            Err(i2c::Error::Nack) => Timer::after(Duration::from_millis(50)).await,
            Err(e) => return Err(e),
        }
    }
    Ok(false)
}

async fn set_feature(shtp: &mut Shtp<'_>, report: u8, interval_us: u32) -> Result<(), i2c::Error> {
    let mut cmd = [0u8; 17];
    cmd[0] = REPORT_SET_FEATURE;
    cmd[1] = report;
    // [2] feature flags, [3..5] change sensitivity: zero.
    cmd[5..9].copy_from_slice(&interval_us.to_le_bytes());
    // [9..13] batch interval, [13..17] sensor-specific config: zero.
    shtp.write(CH_CONTROL, &cmd).await
}

/// Control-channel responses. Only logged; nothing depends on them.
fn decode_control(payload: &[u8]) {
    match payload.first() {
        Some(&REPORT_PRODUCT_ID_RESPONSE) if payload.len() >= 14 => {
            let part = u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
            let build = u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);
            let patch = u16::from_le_bytes([payload[12], payload[13]]);
            info!(
                "[imu] BNO08x part {} firmware {}.{}.{} build {} (reset cause {})",
                part, payload[2], payload[3], patch, build, payload[1]
            );
        }
        Some(&REPORT_GET_FEATURE_RESPONSE) if payload.len() >= 9 => {
            let interval = u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
            info!("[imu] feature 0x{:02x} confirmed, interval {} µs", payload[1], interval);
        }
        Some(&REPORT_COMMAND_RESPONSE) if payload.len() >= 6 => {
            info!("[imu] command 0x{:02x} response: status {}", payload[2], payload[5]);
        }
        Some(&REPORT_COMMAND_RESPONSE) | Some(&REPORT_PRODUCT_ID_RESPONSE) => {}
        Some(id) => info!("[imu] control report 0x{:02x} ({} bytes)", id, payload.len()),
        None => {}
    }
}

/// Walks the reports in an input packet, keeping the newest orientation and
/// acceleration. Returns the number of sensor reports decoded.
fn decode_input(payload: &[u8], orientation: &mut Option<Orientation>, acceleration: &mut Option<Acceleration>) -> u32 {
    let mut i = 0;
    let mut n = 0;
    while i < payload.len() {
        let id = payload[i];
        let len = match id {
            REPORT_TIMEBASE | REPORT_TIMESTAMP_REBASE => 5,
            REPORT_ACCELEROMETER | 0x02 | 0x03 | 0x04 => 10,
            REPORT_ROTATION_VECTOR | 0x09 => 14,
            0x08 => 12,
            _ => {
                warn!("[imu] unknown input report 0x{:02x} at offset {} — skipping rest", id, i);
                break;
            }
        };
        if i + len > payload.len() {
            break;
        }
        let r = &payload[i..i + len];
        let status = r.get(2).map_or(0, |s| s & 0x03);
        match id {
            REPORT_ROTATION_VECTOR => {
                *orientation = Some(Orientation {
                    i: q(r, 4, 14),
                    j: q(r, 6, 14),
                    k: q(r, 8, 14),
                    real: q(r, 10, 14),
                    accuracy: Angle::new::<radian>(q(r, 12, 12)),
                    status,
                });
                n += 1;
            }
            REPORT_ACCELEROMETER => {
                *acceleration = Some(Acceleration {
                    x: Accel::new::<meter_per_second_squared>(q(r, 4, 8)),
                    y: Accel::new::<meter_per_second_squared>(q(r, 6, 8)),
                    z: Accel::new::<meter_per_second_squared>(q(r, 8, 8)),
                    status,
                });
                n += 1;
            }
            _ => {}
        }
        i += len;
    }
    n
}

/// Little-endian signed fixed-point field with `frac` fractional bits.
fn q(r: &[u8], at: usize, frac: u32) -> f32 {
    i16::from_le_bytes([r[at], r[at + 1]]) as f32 / (1u32 << frac) as f32
}
