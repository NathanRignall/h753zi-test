//! u-blox ZOE-M8Q GNSS receiver over I2C (u-blox calls the bus "DDC").
//!
//! The module speaks NMEA out of the box, so no configuration is sent. Its
//! I2C interface is a byte stream behind three registers:
//!
//! * `0xFD..=0xFE` — big-endian `u16` count of bytes waiting to be read
//! * `0xFF`        — the data stream; reads past the end return `0xFF`
//!
//! The task polls the count, drains the stream, splits it into NMEA
//! sentences, verifies each checksum and logs the position/fix fields from
//! `GGA` and `RMC`. Anything else (`GSV`, `GSA`, `VTG`, `GLL`) is ignored.

use defmt::{Format, info, warn};
use embassy_stm32::i2c;
use embassy_time::{Duration, Instant, Timer};
use uom::si::angle::degree;
use uom::si::f32::{Angle as Angle32, Length, Ratio, Velocity};
use uom::si::f64::Angle;
use uom::si::length::meter;
use uom::si::ratio::ratio;
use uom::si::velocity::knot;

use crate::{Bus, telemetry};

/// Default 7-bit address of every u-blox M8 receiver.
pub const ADDRESS: u8 = 0x42;

const REG_BYTES_AVAILABLE: u8 = 0xFD;
const REG_DATA_STREAM: u8 = 0xFF;

/// How often to ask the receiver whether it has anything for us. NMEA output
/// is 1 Hz by default, so 100 ms keeps latency low without hammering the bus.
const POLL_INTERVAL: Duration = Duration::from_millis(100);
/// Back-off after a NAK or bus error. The ZOE-M8Q NAKs its own address
/// while it has nothing to send (and for a while after power-up), so this
/// is normal early on and must not be treated as fatal.
const RETRY_INTERVAL: Duration = Duration::from_millis(250);

/// Longest NMEA sentence is 82 bytes including `$` and `\r\n`.
const MAX_SENTENCE: usize = 96;

/// Type of position fix, from the `GGA` fix-quality field.
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub enum FixQuality {
    None,
    Gnss,
    Dgnss,
    /// Any other non-zero value the receiver may emit (estimated, RTK, ...).
    Other(u8),
}

impl FixQuality {
    fn from_field(field: &str) -> Self {
        match field.parse::<u8>() {
            Ok(0) | Err(_) => Self::None,
            Ok(1) => Self::Gnss,
            Ok(2) => Self::Dgnss,
            Ok(n) => Self::Other(n),
        }
    }
}

/// The fields we care about from a `GGA` sentence.
#[derive(Clone, Copy)]
#[allow(dead_code)] // public data: read through `telemetry::snapshot()`
pub struct Gga {
    /// UTC time of fix, `hhmmss` as an integer (e.g. `123519` = 12:35:19).
    pub utc: u32,
    pub fix: FixQuality,
    pub satellites: u8,
    /// Horizontal dilution of precision; lower is better.
    pub hdop: Ratio,
    /// Positive north.
    pub latitude: Angle,
    /// Positive east.
    pub longitude: Angle,
    /// Altitude above mean sea level.
    pub altitude: Length,
}

/// The fields we care about from an `RMC` sentence.
#[derive(Clone, Copy)]
#[allow(dead_code)] // public data: read through `telemetry::snapshot()`
pub struct Rmc {
    /// UTC time of fix, `hhmmss` as an integer.
    pub utc: u32,
    /// UTC date, `ddmmyy` as an integer.
    pub date: u32,
    pub valid: bool,
    pub latitude: Angle,
    pub longitude: Angle,
    /// Speed over ground.
    pub speed: Velocity,
    /// Course over ground, clockwise from true north.
    pub course: Angle32,
}

/// Reads and logs GNSS data forever. `bus` is shared with the IMU; it is
/// locked only for the duration of each transfer.
#[embassy_executor::task]
pub async fn gnss_task(bus: &'static Bus) -> ! {
    info!("[gnss] ZOE-M8Q reader started (I2C addr 0x{:02x})", ADDRESS);

    let mut line = [0u8; MAX_SENTENCE];
    let mut line_len = 0usize;
    let mut chunk = [0u8; 64];
    let mut had_error = false;
    let mut last_fix = FixQuality::None;

    loop {
        let available = match bytes_available(bus).await {
            Ok(n) => {
                if had_error {
                    info!("[gnss] module responding");
                    had_error = false;
                }
                n
            }
            Err(e) => {
                if !had_error {
                    match e {
                        i2c::Error::Nack => info!("[gnss] module NAKs (booting or idle) — waiting"),
                        _ => warn!("[gnss] I2C error {:?} — is the module plugged in and powered?", e),
                    }
                    had_error = true;
                }
                Timer::after(RETRY_INTERVAL).await;
                continue;
            }
        };

        if available == 0 {
            Timer::after(POLL_INTERVAL).await;
            continue;
        }

        // Drain in bounded chunks so a burst of sentences never blocks the
        // executor for long, and so the DMA buffer can live on the stack.
        let mut remaining = available;
        while remaining > 0 {
            let n = remaining.min(chunk.len());
            let res = bus.lock().await.write_read(ADDRESS, &[REG_DATA_STREAM], &mut chunk[..n]).await;
            if let Err(e) = res {
                warn!("[gnss] read error: {:?}", e);
                break;
            }
            remaining -= n;

            for &b in &chunk[..n] {
                match b {
                    // "Nothing available" filler; also what a NAK'd read yields.
                    0xFF => continue,
                    b'$' => line_len = 0,
                    _ => {}
                }
                if line_len < line.len() {
                    line[line_len] = b;
                    line_len += 1;
                }
                if b == b'\n' {
                    if let Some(sentence) = checked_sentence(&line[..line_len]) {
                        handle_sentence(sentence, &mut last_fix);
                    }
                    line_len = 0;
                }
            }
        }
    }
}

async fn bytes_available(bus: &Bus) -> Result<usize, i2c::Error> {
    let mut count = [0u8; 2];
    bus.lock().await.write_read(ADDRESS, &[REG_BYTES_AVAILABLE], &mut count).await?;
    // The receiver answers 0xFFFF while it is still booting.
    Ok(match u16::from_be_bytes(count) {
        0xFFFF => 0,
        n => n as usize,
    })
}

/// Strips `$`, `*hh` and line ending, verifies the XOR checksum, and returns
/// the payload (`GNGGA,123519,...`) as `&str`.
fn checked_sentence(raw: &[u8]) -> Option<&str> {
    let raw = raw.strip_prefix(b"$")?;
    let raw = raw.strip_suffix(b"\r\n").or_else(|| raw.strip_suffix(b"\n"))?;
    let star = raw.iter().rposition(|&b| b == b'*')?;
    let (payload, hex) = raw.split_at(star);
    let hex = core::str::from_utf8(&hex[1..]).ok()?;
    let want = u8::from_str_radix(hex, 16).ok()?;
    let got = payload.iter().fold(0u8, |acc, &b| acc ^ b);
    if want != got {
        warn!("[gnss] bad checksum: {:02x} != {:02x}", got, want);
        return None;
    }
    core::str::from_utf8(payload).ok()
}

fn handle_sentence(sentence: &str, last_fix: &mut FixQuality) {
    let mut fields = sentence.split(',');
    let kind = fields.next().unwrap_or("");
    // Talker ID (`GP`, `GN`, `GL`, ...) is the first two chars; skip it.
    let kind = kind.get(2..).unwrap_or(kind);

    match kind {
        "GGA" => {
            if let Some(gga) = parse_gga(fields) {
                if gga.fix != *last_fix {
                    *last_fix = gga.fix;
                    info!("[gnss] fix changed: {:?} ({} satellites)", gga.fix, gga.satellites);
                }
                telemetry::update(|t| {
                    t.gga = Some(gga);
                    t.gnss_updated = Some(Instant::now());
                });
            }
        }
        "RMC" => {
            if let Some(rmc) = parse_rmc(fields) {
                telemetry::update(|t| {
                    t.rmc = Some(rmc);
                    t.gnss_updated = Some(Instant::now());
                });
            }
        }
        _ => {}
    }
}

/// `GGA,hhmmss.ss,lat,N,lon,E,fix,sats,hdop,alt,M,geoid,M,age,station`
fn parse_gga<'a>(mut f: impl Iterator<Item = &'a str>) -> Option<Gga> {
    let utc = parse_hhmmss(f.next()?);
    let lat = f.next()?;
    let ns = f.next()?;
    let lon = f.next()?;
    let ew = f.next()?;
    let fix = FixQuality::from_field(f.next()?);
    let satellites = f.next()?.parse().unwrap_or(0);
    let hdop = parse_f32(f.next()?);
    let altitude_m = parse_f32(f.next()?);
    Some(Gga {
        utc,
        fix,
        satellites,
        hdop: Ratio::new::<ratio>(hdop),
        latitude: Angle::new::<degree>(parse_coord(lat, ns)),
        longitude: Angle::new::<degree>(parse_coord(lon, ew)),
        altitude: Length::new::<meter>(altitude_m),
    })
}

/// `RMC,hhmmss.ss,A|V,lat,N,lon,E,speed_kn,course,ddmmyy,magvar,E,mode`
fn parse_rmc<'a>(mut f: impl Iterator<Item = &'a str>) -> Option<Rmc> {
    let utc = parse_hhmmss(f.next()?);
    let valid = f.next()? == "A";
    let lat = f.next()?;
    let ns = f.next()?;
    let lon = f.next()?;
    let ew = f.next()?;
    let speed_kn = parse_f32(f.next()?);
    let course_deg = parse_f32(f.next()?);
    let date = f.next()?.parse().unwrap_or(0);
    Some(Rmc {
        utc,
        date,
        valid,
        latitude: Angle::new::<degree>(parse_coord(lat, ns)),
        longitude: Angle::new::<degree>(parse_coord(lon, ew)),
        speed: Velocity::new::<knot>(speed_kn),
        course: Angle32::new::<degree>(course_deg),
    })
}

/// `hhmmss.ss` → `hhmmss`.
fn parse_hhmmss(field: &str) -> u32 {
    field.split('.').next().and_then(|s| s.parse().ok()).unwrap_or(0)
}

fn parse_f32(field: &str) -> f32 {
    parse_f64(field) as f32
}

/// `no_std` float parse without pulling in `core::str::parse::<f64>`'s
/// dependency on `libm` behaviour — NMEA only ever sends `[-]digits[.digits]`.
fn parse_f64(field: &str) -> f64 {
    let (neg, field) = match field.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, field),
    };
    let mut parts = field.split('.');
    let int: f64 = parts.next().and_then(|s| s.parse::<u64>().ok()).unwrap_or(0) as f64;
    let frac = match parts.next() {
        Some(digits) if !digits.is_empty() => {
            let n = digits.parse::<u64>().unwrap_or(0) as f64;
            let scale = (0..digits.len()).fold(1.0f64, |acc, _| acc * 10.0);
            n / scale
        }
        _ => 0.0,
    };
    let v = int + frac;
    if neg { -v } else { v }
}

/// NMEA `ddmm.mmmm` / `dddmm.mmmm` + hemisphere → signed decimal degrees.
fn parse_coord(field: &str, hemisphere: &str) -> f64 {
    let dot = field.find('.').unwrap_or(field.len());
    if dot < 2 {
        return 0.0;
    }
    let (deg, min) = field.split_at(dot - 2);
    let deg: f64 = deg.parse::<u32>().unwrap_or(0) as f64;
    let value = deg + parse_f64(min) / 60.0;
    match hemisphere {
        "S" | "W" => -value,
        _ => value,
    }
}
