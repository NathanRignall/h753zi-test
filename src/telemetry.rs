//! Central store for everything the sensors produce, plus a task that prints
//! a one-line summary to the console once a second.
//!
//! The sensor tasks call [`update`] whenever a new reading arrives; anyone
//! (the printer, a future network sender, ...) calls [`snapshot`] to get a
//! consistent copy.

use core::cell::RefCell;

use defmt::{Format, Formatter, info, write};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};

use uom::si::angle::degree;
use uom::si::f64::Angle;
use uom::si::length::meter;

use crate::gnss::{FixQuality, Gga, Rmc};
use crate::imu::{Acceleration, Euler, Orientation};

/// Latest reading from every sensor. `None` means nothing received yet;
/// the `*_updated` stamps let a consumer decide when data is stale.
#[derive(Clone, Copy, Default)]
pub struct Telemetry {
    /// Position, fix quality, satellites, altitude (from GNSS `GGA`).
    pub gga: Option<Gga>,
    /// Date, speed and course over ground (from GNSS `RMC`).
    pub rmc: Option<Rmc>,
    pub gnss_updated: Option<Instant>,

    /// Absolute orientation (rotation vector) from the IMU.
    pub orientation: Option<Orientation>,
    /// Raw acceleration, gravity included, from the IMU.
    pub acceleration: Option<Acceleration>,
    pub imu_updated: Option<Instant>,
}

// Convenience accessors for consumers of the store (e.g. a network sender);
// nothing in the firmware uses them yet.
#[allow(dead_code)]
impl Telemetry {
    /// Whether the GNSS currently has a position fix.
    pub fn has_fix(&self) -> bool {
        self.gga.is_some_and(|g| g.fix != FixQuality::None)
    }

    /// `(latitude, longitude)`, if there is a fix.
    pub fn position(&self) -> Option<(Angle, Angle)> {
        self.gga.filter(|g| g.fix != FixQuality::None).map(|g| (g.latitude, g.longitude))
    }

    /// Yaw / pitch / roll, if the IMU has reported.
    pub fn euler(&self) -> Option<Euler> {
        self.orientation.map(|o| o.euler())
    }
}

static STATE: Mutex<CriticalSectionRawMutex, RefCell<Telemetry>> = Mutex::new(RefCell::new(Telemetry {
    gga: None,
    rmc: None,
    gnss_updated: None,
    orientation: None,
    acceleration: None,
    imu_updated: None,
}));

/// Apply a change to the shared state.
pub fn update(f: impl FnOnce(&mut Telemetry)) {
    STATE.lock(|s| f(&mut s.borrow_mut()));
}

/// Copy of the current state.
pub fn snapshot() -> Telemetry {
    STATE.lock(|s| *s.borrow())
}

/// Readings older than this are flagged in the summary line.
const STALE_AFTER: Duration = Duration::from_secs(3);
const PRINT_INTERVAL: Duration = Duration::from_secs(1);

/// Prints the summary line once a second.
#[embassy_executor::task]
pub async fn print_task() -> ! {
    loop {
        Timer::after(PRINT_INTERVAL).await;
        let t = snapshot();
        let now = Instant::now();
        info!("{}", Summary { t, now });
    }
}

/// Human-readable one-liner, e.g.
/// `pos 52.179779,0.147946 alt 16.4m (Gnss, 6 sats) | yaw -70.8 pitch -2.9 roll -34.8 (imu 0/3)`
struct Summary {
    t: Telemetry,
    now: Instant,
}

impl Format for Summary {
    fn format(&self, f: Formatter) {
        let stale = |at: Option<Instant>| at.is_some_and(|at| self.now - at > STALE_AFTER);

        match self.t.gga {
            Some(g) if g.fix != FixQuality::None => write!(
                f,
                "pos {},{} alt {}m ({:?}, {} sats)",
                Fixed::new(g.latitude.get::<degree>(), 6),
                Fixed::new(g.longitude.get::<degree>(), 6),
                Fixed::new(g.altitude.get::<meter>() as f64, 1),
                g.fix,
                g.satellites
            ),
            Some(g) => write!(f, "pos -- (no fix, {} sats)", g.satellites),
            None => write!(f, "pos -- (no gnss data)"),
        }
        if stale(self.t.gnss_updated) {
            write!(f, " [stale]");
        }

        write!(f, " | ");

        match self.t.orientation {
            Some(o) => {
                let e = o.euler();
                write!(
                    f,
                    "yaw {} pitch {} roll {} (imu {}/3)",
                    Fixed::new(e.yaw.get::<degree>() as f64, 1),
                    Fixed::new(e.pitch.get::<degree>() as f64, 1),
                    Fixed::new(e.roll.get::<degree>() as f64, 1),
                    o.status
                );
            }
            None => write!(f, "yaw -- pitch -- roll -- (no imu data)"),
        }
        if stale(self.t.imu_updated) {
            write!(f, " [stale]");
        }
    }
}

/// A float rendered with a fixed number of decimals (defmt has no precision
/// formatting for floats, and integers cost far less to log anyway).
struct Fixed {
    neg: bool,
    int: u32,
    frac: u32,
    decimals: u8,
}

impl Fixed {
    fn new(v: f64, decimals: u8) -> Self {
        let scale = (0..decimals).fold(1.0f64, |acc, _| acc * 10.0);
        let scaled = libm::round(libm::fabs(v) * scale) as u64;
        Self {
            neg: v < 0.0,
            int: (scaled / scale as u64) as u32,
            frac: (scaled % scale as u64) as u32,
            decimals,
        }
    }
}

impl Format for Fixed {
    fn format(&self, f: Formatter) {
        if self.neg {
            write!(f, "-");
        }
        match self.decimals {
            1 => write!(f, "{}.{=u32:01}", self.int, self.frac),
            2 => write!(f, "{}.{=u32:02}", self.int, self.frac),
            _ => write!(f, "{}.{=u32:06}", self.int, self.frac),
        }
    }
}
