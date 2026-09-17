//! The telemetry frame the board publishes and the host decodes.
//!
//! Deliberately boring: a fixed-size little-endian record with a magic and a
//! version, no dependencies and no allocator, so the firmware and the host
//! tools can share exactly one definition.
//!
//! The frame starts with [`MAGIC`], which doubles as the NNG SUB topic — a
//! subscriber filtering on `MAGIC` receives every telemetry frame and nothing
//! else.

#![no_std]

use core::fmt;

/// First four bytes of every frame; also the PUB/SUB topic prefix.
pub const MAGIC: [u8; 4] = *b"H7TM";
/// Bumped whenever the layout below changes.
pub const VERSION: u8 = 1;
/// Encoded size of one frame.
pub const FRAME_LEN: usize = 69;

/// `flags` bit: the GNSS has a position fix.
pub const FLAG_GNSS_FIX: u8 = 1 << 0;
/// `flags` bit: the last GNSS reading is older than the staleness limit.
pub const FLAG_GNSS_STALE: u8 = 1 << 1;
/// `flags` bit: the IMU has reported an orientation.
pub const FLAG_IMU: u8 = 1 << 2;
/// `flags` bit: the last IMU reading is older than the staleness limit.
pub const FLAG_IMU_STALE: u8 = 1 << 3;
/// `flags` bit: `speed_mps` / `course_deg` are valid (GNSS `RMC`).
pub const FLAG_GNSS_COURSE: u8 = 1 << 4;

/// One telemetry sample.
///
/// Fields whose corresponding flag is clear are zero, not stale data.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Frame {
    /// Board uptime when the sample was taken.
    pub uptime_ms: u64,
    /// See the `FLAG_*` constants.
    pub flags: u8,
    /// GNSS fix quality, as reported in `GGA` (0 = none, 1 = GNSS, 2 = DGNSS…).
    pub fix: u8,
    /// Satellites used in the solution.
    pub satellites: u8,
    /// IMU calibration status, 0 (unreliable) … 3 (high).
    pub imu_status: u8,
    /// Degrees, positive north.
    pub latitude_deg: f64,
    /// Degrees, positive east.
    pub longitude_deg: f64,
    /// Metres above mean sea level.
    pub altitude_m: f32,
    /// Speed over ground, m/s.
    pub speed_mps: f32,
    /// Course over ground, degrees clockwise from true north.
    pub course_deg: f32,
    /// Degrees.
    pub yaw_deg: f32,
    /// Degrees.
    pub pitch_deg: f32,
    /// Degrees.
    pub roll_deg: f32,
    /// Acceleration including gravity, m/s², sensor frame.
    pub accel_mss: [f32; 3],
}

/// Why a byte slice is not a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Fewer than [`FRAME_LEN`] bytes.
    TooShort,
    /// The leading bytes are not [`MAGIC`].
    BadMagic,
    /// A version this build does not know how to read.
    BadVersion(u8),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => write!(f, "frame too short"),
            Self::BadMagic => write!(f, "bad magic"),
            Self::BadVersion(v) => write!(f, "unsupported frame version {v}"),
        }
    }
}

impl core::error::Error for DecodeError {}

/// Cursor that writes fields back to back; `encode` sizes the buffer exactly,
/// so the writes cannot overflow.
struct Writer<'a> {
    buf: &'a mut [u8; FRAME_LEN],
    at: usize,
}

impl Writer<'_> {
    fn u8(&mut self, v: u8) {
        self.buf[self.at] = v;
        self.at += 1;
    }
    fn bytes(&mut self, v: &[u8]) {
        self.buf[self.at..self.at + v.len()].copy_from_slice(v);
        self.at += v.len();
    }
    fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.bytes(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.bytes(&v.to_le_bytes());
    }
}

/// Cursor over a validated slice of at least [`FRAME_LEN`] bytes.
struct Reader<'a> {
    buf: &'a [u8],
    at: usize,
}

impl Reader<'_> {
    fn u8(&mut self) -> u8 {
        let v = self.buf[self.at];
        self.at += 1;
        v
    }
    fn array<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0u8; N];
        out.copy_from_slice(&self.buf[self.at..self.at + N]);
        self.at += N;
        out
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.array())
    }
    fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.array())
    }
    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.array())
    }
}

impl Frame {
    /// Serialize to exactly [`FRAME_LEN`] bytes.
    pub fn encode(&self) -> [u8; FRAME_LEN] {
        let mut buf = [0u8; FRAME_LEN];
        let mut w = Writer { buf: &mut buf, at: 0 };
        w.bytes(&MAGIC);
        w.u8(VERSION);
        w.u8(self.flags);
        w.u8(self.fix);
        w.u8(self.satellites);
        w.u8(self.imu_status);
        w.u64(self.uptime_ms);
        w.f64(self.latitude_deg);
        w.f64(self.longitude_deg);
        w.f32(self.altitude_m);
        w.f32(self.speed_mps);
        w.f32(self.course_deg);
        w.f32(self.yaw_deg);
        w.f32(self.pitch_deg);
        w.f32(self.roll_deg);
        for a in self.accel_mss {
            w.f32(a);
        }
        debug_assert_eq!(w.at, FRAME_LEN);
        buf
    }

    /// Parse a frame. Trailing bytes beyond [`FRAME_LEN`] are ignored.
    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        if buf.len() < FRAME_LEN {
            return Err(DecodeError::TooShort);
        }
        if buf[..4] != MAGIC {
            return Err(DecodeError::BadMagic);
        }
        if buf[4] != VERSION {
            return Err(DecodeError::BadVersion(buf[4]));
        }

        let mut r = Reader { buf, at: 5 };
        Ok(Self {
            flags: r.u8(),
            fix: r.u8(),
            satellites: r.u8(),
            imu_status: r.u8(),
            uptime_ms: r.u64(),
            latitude_deg: r.f64(),
            longitude_deg: r.f64(),
            altitude_m: r.f32(),
            speed_mps: r.f32(),
            course_deg: r.f32(),
            yaw_deg: r.f32(),
            pitch_deg: r.f32(),
            roll_deg: r.f32(),
            accel_mss: [r.f32(), r.f32(), r.f32()],
        })
    }

    pub fn has_fix(&self) -> bool {
        self.flags & FLAG_GNSS_FIX != 0
    }

    pub fn has_imu(&self) -> bool {
        self.flags & FLAG_IMU != 0
    }
}

/// One-line summary, the same shape the board prints to RTT.
impl fmt::Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:>9.3}s ", self.uptime_ms as f64 / 1000.0)?;

        if self.has_fix() {
            write!(
                f,
                "pos {:.6},{:.6} alt {:.1}m (fix {}, {} sats)",
                self.latitude_deg, self.longitude_deg, self.altitude_m, self.fix, self.satellites
            )?;
        } else {
            write!(f, "pos -- (no fix, {} sats)", self.satellites)?;
        }
        if self.flags & FLAG_GNSS_STALE != 0 {
            write!(f, " [stale]")?;
        }

        write!(f, " | ")?;

        if self.has_imu() {
            write!(
                f,
                "yaw {:.1} pitch {:.1} roll {:.1} (imu {}/3)",
                self.yaw_deg, self.pitch_deg, self.roll_deg, self.imu_status
            )?;
        } else {
            write!(f, "yaw -- pitch -- roll -- (no imu data)")?;
        }
        if self.flags & FLAG_IMU_STALE != 0 {
            write!(f, " [stale]")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let frame = Frame {
            uptime_ms: 123_456,
            flags: FLAG_GNSS_FIX | FLAG_IMU | FLAG_GNSS_COURSE,
            fix: 1,
            satellites: 12,
            imu_status: 3,
            latitude_deg: 52.179_861,
            longitude_deg: 0.147_983,
            altitude_m: 25.4,
            speed_mps: 1.5,
            course_deg: 91.25,
            yaw_deg: -49.7,
            pitch_deg: 0.2,
            roll_deg: 21.7,
            accel_mss: [0.1, -0.2, 9.81],
        };
        let bytes = frame.encode();
        assert_eq!(bytes.len(), FRAME_LEN);
        assert_eq!(Frame::decode(&bytes), Ok(frame));
    }

    #[test]
    fn rejects_junk() {
        assert_eq!(Frame::decode(&[]), Err(DecodeError::TooShort));

        let mut bytes = Frame::default().encode();
        bytes[0] = b'X';
        assert_eq!(Frame::decode(&bytes), Err(DecodeError::BadMagic));

        let mut bytes = Frame::default().encode();
        bytes[4] = 99;
        assert_eq!(Frame::decode(&bytes), Err(DecodeError::BadVersion(99)));
    }

    #[test]
    fn ignores_trailing_bytes() {
        let frame = Frame { satellites: 7, ..Frame::default() };
        let mut buf = frame.encode().to_vec();
        buf.extend_from_slice(b"extra");
        assert_eq!(Frame::decode(&buf), Ok(frame));
    }
}
