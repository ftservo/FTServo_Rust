//! Shared CLI helpers for the per-device examples.
//!
//! Argument validation happens before any serial I/O. All examples open a
//! 1 Mbaud 8N1 port with a 100 ms response timeout and a 1 s request deadline.

use ftservo::{Bus, Feedback, ImuSample, Request};
use std::{
    env,
    error::Error,
    time::{Duration, Instant},
};

pub const USAGE_PREFIX: &str = "usage: cargo run --example <name> --";

/// Parse a unicast device ID (0..=252; 253/255 are reserved, 254 is broadcast).
#[allow(dead_code)]
pub fn parse_id(v: &str) -> Result<u8, String> {
    let id: u8 = v
        .parse()
        .map_err(|_| format!("invalid ID: {v} (want 0..=252)"))?;
    if id > ftservo::MAX_ID {
        return Err(format!("ID out of range: {id} (want 0..=252)"));
    }
    Ok(id)
}

pub fn arg_error(usage: &str, reason: &str) -> Box<dyn Error> {
    format!("{USAGE_PREFIX} {usage}: {reason}").into()
}

/// Collect CLI arguments, reporting `usage` when the count does not match.
#[allow(dead_code)]
pub fn args(usage: &str, count: usize) -> Result<Vec<String>, Box<dyn Error>> {
    let a: Vec<_> = env::args().skip(1).collect();
    if a.len() != count {
        return Err(arg_error(usage, &format!("expected {count} arguments")));
    }
    Ok(a)
}

/// Open a 1 Mbaud port with the shared 100 ms response timeout.
#[allow(dead_code)]
pub fn open_bus(name: &str) -> Result<Bus, Box<dyn Error>> {
    Ok(Bus::open(name, 1_000_000, Duration::from_millis(100))?)
}

/// One-second absolute deadline shared by all examples.
#[allow(dead_code)]
pub fn request() -> Result<Request, Box<dyn Error>> {
    Ok(Request::with_timeout(Duration::from_secs(1))?)
}

/// Human-readable rendering of one IMU sample. Every field the SDK exposes
/// is printed: the binary16-decoded quaternion (X/Y/Z plus the nonnegative
/// reconstructed W) and its raw words, then the raw and direction-bit signed
/// values of gyro and acceleration. All numbers stay in raw register units;
/// `.signed()` only resolves the direction bit, it is not a physical unit
/// conversion.
#[allow(dead_code)]
pub fn sample_report(id: u8, s: &ImuSample) -> String {
    let q = &s.quaternion;
    let gyro = s.gyro.signed();
    let accel = s.acceleration.signed();
    format!(
        "imu {id}\n  \
         quaternion: x={:.5} y={:.5} z={:.5} w={:.5} (w reconstructed, nonnegative)\n  \
         quat raw  : x={} y={} z={}\n  \
         gyro raw  : x={} y={} z={}\n  \
         gyro valid: x={} y={} z={} (direction bit resolved)\n  \
         accel raw : x={} y={} z={}\n  \
         accel sgn : x={} y={} z={} (direction bit resolved)",
        q.x,
        q.y,
        q.z,
        q.w,
        q.raw.x,
        q.raw.y,
        q.raw.z,
        s.gyro.x,
        s.gyro.y,
        s.gyro.z,
        gyro.x,
        gyro.y,
        gyro.z,
        s.acceleration.x,
        s.acceleration.y,
        s.acceleration.z,
        accel.x,
        accel.y,
        accel.z,
    )
}

/// Multi-line interpretation of one servo feedback block. Position, speed,
/// load and current are already direction-bit decoded by the SDK; voltage and
/// temperature stay in raw register units.
#[allow(dead_code)]
pub fn feedback_report(id: u8, f: &Feedback) -> String {
    format!(
        "servo {id}\n  \
         position   : {} (direction bit resolved)\n  \
         speed      : {} (direction bit resolved)\n  \
         load       : {} (direction bit resolved)\n  \
         current    : {} (direction bit resolved)\n  \
         voltage    : {} (raw register units)\n  \
         temperature: {} (raw register units)\n  \
         moving     : {}",
        f.position, f.speed, f.load, f.current, f.voltage, f.temperature, f.moving
    )
}

/// Pacing helper for the continuous-read examples: sleep the bulk of the
/// period, then spin the last couple of milliseconds. Plain `thread::sleep`
/// on Windows can overshoot by ~15 ms, which alone would drop a "50 Hz"
/// loop to roughly 30 Hz or less.
#[allow(dead_code)]
pub fn pace(started: Instant, period: Duration) {
    if let Some(rest) = period.checked_sub(started.elapsed()) {
        if let Some(nap) = rest.checked_sub(Duration::from_millis(2)) {
            std::thread::sleep(nap);
        }
        while started.elapsed() < period {
            std::hint::spin_loop();
        }
    }
}
