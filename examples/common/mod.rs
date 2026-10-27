//! Shared CLI plumbing for the examples: arguments, the port, the request, the cadence.
//!
//! Device-agnostic on purpose — nothing here knows what a servo or an IMU is. Anything that
//! does (the IMU sample report, the servo feedback block) lives with the example that uses
//! it: every example includes this file, so a device-shaped helper here is dead code in all
//! the others and would need `#[allow(dead_code)]` to stay quiet.
//!
//! Argument validation happens before any serial I/O. All examples open an 8N1 port, 1 Mbaud
//! unless `FTSERVO_BAUD` says otherwise. Reads are not given a timeout of the caller's
//! choosing: each transaction's budget comes from the protocol, see `Bus::command` — and
//! `FTSERVO_READ_DELAY_MS` adjusts the base of that budget, see `open_bus`.

use ftservo::{Bus, Request};
use std::{
    env,
    error::Error,
    time::{Duration, Instant},
};

pub const USAGE_PREFIX: &str = "usage: cargo run --example <name> --";

/// Parse a unicast device ID (0..=253; 254 is broadcast, 255 is not an ID).
#[allow(dead_code)]
pub fn parse_id(v: &str) -> Result<u8, String> {
    let id: u8 = v
        .parse()
        .map_err(|_| format!("invalid ID: {v} (want 0..=253)"))?;
    if id > ftservo::MAX_ID {
        return Err(format!("ID out of range: {id} (want 0..=253)"));
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

/// Open a port. The timeout argument is the **write** budget; reads are timed per transaction,
/// see `Bus::open`.
///
/// Two environment variables override the defaults, because both describe the *setup* rather
/// than the protocol — an adapter that hands replies over late, or a servo reprogrammed off its
/// factory rate, is normally found by running an example rather than by editing one:
///
/// * `FTSERVO_BAUD` — the wire rate in decimal, default 1 Mbaud (what every family ships at);
/// * `FTSERVO_READ_DELAY_MS` — the base of every read window, default `DEFAULT_READ_DELAY`.
///
/// Raise the latter when reads fail as `response timeout (partial packet: false)`: nothing was
/// readable in time, which on a USB adapter is usually its own delivery batching rather than
/// the device. `partial packet: true` is the other case — bytes arrived but the frame did not
/// finish — and points at the rate instead.
#[allow(dead_code)]
pub fn open_bus(name: &str) -> Result<Bus, Box<dyn Error>> {
    let baud = match env::var("FTSERVO_BAUD") {
        Ok(v) => v
            .trim()
            .parse()
            .map_err(|_| format!("FTSERVO_BAUD must be an integer: {v}"))?,
        Err(_) => 1_000_000,
    };
    let bus = Bus::open(name, baud, Duration::from_millis(100))?;
    if let Ok(v) = env::var("FTSERVO_READ_DELAY_MS") {
        let ms: u64 = v
            .trim()
            .parse()
            .map_err(|_| format!("FTSERVO_READ_DELAY_MS must be an integer: {v}"))?;
        bus.set_read_delay(Duration::from_millis(ms));
    }
    Ok(bus)
}

/// A shared `Request` for the examples. It carries no deadline: a transaction is timed from
/// the protocol, see `Bus::command`.
#[allow(dead_code)]
pub fn request() -> Result<Request, Box<dyn Error>> {
    Ok(Request::new())
}

/// Pacing helper for the continuous-read examples: sleep the bulk of the
/// period, then spin the last couple of milliseconds. Plain `thread::sleep`
/// on Windows can overshoot by ~15 ms, which alone would drop a "50 Hz"
/// loop to roughly 30 Hz or less.
///
/// **An overrun is skipped, not made up.** When the round already used more than `period`,
/// `checked_sub` yields `None` and the next round starts immediately: the schedule is left
/// where it was and the missed slot is dropped. Waiting out the remainder instead — i.e.
/// sleeping `period` after the work — would fold every wakeup latency into the period and
/// let the loop drift permanently slow, which is the failure the whole shape guards
/// against.
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
