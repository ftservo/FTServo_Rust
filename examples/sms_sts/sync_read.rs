//! Continuous 50 Hz synchronous read of the feedback block from several
//! servos, mirroring the upstream sms_sts `sync_read.py` and its `while 1`
//! loop. Missing devices and device faults are counted and never stop the
//! loop; press Ctrl+C to stop (the serial port is released on process exit).
//!
//! Every requested ID is printed each round: a field-by-field interpretation
//! when it answered, otherwise this round's failure reason.
//!
//! Reading the block whole is this example's job, not the SDK's: the transaction is
//! `Bus::sync_read_decoded`, called straight from the loop, and the 15-byte payload's layout
//! is the `decode_feedback` below, where the family that defines it is the one being driven.
//!
//! The `Request`, the `Batch` and the report `String` are allocated once before the loop
//! and cleared in place each round — `imu/sync_read.rs` has the same shape, and `src/bus.rs`
//! says why it matters.
//!
//! usage: cargo run --example sms_sts-sync_read -- PORT ID1 [ID2...]
use ftservo::{decode_sign_magnitude, registers, Batch, Feedback, Request};
use std::{
    error::Error,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;

const INTERVAL: Duration = Duration::from_millis(20); // 50 Hz read cadence

/// Decode the 15-byte feedback block that starts at `registers::PRESENT_POSITION`, as
/// SMS/STS reports it: little-endian words, `position`/`speed`/`current` 15-bit
/// sign-magnitude and `load` 10-bit — a direction bit, not two's complement, which is what
/// `decode_sign_magnitude` resolves.
///
/// Spelled out here rather than taken from the SDK because the block's layout is a family
/// fact: SMS/STS shares it with HLS, SCSCL does not.
fn decode_feedback(p: &[u8]) -> ftservo::Result<Feedback> {
    if p.len() != 15 {
        return Err(ftservo::Error::Packet("feedback block requires 15 bytes"));
    }
    let word = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
    Ok(Feedback {
        position: decode_sign_magnitude(word(0), 15)?,
        speed: decode_sign_magnitude(word(2), 15)?,
        load: decode_sign_magnitude(word(4), 10)?,
        current: decode_sign_magnitude(word(13), 15)?,
        voltage: p[6],
        temperature: p[7],
        moving: p[10] != 0,
    })
}

/// Multi-line interpretation of one decoded block, written into `out` (cleared first,
/// capacity kept). Position, speed, load and current are already direction-bit resolved by
/// `decode_feedback`; voltage and temperature stay in raw register units.
fn write_feedback_report(out: &mut String, id: u8, f: &Feedback) {
    use std::fmt::Write as _;
    out.clear();
    let _ = write!(
        out,
        "servo {id}\n  \
         position   : {} (direction bit resolved)\n  \
         speed      : {} (direction bit resolved)\n  \
         load       : {} (direction bit resolved)\n  \
         current    : {} (direction bit resolved)\n  \
         voltage    : {} (raw register units)\n  \
         temperature: {} (raw register units)\n  \
         moving     : {}",
        f.position, f.speed, f.load, f.current, f.voltage, f.temperature, f.moving
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "sms_sts-sync_read PORT ID1 [ID2...]";
    let a: Vec<_> = std::env::args().skip(1).collect();
    if a.len() < 2 {
        return Err(common::arg_error(
            usage,
            "expected at least PORT and one ID",
        ));
    }
    let mut ids = Vec::new();
    for v in &a[1..] {
        let id = common::parse_id(v).map_err(|e| common::arg_error(usage, &e))?;
        if ids.contains(&id) {
            return Err(common::arg_error(usage, "duplicate ID"));
        }
        ids.push(id);
    }

    let bus = common::open_bus(&a[0])?;

    // Allocated once, outside the loop. Everything below is cleared and refilled in place.
    let r = Request::new();
    let mut batch = Batch::new();
    let mut line = String::new();
    let mut round: u64 = 0;
    let mut failures: u64 = 0;
    let mut window = Instant::now();
    loop {
        round += 1;
        let started = Instant::now();
        // One broadcast transaction for every ID; the payload is decoded straight out of the
        // receive buffer, and the results land in `batch`, which is reused across rounds.
        match bus.sync_read_decoded(
            &r,
            registers::PRESENT_POSITION,
            15,
            &ids,
            decode_feedback,
            &mut batch,
        ) {
            Ok(()) => {
                failures += batch.errors.len() as u64;
                for id in &ids {
                    if let Some(f) = batch.get(*id) {
                        write_feedback_report(&mut line, *id, f);
                        println!("{line}");
                    } else if batch.missing.contains(id) {
                        println!("servo {id}: no response");
                    } else {
                        println!("servo {id}: device error");
                    }
                }
            }
            Err(e) => {
                failures += 1;
                for id in &ids {
                    println!("servo {id}: read error");
                }
                println!("sync read failed: {e}");
            }
        }
        let hz = 1.0 / window.elapsed().as_secs_f64();
        println!("rounds={round} failures={failures} rate={hz:.1} Hz");
        window = Instant::now();
        common::pace(started, INTERVAL);
    }
}
