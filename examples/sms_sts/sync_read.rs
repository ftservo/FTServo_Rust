//! Continuous 50 Hz synchronous read of the feedback block from several
//! servos, mirroring the upstream sms_sts `sync_read.py` and its `while 1`
//! loop. Missing devices and device faults are counted and never stop the
//! loop; press Ctrl+C to stop (the serial port is released on process exit).
//!
//! Every requested ID is printed each round: a field-by-field interpretation
//! when it answered, otherwise this round's failure reason.
//!
//! usage: cargo run --example sms_sts-sync_read -- PORT ID1 [ID2...]
use ftservo::{Request, Servo};
use std::{
    error::Error,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;

const INTERVAL: Duration = Duration::from_millis(20); // 50 Hz read cadence

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
    let servo = Servo::sms_sts(bus.clone());
    let mut round: u64 = 0;
    let mut failures: u64 = 0;
    let mut window = Instant::now();
    loop {
        round += 1;
        let started = Instant::now();
        // Fresh request per round: the 1 s deadline is absolute per read.
        let r = Request::with_timeout(Duration::from_secs(1))?;
        match servo.sync_read_feedback(&r, &ids) {
            Ok(batch) => {
                failures += batch.errors.len() as u64;
                for id in &ids {
                    match batch.values.get(id) {
                        Some(f) => println!("{}", common::feedback_report(*id, f)),
                        None if batch.missing.contains(id) => println!("servo {id}: no response"),
                        None => println!("servo {id}: device error"),
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
