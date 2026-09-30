//! Continuous 50 Hz read of the current SCSCL servo position, mirroring the
//! upstream scscl `read.py` (ReadPos) and its `while 1` loop. SCSCL
//! positions are unsigned big-endian words. A failing read is counted and
//! never stops the loop; press Ctrl+C to stop (the serial port is released
//! on process exit). Every round is printed; console output can become the
//! throughput limit (see the measured `rate=`), so redirect to a file when
//! the full 50 Hz cadence matters.
//!
//! usage: cargo run --example scscl-read -- PORT ID
use ftservo::{Request, Servo};
use std::{
    error::Error,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;

const INTERVAL: Duration = Duration::from_millis(20); // 50 Hz read cadence

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "scscl-read PORT ID";
    let a = common::args(usage, 2)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::scscl(bus.clone());
    let mut last: Option<i32> = None;
    let mut last_error = String::new();
    let mut round: u64 = 0;
    let mut failures: u64 = 0;
    let mut window = Instant::now();
    loop {
        round += 1;
        let started = Instant::now();
        // Fresh request per round: the 1 s deadline is absolute per read.
        let r = Request::with_timeout(Duration::from_secs(1))?;
        match servo.read_position(&r, id) {
            Ok(position) => {
                last = Some(position);
                last_error.clear();
            }
            Err(e) => {
                failures += 1;
                last_error = e.to_string();
            }
        }
        match last {
            Some(p) => println!("position: {p}"),
            None => println!("position: no data yet"),
        }
        if !last_error.is_empty() {
            println!("last error: {last_error}");
        }
        let hz = 1.0 / window.elapsed().as_secs_f64();
        println!("rounds={round} failures={failures} rate={hz:.1} Hz");
        window = Instant::now();
        common::pace(started, INTERVAL);
    }
}
