//! Continuous 50 Hz position read of an HLS servo, mirroring the upstream hls
//! `read.py` and its `while 1` loop. A failing read is counted and never stops
//! the loop; press Ctrl+C to stop (the serial port is released on process
//! exit). Every round is printed; console output can become the throughput
//! limit (see the measured `rate=`), so redirect to a file when the full 50 Hz
//! cadence matters.
//!
//! usage: cargo run --example hls-read -- PORT ID
use ftservo::{Request, Servo};
use std::{
    error::Error,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;

const INTERVAL: Duration = Duration::from_millis(20); // 50 Hz read cadence

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "hls-read PORT ID";
    let a = common::args(usage, 2)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::hls(bus.clone());
    let mut last: Option<i32> = None;
    let mut last_error = String::new();
    let mut round: u64 = 0;
    let mut failures: u64 = 0;
    let mut window = Instant::now();
    // One `Request` for the whole run: it carries no deadline any more, and a read's time
    // limit comes from the protocol, so there is nothing to rebuild per round.
    let r = Request::new();
    loop {
        round += 1;
        let started = Instant::now();
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
            Some(p) => println!("servo {id}: position={p}"),
            None => println!("servo {id}: no data yet"),
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
