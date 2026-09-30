//! Continuous 50 Hz synchronous read of fused IMU samples (quaternion +
//! gyro + acceleration) from several bus IMUs, mirroring the upstream imu
//! `sync_read.py` and its `while 1` loop. Missing devices and device faults
//! are counted and never stop the loop; press Ctrl+C to stop (the serial
//! port is released on process exit).
//!
//! Per round every ID except the last prints only its read outcome
//! (ok / no response / device error / read error); the last ID prints the
//! full sample report, including raw and direction-bit signed values.
//!
//! usage: cargo run --example imu-sync_read -- PORT ID1 [ID2...]
use ftservo::{Imu, ImuSample, Request};
use std::{
    collections::BTreeMap,
    error::Error,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;

const INTERVAL: Duration = Duration::from_millis(20); // 50 Hz read cadence

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "imu-sync_read PORT ID1 [ID2...]";
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
    let imu = Imu::new(bus.clone());
    let mut last: BTreeMap<u8, ImuSample> = BTreeMap::new();
    // Per-round outcome, aligned with `ids`.
    let mut status: Vec<&'static str> = vec!["no data yet"; ids.len()];
    let mut round: u64 = 0;
    let mut failures: u64 = 0;
    let mut window = Instant::now();
    loop {
        round += 1;
        let started = Instant::now();
        // Fresh request per round: the 1 s deadline is absolute per read.
        let r = Request::with_timeout(Duration::from_secs(1))?;
        match imu.sync_read_samples(&r, &ids) {
            Ok(batch) => {
                failures += batch.errors.len() as u64;
                for (i, id) in ids.iter().enumerate() {
                    status[i] = if batch.values.contains_key(id) {
                        "ok"
                    } else if batch.missing.contains(id) {
                        "no response"
                    } else {
                        "device error"
                    };
                }
                for (id, sample) in &batch.values {
                    last.insert(*id, *sample);
                }
            }
            Err(e) => {
                failures += 1;
                status.fill("read error");
                println!("sync read failed: {e}");
            }
        }
        // Only the last requested ID prints its full sample; the others show
        // just whether this round succeeded.
        let last_index = ids.len() - 1;
        for (i, id) in ids.iter().enumerate() {
            if i == last_index {
                match last.get(id) {
                    Some(sample) => println!("{}", common::sample_report(*id, sample)),
                    None => println!("imu {id}: no sample yet ({})", status[i]),
                }
            } else {
                println!("imu {id}: {}", status[i]);
            }
        }
        let hz = 1.0 / window.elapsed().as_secs_f64();
        println!("rounds={round} failures={failures} rate={hz:.1} Hz");
        window = Instant::now();
        common::pace(started, INTERVAL);
    }
}
