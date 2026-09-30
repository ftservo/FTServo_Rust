//! Position write, mirroring the upstream scscl `write.py`: ping-pong
//! between 20 and 1000 at speed 1500 (time = 0).
//!
//! Deliberate differences from the upstream example:
//! - a bounded number of strokes instead of an infinite `while 1` loop;
//! - no `EnableTorque` call, matching the upstream script. If the servo does
//!   not move, enable torque explicitly with `enable_torque` first.
//!
//! usage: cargo run --example scscl-write -- PORT ID
use ftservo::{Motion, Servo};
use std::{error::Error, thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;

const POSITIONS: [i32; 2] = [1000, 20];
const SPEED: u16 = 1500;
const STROKES: u32 = 3;

/// Upstream travel-time estimate: [(P1-P0)/V] + 0.05 seconds.
fn travel_time(p0: i32, p1: i32) -> Duration {
    let d = f64::from((p1.max(p0) - p1.min(p0)) as u16);
    Duration::from_secs_f64(d / f64::from(SPEED) + 0.05)
}

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "scscl-write PORT ID";
    let a = common::args(usage, 2)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::scscl(bus.clone());
    let r = common::request()?;
    let mut previous = 0;
    for stroke in 0..STROKES {
        let position = if stroke % 2 == 0 {
            POSITIONS[0]
        } else {
            POSITIONS[1]
        };
        servo.write_position(
            &r,
            Motion {
                id,
                position,
                speed: SPEED,
                ..Motion::default()
            },
        )?;
        println!("wrote position {position}");
        thread::sleep(travel_time(previous, position));
        previous = position;
    }
    bus.close()?;
    Ok(())
}
