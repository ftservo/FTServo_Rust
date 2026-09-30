//! Position write, mirroring the upstream hls `write.py`: ping-pong between
//! 0 and 4095 at speed 60, acceleration 50, torque 500 (HLS carries a torque
//! field in its motion write).
//!
//! Deliberate differences from the upstream example:
//! - a bounded number of strokes instead of an infinite `while 1` loop;
//! - no `EnableTorque` call, matching the upstream script. If the servo does
//!   not move, enable torque explicitly with `enable_torque` first.
//!
//! usage: cargo run --example hls-write -- PORT ID
use ftservo::{Motion, Servo};
use std::{error::Error, thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;

const POSITIONS: [i32; 2] = [4095, 0];
const SPEED: u16 = 60;
const ACCELERATION: u8 = 50;
const TORQUE: u16 = 500;
const STROKES: u32 = 3;

/// Upstream travel-time estimate:
/// [(P1-P0)/(V*50)] + [(V*50)/(A*100)] + 0.05 seconds.
fn travel_time(p0: i32, p1: i32) -> Duration {
    let (p0, p1) = (f64::from(p0), f64::from(p1));
    let v = f64::from(SPEED) * 50.0;
    let a = f64::from(ACCELERATION) * 100.0;
    Duration::from_secs_f64(((p1 - p0).abs() / v) + (v / a) + 0.05)
}

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "hls-write PORT ID";
    let a = common::args(usage, 2)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::hls(bus.clone());
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
                acceleration: ACCELERATION,
                torque: TORQUE,
                ..Motion::default()
            },
        )?;
        println!("wrote position {position} (torque {TORQUE})");
        thread::sleep(travel_time(previous, position));
        previous = position;
    }
    bus.close()?;
    Ok(())
}
