//! Synchronous write: move several SCSCL servos simultaneously in one
//! broadcast transaction, mirroring the upstream scscl `sync_write.py`.
//! No per-device acknowledgment is sent for Sync Write.
//! usage: cargo run --example scscl-sync_write -- PORT ID1 ID2 [ID...]
use ftservo::{Motion, Servo};
use std::{error::Error, thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;

const POSITIONS: [i32; 2] = [1000, 20];
const SPEED: u16 = 1500;
const STROKES: u32 = 3;

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "scscl-sync_write PORT ID1 ID2 [ID...]";
    let a: Vec<_> = std::env::args().skip(1).collect();
    if a.len() < 3 {
        return Err(common::arg_error(
            usage,
            "expected at least PORT and two IDs",
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
    let servo = Servo::scscl(bus.clone());
    let r = common::request()?;
    for stroke in 0..STROKES {
        let position = if stroke % 2 == 0 {
            POSITIONS[0]
        } else {
            POSITIONS[1]
        };
        let motions: Vec<_> = ids
            .iter()
            .map(|&id| Motion {
                id,
                position,
                speed: SPEED,
                ..Motion::default()
            })
            .collect();
        servo.sync_write_position(&r, &motions)?;
        println!("sync-wrote position {position} to {ids:?}");
        thread::sleep(Duration::from_secs(2));
    }
    bus.close()?;
    Ok(())
}
