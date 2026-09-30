//! Offset calibration for HLS servos, mirroring the upstream hls `ofscal.py`:
//! write the current position as the new zero reference at the given raw
//! position value. A normal, upstream-documented calibration procedure.
//! usage: cargo run --example hls-ofscal -- PORT ID POSITION
use ftservo::Servo;
use std::error::Error;

#[path = "../common/mod.rs"]
mod common;

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "hls-ofscal PORT ID POSITION";
    let a = common::args(usage, 3)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;
    let position: u16 = a[2]
        .parse()
        .map_err(|_| common::arg_error(usage, "POSITION must be 0..=65535"))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::hls(bus.clone());
    let r = common::request()?;
    servo.calibrate_offset(&r, id, position)?;
    println!("offset calibrated at position {position}");
    bus.close()?;
    Ok(())
}
