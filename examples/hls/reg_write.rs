//! Registered write (RegWrite + Action), mirroring the upstream hls
//! `reg_write.py`: the motion starts only when Action is issued.
//! usage: cargo run --example hls-reg_write -- PORT ID POSITION SPEED ACCELERATION TORQUE
use ftservo::{Motion, Servo};
use std::error::Error;

#[path = "../common/mod.rs"]
mod common;

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "hls-reg_write PORT ID POSITION SPEED ACCELERATION TORQUE";
    let a = common::args(usage, 6)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;
    let position: i32 = a[2]
        .parse()
        .map_err(|_| common::arg_error(usage, "POSITION must be an integer"))?;
    let speed: u16 = a[3]
        .parse()
        .map_err(|_| common::arg_error(usage, "SPEED must be an integer"))?;
    let acceleration: u8 = a[4]
        .parse()
        .map_err(|_| common::arg_error(usage, "ACCELERATION must be 0..=255"))?;
    let torque: u16 = a[5]
        .parse()
        .map_err(|_| common::arg_error(usage, "TORQUE must be an integer"))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::hls(bus.clone());
    let r = common::request()?;
    let motion = Motion {
        id,
        position,
        speed,
        acceleration,
        torque,
        ..Motion::default()
    };
    servo.reg_write_position(&r, motion)?;
    println!("registered; sending Action to start the motion");
    servo.action(&r, id)?;
    bus.close()?;
    Ok(())
}
