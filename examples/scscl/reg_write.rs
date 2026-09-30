//! Registered write (RegWrite + Action), mirroring the upstream scscl
//! `reg_write.py`: the motion starts only when Action is issued.
//! SCSCL Motion accepts position/time/speed only.
//! usage: cargo run --example scscl-reg_write -- PORT ID POSITION SPEED
use ftservo::{Motion, Servo};
use std::error::Error;

#[path = "../common/mod.rs"]
mod common;

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "scscl-reg_write PORT ID POSITION SPEED";
    let a = common::args(usage, 4)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;
    let position: i32 = a[2]
        .parse()
        .map_err(|_| common::arg_error(usage, "POSITION must be an integer"))?;
    let speed: u16 = a[3]
        .parse()
        .map_err(|_| common::arg_error(usage, "SPEED must be an integer"))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::scscl(bus.clone());
    let r = common::request()?;
    let motion = Motion {
        id,
        position,
        speed,
        ..Motion::default()
    };
    servo.reg_write_position(&r, motion)?;
    println!("registered; sending Action to start the motion");
    servo.action(&r, id)?;
    bus.close()?;
    Ok(())
}
