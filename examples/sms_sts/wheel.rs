//! Wheel (continuous-rotation) mode, mirroring the upstream sms_sts
//! `wheel.py`: switch mode explicitly, run, stop, then restore position mode.
//! SMS/STS wheel speed requires torque = 0.
//! usage: cargo run --example sms_sts-wheel -- PORT ID SPEED [ACCELERATION]
use ftservo::Servo;
use std::{error::Error, thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "sms_sts-wheel PORT ID SPEED [ACCELERATION]";
    let a: Vec<_> = std::env::args().skip(1).collect();
    if a.len() != 3 && a.len() != 4 {
        return Err(common::arg_error(usage, "expected 3 or 4 arguments"));
    }
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;
    let speed: i32 = a[2]
        .parse()
        .map_err(|_| common::arg_error(usage, "SPEED must be a signed integer"))?;
    let acceleration: u8 = if a.len() == 4 {
        a[3].parse()
            .map_err(|_| common::arg_error(usage, "ACCELERATION must be 0..=255"))?
    } else {
        0
    };

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::sms_sts(bus.clone());
    let r = common::request()?;
    servo.wheel_mode(&r, id)?;
    println!("wheel mode set; running at speed {speed}");
    servo.write_speed(&r, id, speed, acceleration, 0)?;
    thread::sleep(Duration::from_secs(2));
    servo.write_speed(&r, id, 0, acceleration, 0)?;
    println!("stopped; restoring position mode");
    servo.position_mode(&r, id)?;
    bus.close()?;
    Ok(())
}
