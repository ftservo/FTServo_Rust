//! Continuous rotation for SCSCL via PWM mode, mirroring the upstream
//! scscl `wheel.py` (PWM control instead of a wheel-mode register).
//! usage: cargo run --example scscl-wheel -- PORT ID PWM
use ftservo::Servo;
use std::{error::Error, thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "scscl-wheel PORT ID PWM";
    let a = common::args(usage, 3)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;
    let pwm: i32 = a[2]
        .parse()
        .map_err(|_| common::arg_error(usage, "PWM must be a signed integer"))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::scscl(bus.clone());
    let r = common::request()?;
    servo.pwm_mode(&r, id)?;
    println!("PWM mode set; driving {pwm}");
    servo.write_pwm(&r, id, pwm)?;
    thread::sleep(Duration::from_secs(2));
    servo.write_pwm(&r, id, 0)?;
    println!("stopped");
    bus.close()?;
    Ok(())
}
