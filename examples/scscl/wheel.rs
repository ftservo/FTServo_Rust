//! Continuous rotation for SCSCL via PWM mode, mirroring the upstream
//! scscl `wheel.py` (PWM control instead of a wheel-mode register).
//! usage: cargo run --example scscl-wheel -- PORT ID PWM
use ftservo::{Servo, EEPROM_READ_DELAY};
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
    // `MIN_ANGLE` is an EEPROM register on SCSCL, and switching PWM mode is a write to it:
    // the device spends an erase cycle and stays off the bus meanwhile. The guard raises
    // the read delay for the write and restores it on the way out.
    {
        let _slow = bus.with_read_delay(EEPROM_READ_DELAY);
        servo.pwm_mode(&r, id)?;
    }
    println!("PWM mode set; driving {pwm}");
    servo.write_pwm(&r, id, pwm)?;
    thread::sleep(Duration::from_secs(2));
    servo.write_pwm(&r, id, 0)?;
    println!("stopped");
    bus.close()?;
    Ok(())
}
