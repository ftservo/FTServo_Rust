//! HLS state/turn-count reset, mirroring the upstream hls `reset.py`.
//! This is the upstream RESET instruction clearing accumulated state
//! (e.g. multi-turn count); it is not a factory reset of EPROM settings.
//!
//! `reset` is a `Device` instruction — `ping`'s neighbour — rather than a
//! servo-family one, so any handle on the bus can send it. The servo handle
//! below is simply the natural way to address a servo.
//!
//! usage: cargo run --example hls-reset -- PORT ID
use ftservo::Servo;
use std::error::Error;

#[path = "../common/mod.rs"]
mod common;

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "hls-reset PORT ID";
    let a = common::args(usage, 2)?;
    let id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;

    let bus = common::open_bus(&a[0])?;
    let servo = Servo::hls(bus.clone());
    let r = common::request()?;
    servo.reset(&r, id)?;
    println!("state/turn-count reset sent to servo {id}");
    bus.close()?;
    Ok(())
}
