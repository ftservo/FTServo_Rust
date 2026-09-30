//! Connectivity, model and firmware probe for all device types.
//! Mirrors the upstream sms_sts/scscl/hls/imu `ping.py` examples.
//!
//! Without an ID, every valid unicast ID (0..=252; 253 is reserved and 254
//! is broadcast) is probed with the bare PING instruction. Any device that
//! answers is followed by one READ of the five read-only EPROM bytes at
//! addresses 0..=4 (firmware version, one reserved byte, model number). The
//! bytes are shown per address, never composed into one integer, so no byte
//! order is assumed. Wire frames are printed in hex for comparison with
//! docs/PROTOCOL.md:
//! TX `FF FF ID LEN INST [PARAMS] CHKSUM`, RX `FF FF ID LEN FLAGS [DATA] CHKSUM`.
//! RX hex is rebuilt from the decoded reply; the bytes match the wire, but
//! the SDK does not expose the raw receive buffer.
//!
//! usage: cargo run --example ping -- PORT
//!        cargo run --example ping -- PORT ID [ID...]
use ftservo::{
    protocol::{self, encode_packet},
    Error, Request,
};
use std::{
    error::Error as ErrorTrait,
    io::{self, Write},
    time::Duration,
};

#[path = "common/mod.rs"]
mod common;

/// First address of the read-only EPROM block: firmware version (0/1),
/// one reserved byte (2), model number (3/4).
const BLOCK_ADDRESS: u8 = 0;
const BLOCK_LENGTH: u8 = 5;
const BLOCK_LABELS: [&str; BLOCK_LENGTH as usize] = [
    "firmware version",
    "firmware version",
    "reserved/unknown (check device manual)",
    "model number, low byte",
    "model number, high byte",
];

fn hex(p: &[u8]) -> String {
    p.iter()
        .map(|v| format!("{v:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Rebuild a status frame from decoded fields; checksum per protocol:
/// `~(ID + LEN + FLAGS + DATA) & 0xFF` where LEN counts FLAGS + DATA.
fn status_frame(id: u8, flags: u8, data: &[u8]) -> Vec<u8> {
    let len = 2 + data.len() as u16;
    let mut sum = u16::from(id) + len + u16::from(flags);
    sum += data.iter().map(|&v| u16::from(v)).sum::<u16>();
    let mut p = vec![255, 255, id, len as u8, flags];
    p.extend_from_slice(data);
    p.push((!sum & 0xFF) as u8);
    p
}

/// One READ of the five EPROM bytes at addresses 0..=4, then a per-address
/// interpretation. The bytes are never composed into one integer, so no
/// byte order is assumed.
fn read_block(bus: &ftservo::Bus, r: &Request, id: u8) -> Result<(), Box<dyn ErrorTrait>> {
    let tx = encode_packet(id, protocol::READ, &[BLOCK_ADDRESS, BLOCK_LENGTH])?;
    println!("TX: {}", hex(&tx));
    match bus.read(r, id, BLOCK_ADDRESS, usize::from(BLOCK_LENGTH)) {
        Ok(data) => {
            println!("RX: {}", hex(&status_frame(id, 0, &data)));
            println!("raw: {}", hex(&data));
            for (i, v) in data.iter().enumerate() {
                println!(
                    "  addr {} = 0x{v:02X}  {}",
                    BLOCK_ADDRESS + i as u8,
                    BLOCK_LABELS[i]
                );
            }
            Ok(())
        }
        Err(Error::Device(e)) => {
            println!("RX: {}", hex(&status_frame(e.id, e.flags, &[])));
            Err(e.into())
        }
        Err(e) => Err(e.into()),
    }
}

/// One unicast probe: PING, then the automatic EPROM block read.
fn probe(bus: &ftservo::Bus, r: &Request, id: u8) -> Result<(), Box<dyn ErrorTrait>> {
    let tx = encode_packet(id, protocol::PING, &[])?;
    println!("TX: {}", hex(&tx));
    match bus.ping(r, id) {
        Ok(()) => println!("RX: {}", hex(&status_frame(id, 0, &[]))),
        Err(Error::Device(e)) => {
            println!("RX: {}", hex(&status_frame(e.id, e.flags, &[])));
            return Err(e.into());
        }
        Err(e) => return Err(e.into()),
    }
    println!("device {id} answered");
    read_block(bus, r, id)
}

fn main() -> Result<(), Box<dyn ErrorTrait>> {
    let usage = "ping PORT [ID...]";
    let a: Vec<_> = std::env::args().skip(1).collect();
    if a.is_empty() {
        return Err(common::arg_error(usage, "expected PORT and optional IDs"));
    }

    // Full scan. The per-ID response window must cover the adapter's
    // half-duplex turnaround time; 50 ms per ID keeps the sweep reliable
    // (an empty bus takes about 13 s).
    if a.len() == 1 {
        let bus = ftservo::Bus::open(&a[0], 1_000_000, Duration::from_millis(50))?;
        let r = Request::with_timeout(Duration::from_secs(60))?;
        let total = u16::from(ftservo::MAX_ID);
        let progress_width = format!("scanning id {total}/{total} ...").len();
        let mut found = 0;
        for id in 0..=ftservo::MAX_ID {
            print!("\rscanning id {id:>3}/{total} ...");
            io::stdout().flush()?;
            let hit = match bus.ping(&r, id) {
                Ok(()) => Some(0u8),
                Err(Error::Device(e)) => Some(e.flags),
                Err(_) => None,
            };
            if let Some(flags) = hit {
                let tx = encode_packet(id, protocol::PING, &[])?;
                let rx = status_frame(id, flags, &[]);
                let summary = if flags == 0 {
                    format!("device {id} answered")
                } else {
                    format!("device {id} answered with status 0x{flags:02X}")
                };
                // Pad over the progress line so no stale characters remain.
                let pad = " ".repeat(progress_width.saturating_sub(summary.len()));
                println!("\r{summary}{pad}");
                println!("TX: {}", hex(&tx));
                println!("RX: {}", hex(&rx));
                found += 1;
                // Block read failures do not abort the scan.
                if let Err(e) = read_block(&bus, &r, id) {
                    println!("eprom read failed: {e}");
                }
            }
        }
        // Clear the progress line, then summarize.
        print!("\r{}\r", " ".repeat(progress_width));
        if found == 0 {
            println!("no device answered");
        } else {
            println!("scan complete: {found} device(s) found");
        }
        io::stdout().flush()?;
        bus.close()?;
        return Ok(());
    }

    // Probe the listed IDs (one or more) with the automatic EPROM block
    // read; a failing probe is reported and the remaining IDs continue.
    let mut ids = Vec::new();
    for v in &a[1..] {
        let id = common::parse_id(v).map_err(|e| common::arg_error(usage, &e))?;
        if !ids.contains(&id) {
            ids.push(id);
        }
    }

    let bus = common::open_bus(&a[0])?;
    let r = common::request()?;
    for id in ids {
        if let Err(e) = probe(&bus, &r, id) {
            println!("probe failed: {e}");
        }
    }
    bus.close()?;
    Ok(())
}
