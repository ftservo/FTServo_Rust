//! Continuous 50 Hz synchronous read of one IMU plus any number of HLS servos on the same bus,
//! mirroring the upstream imu `sync_read.py`: **the first ID is the IMU**, the remaining IDs are
//! HLS servos, and one broadcast SYNC_READ serves all of them.
//!
//! A single read of 15 bytes from address 56 covers both kinds, because
//! `IMU_PRESENT_SLFP_QXL` and `HLS_PRESENT_POSITION_L` are the same address and only their
//! meaning differs after it:
//!
//! * IMU — 56/58/60 quaternion X/Y/Z (binary16, `S1E5F10`), 62/65/68 gyroscope X/Y/Z on a
//!   stride of 3 (the bytes at 64 and 67 are undefined), `QW = sqrt(1 − QX² − QY² − QZ²)`,
//!   nonnegative, 0 when the XYZ norm exceeds 1;
//! * HLS servo — 56 position, 58 speed, 60 load, 62 voltage, 63 temperature, 66 moving,
//!   69 current.
//!
//! Decoding is this example's job, not the SDK's: the block layout is a family fact, and the
//! same 15 bytes mean different things depending on which device answered. The SDK does the
//! transaction — `Bus::sync_read_decoded`, called straight from the loop — and the payload never
//! leaves its receive buffer. `Request`, `Batch` and the report `String` are allocated once
//! before the loop and reused, so from the second round on the loop allocates nothing.
//!
//! Every requested ID prints once per round: its decoded fields when it answered, otherwise
//! this round's failure. A missing device and a device fault are counted and never stop the
//! loop; press Ctrl+C to stop (the serial port is released on process exit).
//!
//! usage: cargo run --example imu-sync_read -- PORT IMU_ID [HLS_ID...]
use ftservo::{decode_sign_magnitude, registers, Batch, Feedback, Quaternion, Request, VectorRaw};
use std::{
    error::Error,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;

const INTERVAL: Duration = Duration::from_millis(20); // 50 Hz read cadence

/// Where both families' 15-byte block starts: `HLS_PRESENT_POSITION_L` for a servo,
/// `IMU_PRESENT_SLFP_QXL` for the IMU — 56 either way, which is what lets one transaction serve
/// both.
const BLOCK_ADDRESS: u8 = registers::PRESENT_POSITION;
const BLOCK_LENGTH: usize = 15;

/// One 15-byte block, kept as it came off the wire.
///
/// The decoder given to `sync_read_decoded` sees bytes, not the ID they belong to, so it cannot
/// choose between the IMU and the servo layout by itself. Carrying the block and interpreting it
/// per ID afterwards is what keeps both kinds in a single broadcast, exactly as the upstream
/// example does. `Copy`, so a `Batch` of these allocates nothing.
#[derive(Debug, Clone, Copy)]
struct Block([u8; BLOCK_LENGTH]);

impl Block {
    /// One little-endian word at `off`.
    fn word(self, off: usize) -> u16 {
        u16::from_le_bytes([self.0[off], self.0[off + 1]])
    }
    /// Quaternion X/Y/Z from the first six bytes; `W` is reconstructed nonnegative.
    fn quaternion(self) -> ftservo::Result<Quaternion> {
        Quaternion::from_raw(VectorRaw {
            x: self.word(0),
            y: self.word(2),
            z: self.word(4),
        })
    }
    /// Gyroscope at 62/65/68 — a stride of 3, over the undefined bytes at 64 and 67.
    fn gyro(self) -> VectorRaw {
        VectorRaw {
            x: self.word(6),
            y: self.word(9),
            z: self.word(12),
        }
    }
    /// The HLS feedback block. Load carries its direction bit at 10, the width the family's
    /// memory table and `Servo::read_load` use; the other signed fields use bit 15.
    fn feedback(self) -> ftservo::Result<Feedback> {
        Ok(Feedback {
            position: decode_sign_magnitude(self.word(0), 15)?,
            speed: decode_sign_magnitude(self.word(2), 15)?,
            load: decode_sign_magnitude(self.word(4), 10)?,
            current: decode_sign_magnitude(self.word(13), 15)?,
            voltage: self.0[6],
            temperature: self.0[7],
            moving: self.0[10] != 0,
        })
    }
}

/// Every reply carries the requested length, so this only guards the assumption the two
/// decoders above are written against.
fn decode_block(p: &[u8]) -> ftservo::Result<Block> {
    let block: [u8; BLOCK_LENGTH] = p
        .try_into()
        .map_err(|_| ftservo::Error::Packet("sync read block requires 15 bytes"))?;
    Ok(Block(block))
}

/// The IMU report, written into `out` (cleared first, capacity kept).
///
/// The binary16-decoded quaternion with its raw words, then each gyro axis raw and with its
/// direction bit resolved. Nothing is converted to physical units — the register values are
/// reported as they are, the same rule the rest of the SDK follows.
fn write_imu_report(out: &mut String, id: u8, b: Block) {
    use std::fmt::Write as _;
    out.clear();
    let _ = match b.quaternion() {
        Ok(q) => {
            let gyro = b.gyro();
            let signed = gyro.signed();
            write!(
                out,
                "imu {id}\n  \
                 quaternion: x={:.5} y={:.5} z={:.5} w={:.5} (w reconstructed, nonnegative)\n  \
                 quat raw  : x={} y={} z={}\n  \
                 gyro raw  : x={} y={} z={}\n  \
                 gyro valid: x={} y={} z={} (direction bit resolved)",
                q.x,
                q.y,
                q.z,
                q.w,
                q.raw.x,
                q.raw.y,
                q.raw.z,
                gyro.x,
                gyro.y,
                gyro.z,
                signed.x,
                signed.y,
                signed.z,
            )
        }
        Err(e) => write!(out, "imu {id}: undecodable quaternion block ({e})"),
    };
}

/// The HLS servo report, written into `out` (cleared first, capacity kept).
///
/// Position, speed, load and current are direction-bit resolved; voltage and temperature stay in
/// raw register units.
fn write_servo_report(out: &mut String, id: u8, b: Block) {
    use std::fmt::Write as _;
    out.clear();
    let _ = match b.feedback() {
        Ok(f) => write!(
            out,
            "servo {id}\n  \
             position   : {} (direction bit resolved)\n  \
             speed      : {} (direction bit resolved)\n  \
             load       : {} (direction bit resolved, bit 10)\n  \
             current    : {} (direction bit resolved)\n  \
             voltage    : {} (raw register units)\n  \
             temperature: {} (raw register units)\n  \
             moving     : {}",
            f.position, f.speed, f.load, f.current, f.voltage, f.temperature, f.moving
        ),
        Err(e) => write!(out, "servo {id}: undecodable feedback block ({e})"),
    };
}

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "imu-sync_read PORT IMU_ID [HLS_ID...]";
    let a: Vec<_> = std::env::args().skip(1).collect();
    if a.len() < 2 {
        return Err(common::arg_error(usage, "expected PORT and the IMU's ID"));
    }
    // The IMU goes first, exactly as `SYNC_IDS = (IMU_ID,) + SERVO_IDS` does upstream: position
    // in the list is what decides which layout each reply is decoded with.
    let imu_id = common::parse_id(&a[1]).map_err(|e| common::arg_error(usage, &e))?;
    let mut ids = vec![imu_id];
    for v in &a[2..] {
        let id = common::parse_id(v).map_err(|e| common::arg_error(usage, &e))?;
        if ids.contains(&id) {
            return Err(common::arg_error(usage, "duplicate ID"));
        }
        ids.push(id);
    }

    let bus = common::open_bus(&a[0])?;

    // Allocated once, outside the loop. Everything below is cleared and refilled in place.
    let r = Request::new();
    let mut batch = Batch::new();
    let mut line = String::new();
    // Per-round outcome, aligned with `ids`.
    let mut status: Vec<&'static str> = vec!["no data yet"; ids.len()];
    let mut round: u64 = 0;
    let mut failures: u64 = 0;
    let mut window = Instant::now();
    loop {
        round += 1;
        let started = Instant::now();
        // The `Request` is reused as it is: it only carries cancellation, and a transaction's
        // read budget is computed from the protocol, so there is nothing to re-arm.
        // One broadcast transaction for every ID; the payload is decoded straight out of the
        // receive buffer, and the results land in `batch`, which is reused across rounds.
        match bus.sync_read_decoded(
            &r,
            BLOCK_ADDRESS,
            BLOCK_LENGTH,
            &ids,
            decode_block,
            &mut batch,
        ) {
            Ok(()) => {
                failures += batch.errors.len() as u64;
                for (i, id) in ids.iter().enumerate() {
                    status[i] = if batch.get(*id).is_some() {
                        "ok"
                    } else if batch.missing.contains(id) {
                        "no response"
                    } else {
                        "device error"
                    };
                }
                // Request order, so the IMU's line comes first and then one line per servo.
                for (i, id) in ids.iter().enumerate() {
                    match batch.get(*id) {
                        Some(block) => {
                            if *id == imu_id {
                                write_imu_report(&mut line, *id, *block);
                            } else {
                                write_servo_report(&mut line, *id, *block);
                            }
                            println!("{line}");
                        }
                        None => println!(
                            "{} {id}: {}",
                            if *id == imu_id { "imu" } else { "servo" },
                            status[i]
                        ),
                    }
                }
            }
            Err(e) => {
                failures += 1;
                status.fill("read error");
                println!("sync read failed: {e}");
                for id in &ids {
                    println!(
                        "{} {id}: read error",
                        if *id == imu_id { "imu" } else { "servo" }
                    );
                }
            }
        }
        let hz = 1.0 / window.elapsed().as_secs_f64();
        println!("rounds={round} failures={failures} rate={hz:.1} Hz");
        window = Instant::now();
        common::pace(started, INTERVAL);
    }
}
