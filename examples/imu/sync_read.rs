//! Continuous 50 Hz synchronous read of the fused IMU block (quaternion +
//! gyro) from several bus IMUs, mirroring the upstream imu `sync_read.py` and
//! its `while 1` loop. Missing devices and device faults are counted and never
//! stop the loop; press Ctrl+C to stop (the serial port is released on process
//! exit).
//!
//! Per round every ID except the last prints only its read outcome
//! (ok / no response / device error / read error); the last ID prints the
//! decoded quaternion and gyroscope of its block.
//!
//! Reading and decoding the block is this example's job, not the SDK's: the transaction is
//! `Bus::sync_read_decoded`, called straight from the loop for the 15 bytes starting at
//! `QUATERNION_X`, and the payload becomes the `Sample` that `decode_feedback` below builds.
//!
//! 15 is where IMU.MD's 内存表 puts the accelerometer. Its SRAM 反馈 table runs
//! 四元数 QX/QY/QZ at 0x38/0x3A/0x3C (56/58/60), then 陀螺仪 WX at 0x3E (62), with one
//! 无定义 byte after each gyro axis at 0x40/0x43/0x46 (64/67/70), and 加速计 AX begins at
//! 0x47 (71) — exactly 56 + 15. So the accelerometer is not in this read; `imu-read_acc`
//! covers that register on its own.
//!
//! Everything the loop touches is allocated **once, before the loop**: the `Request`, the
//! `Batch` the results land in, and the `String` the report is written into. Each round
//! clears them and keeps the capacity, so from the second round on the loop allocates
//! nothing — no small allocations left sitting on the serial I/O path.
//!
//! usage: cargo run --example imu-sync_read -- PORT ID1 [ID2...]
use ftservo::{imu::registers::QUATERNION_X, Batch, Quaternion, Request, VectorRaw};
use std::{
    collections::BTreeMap,
    error::Error,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;

const INTERVAL: Duration = Duration::from_millis(20); // 50 Hz read cadence

/// The block, as IMU.MD's SRAM 反馈 table says the bytes are.
///
/// The quaternion is three half floats (IMU.MD: 半精度浮点 `S1E5F10`, i.e. binary16) at
/// 0x38/0x3A/0x3C. `QW` is not stored at all — IMU.MD defines it as `sqrt(1 − QX² − QY² − QZ²)`,
/// with the 备注 "先归一化…或直接把 QW 置 0" when the XYZ norm exceeds 1 — which is exactly what
/// `Quaternion::from_raw` recovers.
///
/// The gyroscope is three signed 16-bit words whose BIT15 is a direction bit (IMU.MD:
/// 有符号16位整数，BIT15 为方向位) at 0x3E/0x41/0x44, so the axes are strided with one 无定义
/// byte between them rather than packed. `Copy`, so the batch and the per-round map hold it
/// without allocating.
#[derive(Debug, Clone, Copy)]
struct Sample {
    quaternion: Quaternion,
    gyro: VectorRaw,
}

/// Decode the block the loop asked for: little-endian words, the quaternion on a stride of 2
/// bytes and the gyroscope on a stride of 3 from `GYRO_X` (0x3E → 0x41 → 0x44, over the
/// 无定义 bytes at 0x40 and 0x43).
///
/// Spelled out here rather than taken from the SDK because the block's extent is this
/// example's choice, and a decoder has to agree with the length that was requested.
fn decode_feedback(p: &[u8]) -> ftservo::Result<Sample> {
    let bytes: [u8; 15] = p
        .try_into()
        .map_err(|_| ftservo::Error::Packet("IMU block requires 15 bytes"))?;
    let word = |off: usize| u16::from_le_bytes([bytes[off], bytes[off + 1]]);
    Ok(Sample {
        quaternion: Quaternion::from_raw(VectorRaw {
            x: word(0),
            y: word(2),
            z: word(4),
        })?,
        gyro: VectorRaw {
            x: word(6),
            y: word(9),
            z: word(12),
        },
    })
}

/// Report of one decoded sample, written into `out` (cleared first, capacity kept).
///
/// Local to this example because it is the only one that prints a whole block: the
/// single-field examples (`read_acc`, `read_gyro`, `read_slfp`) format their two lines
/// directly, and `common/mod.rs` takes nothing device-shaped.
///
/// The binary16-decoded quaternion (X/Y/Z plus the nonnegative reconstructed W) with its raw
/// words, then the raw and `.signed()` values of each gyro axis. `.signed()` only resolves the
/// direction bit; nothing is converted to physical units. `out` is cleared and refilled in
/// place, so the 50 Hz loop reuses a single `String` rather than allocating a fresh one every
/// round, the same reason it reuses its `Batch`.
fn write_sample_report(out: &mut String, id: u8, s: &Sample) {
    use std::fmt::Write as _;
    out.clear();
    let q = &s.quaternion;
    let gyro = s.gyro.signed();
    let _ = write!(
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
        s.gyro.x,
        s.gyro.y,
        s.gyro.z,
        gyro.x,
        gyro.y,
        gyro.z,
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let usage = "imu-sync_read PORT ID1 [ID2...]";
    let a: Vec<_> = std::env::args().skip(1).collect();
    if a.len() < 2 {
        return Err(common::arg_error(
            usage,
            "expected at least PORT and one ID",
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

    // Allocated once, outside the loop. Everything below is cleared and refilled in place.
    let r = Request::new();
    let mut batch = Batch::new();
    let mut line = String::new();
    let mut last: BTreeMap<u8, Sample> = BTreeMap::new();
    // Per-round outcome, aligned with `ids`.
    let mut status: Vec<&'static str> = vec!["no data yet"; ids.len()];
    let mut round: u64 = 0;
    let mut failures: u64 = 0;
    let mut window = Instant::now();
    loop {
        round += 1;
        let started = Instant::now();
        // The `Request` is reused as it is: its only job is cancellation, and a transaction's
        // read budget is computed from the protocol, so there is nothing to re-arm.
        // One broadcast transaction for every ID; the payload is decoded straight out of the
        // receive buffer, and the results land in `batch`, which is reused across rounds.
        match bus.sync_read_decoded(&r, QUATERNION_X, 15, &ids, decode_feedback, &mut batch) {
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
                for (id, sample) in &batch.values {
                    last.insert(*id, *sample);
                }
            }
            Err(e) => {
                failures += 1;
                status.fill("read error");
                println!("sync read failed: {e}");
            }
        }
        // Only the last requested ID prints its sample; the others show just
        // whether this round succeeded.
        let last_index = ids.len() - 1;
        for (i, id) in ids.iter().enumerate() {
            if i == last_index {
                match last.get(id) {
                    Some(sample) => {
                        write_sample_report(&mut line, *id, sample);
                        println!("{line}");
                    }
                    None => println!("imu {id}: no sample yet ({})", status[i]),
                }
            } else {
                println!("imu {id}: {}", status[i]);
            }
        }
        let hz = 1.0 / window.elapsed().as_secs_f64();
        println!("rounds={round} failures={failures} rate={hz:.1} Hz");
        window = Instant::now();
        common::pace(started, INTERVAL);
    }
}
