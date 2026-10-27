**English** | [简体中文](README.zh-CN.md)

# ftservo · FEETECH bus servos and IMU, in Rust

A native Rust implementation of the FEETECH bus SDK. It covers the **SMS/STS, SCSCL and HLS / HTS-compatible servo families** and the **bus IMU** of the official Python SDK: plain reads and writes, synchronous reads and writes, REG_WRITE/ACTION, individual feedback fields, the RESET instruction, EPROM lock, HLS offset calibration, and IMU quaternion / gyroscope / acceleration.

## Supported families

| Family | Byte order | Covered | Limits |
| --- | --- | --- | --- |
| SMS/STS | little-endian words | position/speed writes (with the acceleration field), sync read/write, wheel mode | torque mode does not apply; `write_speed` requires a zero torque field |
| SCSCL | **big-endian** words | position writes, PWM mode and PWM output, sync write | no mode register, **no sync read**; positions are unsigned |
| HLS / HTS-compatible | little-endian words | position/speed writes (with acceleration and torque), sync read/write, wheel mode, offset calibration | — |
| Bus IMU | little-endian words | quaternion, gyroscope, acceleration | quaternion XYZ are binary16 (half float, S1E5F10); W is rebuilt from the nonnegative root |

## Quick start

Build and test in this directory (`serial` is on by default):

```sh
cargo build --locked
cargo test --locked
```

Depend on it by path from an application's `Cargo.toml` (this crate is not published on crates.io):

```toml
[dependencies]
ftservo = { path = "../FTServo_Rust" }

# Custom Transport only, without pulling in serialport:
# ftservo = { path = "../FTServo_Rust", default-features = false }
```

Minimal example:

```rust
use ftservo::{decode_sign_magnitude, registers, Batch, Bus, Imu, Motion, Request, Servo};
use std::time::Duration;

fn main() -> ftservo::Result<()> {
    // One Bus per physical port; clones share the port and the transaction lock instead of
    // opening it twice. The third argument is the **write** budget — read time limits are
    // computed from the protocol, see Bus::command.
    let bus = Bus::open("COM3", 1_000_000, Duration::from_millis(100))?;
    let servo = Servo::sms_sts(bus.clone());
    let imu = Imu::new(bus.clone());
    // A Request carries cancellation only, no timeout; a read's time limit comes from the protocol.
    let request = Request::new();

    // Write a target position: the SDK does not enable torque on its own, so the device must
    // already be in the right mode and allowed to drive.
    servo.write_position(&request, Motion::new(1, 2048))?;

    // Reading several devices at once: Bus::sync_read_decoded runs the transaction, while
    // decoding the 15-byte feedback block is the application's job — the layout (byte order,
    // direction bits, field offsets) is a family fact the library does not guess. Only two
    // bytes of position are taken here as an illustration; the full decode lives in
    // examples/hls/sync_read.rs and examples/sms_sts/sync_read.rs, plus
    // examples/imu/sync_read.rs (its first ID is the IMU, the rest are HLS servos on the same
    // bus, all answering from address 56).
    let mut batch = Batch::new();
    bus.sync_read_decoded(
        &request,
        registers::PRESENT_POSITION,
        15,
        &[1, 2],
        |p| decode_sign_magnitude(u16::from_le_bytes([p[0], p[1]]), 15),
        &mut batch,
    )?;
    for (id, position) in &batch.values {
        println!("servo {id}: position={position}");
    }
    if !batch.is_ok() {
        println!("errors={:?} missing={:?}", batch.errors, batch.missing);
    }

    // IMU: the quaternion arrives as half floats with W rebuilt from the nonnegative root;
    // the gyroscope and the accelerometer are read one at a time.
    let quaternion = imu.read_quaternion(&request, 3)?;
    println!("quaternion: {quaternion:?}");
    let gyro = imu.read_gyro(&request, 3)?;
    println!("gyro raw: {gyro:?} signed: {:?}", gyro.signed());

    bus.close()?;
    Ok(())
}
```

## Examples

```sh
# Connectivity probe: with no ID it scans 0..=253 and then reads firmware version / model
# (addresses 0..=4) from every device that answered
cargo run --example ping -- COM3
cargo run --example ping -- COM3 1            # a single ID
cargo run --example ping -- COM3 0 1 2 3      # several IDs

# SMS/STS
cargo run --example sms_sts-read -- COM3 1
cargo run --example sms_sts-write -- COM3 1
cargo run --example sms_sts-reg_write -- COM3 1 2048 300 20
cargo run --example sms_sts-sync_read -- COM3 1 2
cargo run --example sms_sts-sync_write -- COM3 1 2
cargo run --example sms_sts-wheel -- COM3 1 300 20

# SCSCL (big-endian; wheel uses PWM, there is no sync read)
cargo run --example scscl-read -- COM3 1
cargo run --example scscl-write -- COM3 1
cargo run --example scscl-reg_write -- COM3 1 500 1500
cargo run --example scscl-sync_write -- COM3 1 2
cargo run --example scscl-wheel -- COM3 1 1000

# HLS (the motion frame carries a torque field; ofscal is offset calibration; reset is the
# device-level RESET instruction, which on HLS clears state / turn count rather than doing a
# factory reset)
cargo run --example hls-read -- COM3 1
cargo run --example hls-write -- COM3 1
cargo run --example hls-reg_write -- COM3 1 2048 300 20 500
cargo run --example hls-sync_read -- COM3 1 2
cargo run --example hls-sync_write -- COM3 1 2
cargo run --example hls-wheel -- COM3 1 300 20
cargo run --example hls-ofscal -- COM3 1 2048
cargo run --example hls-reset -- COM3 1

# IMU
cargo run --example imu-read_acc -- COM3 2
cargo run --example imu-read_gyro -- COM3 2
cargo run --example imu-read_slfp -- COM3 2
cargo run --example imu-sync_read -- COM3 200 10 11   # first ID is the IMU, the rest are HLS servos on the same bus
```

## Architecture

| Module | Responsibility |
| --- | --- |
| `src/protocol.rs` | Packet encoding, status frames, ID / packet-length / register-range validation, field codecs (direction bit, half float), `Device` (family byte order, raw register access, write-response policy, EPROM lock, ping/reset) |
| `src/bus.rs` | `Transport` abstraction, shared `Bus`, whole-transaction lock, `Request` cancellation, partial `Batch` results |
| `src/servo.rs` | Family spec tables, motion parameters and validation, control for the three servo families, single-field feedback reads |
| `src/imu.rs` | Strided layout, quaternion (binary16 conversion, W rebuild), gyroscope and accelerometer reads |
| `src/serial.rs` | Optional `serialport` adapter (platform logic kept out of the protocol layer) |
| `src/error.rs` / `src/registers.rs` | Structured errors and register tables with provenance |
| `tests/sdk.rs` | Hardware-free regression tests over a mock transport |
