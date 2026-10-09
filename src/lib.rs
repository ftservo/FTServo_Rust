//! Native FEETECH bus SDK. Clone one [`Bus`] to share a physical port.
//! Values remain in device register units unless explicitly documented.
//! See `README.md` for the usage guide.
//!
//! # Layout
//!
//! Three layers, with dependencies pointing one way — a layer may use the ones below it,
//! never the other way round. `error` sits underneath all three.
//!
//! **1. Serial link** — the physical port and the bytes on it: `bus` (port ownership,
//! transactions, framing, batching) and `serial` (the `serialport` backend, which keeps its
//! own file so the `serial` feature can drop it whole).
//!
//! **2. Wire protocol** — what a packet means: `protocol` (packet layout, instruction codes,
//! field codecs, register access).
//!
//! **3. Servo and IMU** — one module per device family: `servo` (with `sms_sts` / `scscl` /
//! `hls` inlined), `imu`, and `registers` for the addresses all three servo families agree on.
#![forbid(unsafe_code)]

// 1. Serial link.
mod bus;
#[cfg(feature = "serial")]
mod serial;

// 2. Wire protocol.
pub mod protocol;

// 3. Servo and IMU.
pub mod imu;
pub mod registers;
mod servo;

// Underneath all three: the error types every layer returns. Depends on nothing.
mod error;

pub use bus::{
    Batch, Bus, ReadDelayGuard, Request, Transport, WriteEntry, DEFAULT_READ_DELAY,
    EEPROM_READ_DELAY,
};
pub use error::{DeviceError, Error, Result};
pub use imu::{Imu, Quaternion, Vector, VectorRaw};
pub use protocol::{
    decode_sign_magnitude, encode_sign_magnitude, half_float, ByteOrder, Device, Status,
    BROADCAST_ID, MAX_ID, MAX_PACKET_SIZE,
};
pub use servo::{hls, scscl, sms_sts, Family, Feedback, Motion, Servo};
