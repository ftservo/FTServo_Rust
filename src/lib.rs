//! Native FEETECH bus SDK. Clone one [`Bus`] to share a physical port.
//! Values remain in device register units unless explicitly documented.
//! See the Chinese usage guide bundled in `docs/USAGE.md`.
#![forbid(unsafe_code)]

mod bus;
mod device;
mod encoding;
mod error;
mod imu;
pub mod protocol;
pub mod registers;
#[cfg(feature = "serial")]
mod serial;
mod servo;

pub use bus::{Batch, Bus, Request, Transport, WriteEntry};
pub use device::{ByteOrder, Device};
pub use encoding::{decode_sign_magnitude, encode_sign_magnitude, half_float};
pub use error::{DeviceError, Error, Result};
pub use imu::{decode_imu_sample, Imu, ImuSample, Quaternion, Vector, VectorRaw};
pub use protocol::{Status, BROADCAST_ID, MAX_ID, MAX_PACKET_SIZE};
pub use servo::{Family, Feedback, Motion, Servo};

/// Chinese usage guide, whose Rust snippets are compiled by cargo test.
#[doc = include_str!("../docs/USAGE.md")]
pub mod guide {}
