//! The addresses all three servo families agree on, plus the baud-rate codes.
//!
//! Anything a family places elsewhere lives in that family's own `registers` module — see
//! [`crate::sms_sts`], [`crate::scscl`] and [`crate::hls`] — because a shared name would be
//! wrong for at least one of them. Address 44 is the clearest case: the running time on
//! SCSCL, the open-loop PWM speed on SMS/STS, the target current on HLS.
//!
//! The IMU keeps a table of its own in [`crate::imu::registers`] rather than borrowing from
//! here: its 9, 11 and 65 mean something entirely different from a servo's.
//!
//! Word byte order is per family — see [`crate::ByteOrder`].

/// Re-exported from [`crate::protocol`], where it lives because `Device::ping` reads it.
pub use crate::protocol::MODEL;

pub const ID: u8 = 5;
pub const BAUD_RATE: u8 = 6;
pub const MIN_ANGLE: u8 = 9;
pub const MAX_ANGLE: u8 = 11;
pub const CW_DEAD: u8 = 26;
pub const CCW_DEAD: u8 = 27;
pub const TORQUE_ENABLE: u8 = 40;
pub const GOAL_POSITION: u8 = 42;
pub const GOAL_SPEED: u8 = 46;
pub const PRESENT_POSITION: u8 = 56;
pub const PRESENT_SPEED: u8 = 58;
pub const PRESENT_LOAD: u8 = 60;
pub const PRESENT_VOLTAGE: u8 = 62;
pub const PRESENT_TEMPERATURE: u8 = 63;
/// Device status: `0` healthy, otherwise a bitmask of [`crate::DeviceError`] flags.
pub const SERVO_STATUS: u8 = 65;
pub const MOVING: u8 = 66;

// Device register codes, not host serial baud rates.
pub const BAUD_1M: u8 = 0;
pub const BAUD_500K: u8 = 1;
pub const BAUD_250K: u8 = 2;
pub const BAUD_128K: u8 = 3;
pub const BAUD_115200: u8 = 4;
pub const BAUD_76800: u8 = 5;
pub const BAUD_57600: u8 = 6;
pub const BAUD_38400: u8 = 7;
