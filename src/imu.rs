//! The IMU: fused orientation, gyroscope and accelerometer, over the same [`Bus`].
//!
//! Addresses live in this module's own [`registers`] table, taken straight from the official
//! memory map, rather than shared with the servo families.

use self::registers::{ACC_X, GYRO_X, LOCK, QUATERNION_X};
use crate::{half_float, protocol::signed15, Bus, ByteOrder, Device, Error, Request, Result};
use std::ops::Deref;

/// The IMU's register table, as published in the official memory map.
///
/// Deliberately self-contained. The IMU is not a servo, and the addresses that look shared
/// are not: its 9 is an LED alarm mask rather than a minimum angle, its 11 is the gyroscope
/// range rather than a maximum angle, and its 65 is the second gyroscope axis rather than a
/// status byte. Borrowing from [`crate::registers`] here would apply servo meanings by
/// accident.
///
/// Everything multi-byte is little-endian, and the quaternion's `W` is not stored at all —
/// it is recovered arithmetically, see [`super::Quaternion`].
pub mod registers {
    /// Gyroscope full-scale range: `0` = 4000 dps, `1` = 2000, `2` = 1000.
    pub const GYRO_DPS: u8 = 11;
    /// Accelerometer full-scale range: `0` = 16G, `1` = 8G, `2` = 4G, `3` = 2G.
    pub const ACC_SCALE: u8 = 12;
    /// SFLP sample rate: `0` = 480, `1` = 240, `2` = 120.
    pub const SFLP_ODR: u8 = 13;
    /// EEPROM unlock register.
    ///
    /// **Not in the official memory map** — the IMU's configuration block spans 5 to 13 and
    /// contains no lock register at all. The value is kept so [`super::Imu::new`] behaves
    /// exactly as it always has; a write here has no documented effect on real hardware.
    pub const LOCK: u8 = 55;
    /// Quaternion X, half float.
    pub const QUATERNION_X: u8 = 56;
    /// Quaternion Y, half float.
    pub const QUATERNION_Y: u8 = 58;
    /// Quaternion Z, half float.
    pub const QUATERNION_Z: u8 = 60;
    /// Gyroscope X, a signed 16-bit value whose bit 15 is a direction bit.
    pub const GYRO_X: u8 = 62;
    /// Gyroscope Y. The map leaves one undefined byte on either side of it, at 64 and 67.
    pub const GYRO_Y: u8 = 65;
    /// Gyroscope Z.
    pub const GYRO_Z: u8 = 68;
    /// Accelerometer X, a signed 16-bit value whose bit 15 is a direction bit.
    pub const ACC_X: u8 = 71;
    /// Accelerometer Y.
    pub const ACC_Y: u8 = 74;
    /// Accelerometer Z.
    pub const ACC_Z: u8 = 77;
    /// IMU status: `0` normal, `1` abnormal. Bit 1 means `COMM_ERR`, the link is down.
    ///
    /// The IMU's counterpart of a servo's [`SERVO_STATUS`](crate::registers::SERVO_STATUS),
    /// but at its own address and with its own bits.
    pub const STATUS: u8 = 79;
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VectorRaw {
    pub x: u16,
    pub y: u16,
    pub z: u16,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vector {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}
impl VectorRaw {
    /// Direction-bit decoding only; units remain raw register units.
    pub fn signed(self) -> Vector {
        Vector {
            x: signed15(self.x),
            y: signed15(self.y),
            z: signed15(self.z),
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
    pub raw: VectorRaw,
}
impl Quaternion {
    /// Nonnegative W; if XYZ norm exceeds 1, W=0 and XYZ is preserved.
    pub fn from_raw(raw: VectorRaw) -> Result<Self> {
        let (x, y, z) = (half_float(raw.x), half_float(raw.y), half_float(raw.z));
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(Error::Packet("non-finite quaternion"));
        }
        Ok(Self {
            x,
            y,
            z,
            w: (1.0 - x * x - y * y - z * z).max(0.0).sqrt(),
            raw,
        })
    }
}
fn vector(p: &[u8], stride: usize) -> VectorRaw {
    let word = |off| u16::from_le_bytes([p[off], p[off + 1]]);
    VectorRaw {
        x: word(0),
        y: word(stride),
        z: word(2 * stride),
    }
}
#[derive(Clone)]
pub struct Imu {
    device: Device,
}
impl Deref for Imu {
    type Target = Device;
    fn deref(&self) -> &Device {
        &self.device
    }
}
impl Imu {
    pub fn new(bus: Bus) -> Self {
        Self {
            device: Device::new(bus, ByteOrder::Little, LOCK),
        }
    }
    pub fn without_write_response(mut self) -> Self {
        self.device.reply = false;
        self
    }
    pub fn read_quaternion_raw(&self, r: &Request, id: u8) -> Result<VectorRaw> {
        Ok(vector(&self.read(r, id, QUATERNION_X, 6)?, 2))
    }
    pub fn read_quaternion(&self, r: &Request, id: u8) -> Result<Quaternion> {
        Quaternion::from_raw(self.read_quaternion_raw(r, id)?)
    }
    pub fn read_gyro(&self, r: &Request, id: u8) -> Result<VectorRaw> {
        Ok(vector(&self.read(r, id, GYRO_X, 8)?, 3))
    }
    pub fn read_acceleration(&self, r: &Request, id: u8) -> Result<VectorRaw> {
        Ok(vector(&self.read(r, id, ACC_X, 8)?, 3))
    }
}

#[cfg(test)]
mod tests {
    use super::registers as reg;

    /// The IMU's addresses against the official memory map. They are deliberately not shared
    /// with the servo tables — 9, 11 and 65 mean something else there — so each one is
    /// pinned here rather than inherited.
    #[test]
    fn registers_match_the_memory_map() {
        assert_eq!(reg::GYRO_DPS, 11);
        assert_eq!(reg::ACC_SCALE, 12);
        assert_eq!(reg::SFLP_ODR, 13);
        assert_eq!(reg::LOCK, 55);
        assert_eq!(reg::QUATERNION_X, 56);
        assert_eq!(reg::QUATERNION_Y, 58);
        assert_eq!(reg::QUATERNION_Z, 60);
        assert_eq!(reg::GYRO_X, 62);
        // One undefined byte sits either side of the Y axis.
        assert_eq!(reg::GYRO_Y, 65);
        assert_eq!(reg::GYRO_Z, 68);
        assert_eq!(reg::ACC_X, 71);
        assert_eq!(reg::ACC_Y, 74);
        assert_eq!(reg::ACC_Z, 77);
        // Added from IMU.MD: 0 normal, 1 abnormal, bit 1 a dropped link.
        assert_eq!(reg::STATUS, 79);
    }
    /// The 23-byte sample block starts at the quaternion and stops one byte short of the
    /// status register, stepping over the undefined bytes on the way.
    #[test]
    fn sample_block_spans_the_strided_fields() {
        assert_eq!(reg::QUATERNION_X + 23, reg::STATUS);
        assert_eq!(reg::GYRO_X - reg::QUATERNION_X, 6);
        assert_eq!(reg::ACC_X - reg::QUATERNION_X, 15);
    }
}
