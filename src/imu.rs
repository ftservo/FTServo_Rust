use crate::{
    encoding::signed15, half_float, registers::*, Batch, Bus, ByteOrder, Device, Error, Request,
    Result,
};
use std::ops::Deref;

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
/// Contiguous register read; firmware-side simultaneous latching is not implied.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub quaternion: Quaternion,
    pub gyro: VectorRaw,
    pub acceleration: VectorRaw,
}
pub fn decode_imu_sample(p: &[u8]) -> Result<ImuSample> {
    if p.len() != 23 {
        return Err(Error::Packet("IMU sample requires 23 bytes"));
    }
    Ok(ImuSample {
        quaternion: Quaternion::from_raw(vector(p, 2))?,
        gyro: vector(&p[6..], 3),
        acceleration: vector(&p[15..], 3),
    })
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
            device: Device::new(bus, ByteOrder::Little, IMU_LOCK),
        }
    }
    pub fn without_write_response(mut self) -> Self {
        self.device.reply = false;
        self
    }
    pub fn read_quaternion_raw(&self, r: &Request, id: u8) -> Result<VectorRaw> {
        Ok(vector(&self.read(r, id, IMU_QUATERNION_X, 6)?, 2))
    }
    pub fn read_quaternion(&self, r: &Request, id: u8) -> Result<Quaternion> {
        Quaternion::from_raw(self.read_quaternion_raw(r, id)?)
    }
    pub fn read_gyro(&self, r: &Request, id: u8) -> Result<VectorRaw> {
        Ok(vector(&self.read(r, id, IMU_GYRO_X, 8)?, 3))
    }
    pub fn read_acceleration(&self, r: &Request, id: u8) -> Result<VectorRaw> {
        Ok(vector(&self.read(r, id, IMU_ACC_X, 8)?, 3))
    }
    pub fn read_sample(&self, r: &Request, id: u8) -> Result<ImuSample> {
        decode_imu_sample(&self.read(r, id, IMU_QUATERNION_X, 23)?)
    }
    pub fn sync_read_samples(&self, r: &Request, ids: &[u8]) -> Result<Batch<ImuSample>> {
        Ok(self
            .bus
            .sync_read(r, IMU_QUATERNION_X, 23, ids)?
            .decode(decode_imu_sample))
    }
}
