use crate::{
    encode_sign_magnitude,
    encoding::{signed10, signed15},
    protocol,
    registers::*,
    Batch, Bus, ByteOrder, Device, Error, Request, Result, WriteEntry,
};
use std::ops::Deref;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    SmsSts,
    Scscl,
    Hls,
}

/// Register units. Time is SCSCL-only, torque is HLS-only; acceleration applies to
/// SMS/STS and HLS. Unsupported nonzero fields are rejected before any serial I/O.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Motion {
    pub id: u8,
    pub position: i32,
    pub speed: u16,
    pub time: u16,
    pub acceleration: u8,
    pub torque: u16,
}
impl Motion {
    pub fn new(id: u8, position: i32) -> Self {
        Self {
            id,
            position,
            ..Self::default()
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Feedback {
    pub position: i32,
    pub speed: i32,
    pub load: i32,
    pub current: i32,
    pub voltage: u8,
    pub temperature: u8,
    pub moving: bool,
}

#[derive(Clone)]
pub struct Servo {
    device: Device,
    family: Family,
}
impl Deref for Servo {
    type Target = Device;
    fn deref(&self) -> &Device {
        &self.device
    }
}
impl Servo {
    pub fn new(bus: Bus, family: Family) -> Self {
        let (order, lock) = if family == Family::Scscl {
            (ByteOrder::Big, SCSCL_LOCK)
        } else {
            (ByteOrder::Little, LOCK)
        };
        Self {
            device: Device::new(bus, order, lock),
            family,
        }
    }
    pub fn sms_sts(bus: Bus) -> Self {
        Self::new(bus, Family::SmsSts)
    }
    pub fn scscl(bus: Bus) -> Self {
        Self::new(bus, Family::Scscl)
    }
    pub fn hls(bus: Bus) -> Self {
        Self::new(bus, Family::Hls)
    }
    pub fn family(&self) -> Family {
        self.family
    }
    /// Use only for a device configured to suppress write acknowledgments.
    pub fn without_write_response(mut self) -> Self {
        self.device.reply = false;
        self
    }
    pub fn enable_torque(&self, r: &Request, id: u8, enable: bool) -> Result<()> {
        self.write_u8(r, id, TORQUE_ENABLE, u8::from(enable))
    }
    fn position(&self, v: u16) -> i32 {
        if self.family == Family::Scscl {
            i32::from(v)
        } else {
            signed15(v)
        }
    }
    pub fn read_position(&self, r: &Request, id: u8) -> Result<i32> {
        Ok(self.position(self.read_u16(r, id, PRESENT_POSITION)?))
    }
    pub fn read_speed(&self, r: &Request, id: u8) -> Result<i32> {
        Ok(signed15(self.read_u16(r, id, PRESENT_SPEED)?))
    }
    pub fn read_load(&self, r: &Request, id: u8) -> Result<i32> {
        Ok(signed10(self.read_u16(r, id, PRESENT_LOAD)?))
    }
    pub fn read_current(&self, r: &Request, id: u8) -> Result<i32> {
        Ok(signed15(self.read_u16(r, id, PRESENT_CURRENT)?))
    }
    pub fn read_voltage(&self, r: &Request, id: u8) -> Result<u8> {
        self.read_u8(r, id, PRESENT_VOLTAGE)
    }
    pub fn read_temperature(&self, r: &Request, id: u8) -> Result<u8> {
        self.read_u8(r, id, PRESENT_TEMPERATURE)
    }
    pub fn read_moving(&self, r: &Request, id: u8) -> Result<bool> {
        Ok(self.read_u8(r, id, MOVING)? != 0)
    }
    pub fn read_position_speed(&self, r: &Request, id: u8) -> Result<(i32, i32)> {
        let p = self.read(r, id, PRESENT_POSITION, 4)?;
        Ok((
            self.position(self.order.word(&p)),
            signed15(self.order.word(&p[2..])),
        ))
    }
    pub fn decode_feedback(&self, p: &[u8]) -> Result<Feedback> {
        if p.len() != 15 {
            return Err(Error::Packet("feedback requires 15 bytes"));
        }
        Ok(Feedback {
            position: self.position(self.order.word(p)),
            speed: signed15(self.order.word(&p[2..])),
            load: signed10(self.order.word(&p[4..])),
            current: signed15(self.order.word(&p[13..])),
            voltage: p[6],
            temperature: p[7],
            moving: p[10] != 0,
        })
    }
    pub fn read_feedback(&self, r: &Request, id: u8) -> Result<Feedback> {
        self.decode_feedback(&self.read(r, id, PRESENT_POSITION, 15)?)
    }
    pub fn sync_read_feedback(&self, r: &Request, ids: &[u8]) -> Result<Batch<Feedback>> {
        if self.family == Family::Scscl {
            return Err(Error::Unsupported("SCSCL sync read"));
        }
        Ok(self
            .bus
            .sync_read(r, PRESENT_POSITION, 15, ids)?
            .decode(|p| self.decode_feedback(p)))
    }
    fn motion_bytes(&self, m: &Motion) -> Result<Vec<u8>> {
        if !protocol::valid_id(m.id, true) {
            return Err(Error::InvalidArgument("motion ID"));
        }
        if self.family == Family::Scscl {
            if !(0..=65535).contains(&m.position) || m.acceleration != 0 || m.torque != 0 {
                return Err(Error::InvalidArgument("SCSCL motion fields"));
            }
            let mut p = (m.position as u16).to_be_bytes().to_vec();
            p.extend_from_slice(&m.time.to_be_bytes());
            p.extend_from_slice(&m.speed.to_be_bytes());
            Ok(p)
        } else {
            if m.time != 0 || self.family == Family::SmsSts && m.torque != 0 {
                return Err(Error::InvalidArgument("unsupported motion time/torque"));
            }
            let pos = encode_sign_magnitude(m.position, 15)?;
            let mut p = vec![m.acceleration];
            p.extend_from_slice(&pos.to_le_bytes());
            p.extend_from_slice(&m.torque.to_le_bytes());
            p.extend_from_slice(&m.speed.to_le_bytes());
            Ok(p)
        }
    }
    fn motion_address(&self) -> u8 {
        if self.family == Family::Scscl {
            GOAL_POSITION
        } else {
            ACCELERATION
        }
    }
    pub fn write_position(&self, r: &Request, m: Motion) -> Result<()> {
        let p = self.motion_bytes(&m)?;
        self.write(r, m.id, self.motion_address(), &p)
    }
    pub fn reg_write_position(&self, r: &Request, m: Motion) -> Result<()> {
        let p = self.motion_bytes(&m)?;
        self.reg_write(r, m.id, self.motion_address(), &p)
    }
    pub fn sync_write_position(&self, r: &Request, motions: &[Motion]) -> Result<()> {
        let entries: Result<Vec<_>> = motions
            .iter()
            .map(|m| {
                Ok(WriteEntry {
                    id: m.id,
                    data: self.motion_bytes(m)?,
                })
            })
            .collect();
        self.bus.sync_write(
            r,
            self.motion_address(),
            if self.family == Family::Scscl { 6 } else { 7 },
            &entries?,
        )
    }
    pub fn wheel_mode(&self, r: &Request, id: u8) -> Result<()> {
        self.extended_mode(r, id, 1)
    }
    pub fn position_mode(&self, r: &Request, id: u8) -> Result<()> {
        self.extended_mode(r, id, 0)
    }
    fn extended_mode(&self, r: &Request, id: u8, mode: u8) -> Result<()> {
        if self.family == Family::Scscl {
            return Err(Error::Unsupported("SCSCL mode register"));
        }
        self.write_u8(r, id, MODE, mode)
    }
    /// Signed wheel speed. For SMS/STS torque must be zero.
    pub fn write_speed(
        &self,
        r: &Request,
        id: u8,
        speed: i32,
        acceleration: u8,
        torque: u16,
    ) -> Result<()> {
        if self.family == Family::Scscl {
            return Err(Error::Unsupported("SCSCL wheel speed; use PWM"));
        }
        let speed = encode_sign_magnitude(speed, 15)?;
        self.write_position(
            r,
            Motion {
                id,
                speed,
                acceleration,
                torque,
                ..Motion::default()
            },
        )
    }
    pub fn pwm_mode(&self, r: &Request, id: u8) -> Result<()> {
        if self.family != Family::Scscl {
            return Err(Error::Unsupported("PWM mode is SCSCL-only"));
        }
        self.write(r, id, MIN_ANGLE, &[0; 4])
    }
    pub fn write_pwm(&self, r: &Request, id: u8, value: i32) -> Result<()> {
        if self.family != Family::Scscl {
            return Err(Error::Unsupported("PWM is SCSCL-only"));
        }
        self.write_u16(r, id, GOAL_TIME, encode_sign_magnitude(value, 10)?)
    }
    fn require_hls(&self) -> Result<()> {
        if self.family != Family::Hls {
            Err(Error::Unsupported("HLS-only operation"))
        } else {
            Ok(())
        }
    }
    pub fn torque_mode(&self, r: &Request, id: u8) -> Result<()> {
        self.require_hls()?;
        self.write_u8(r, id, MODE, 2)
    }
    pub fn write_torque(&self, r: &Request, id: u8, value: i32) -> Result<()> {
        self.require_hls()?;
        self.write_u16(r, id, GOAL_TORQUE, encode_sign_magnitude(value, 15)?)
    }
    /// Upstream HLS state/turn-count reset, not advertised as factory reset.
    pub fn reset(&self, r: &Request, id: u8) -> Result<()> {
        self.require_hls()?;
        self.bus
            .command(r, id, protocol::RESET, &[], self.reply.then_some(0))
            .map(|_| ())
    }
    pub fn calibrate_offset(&self, r: &Request, id: u8, position: u16) -> Result<()> {
        self.require_hls()?;
        self.bus
            .command(
                r,
                id,
                protocol::OFFSET_CALIBRATION,
                &position.to_le_bytes(),
                self.reply.then_some(0),
            )
            .map(|_| ())
    }
}
