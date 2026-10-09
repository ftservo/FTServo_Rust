//! Servos: one entry point, three families.
//!
//! [`Servo`] is the handle callers hold; what it does differently per family is described by
//! a static [`ModelSpec`], one per module inlined at the bottom of this file. The only place
//! a [`Family`] turns into behaviour is [`Family::spec`].

use crate::{
    encode_sign_magnitude,
    protocol::{self, signed10, signed15},
    registers::*,
    Bus, ByteOrder, Device, Error, Request, Result, WriteEntry,
};
use std::ops::Deref;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    SmsSts,
    Scscl,
    Hls,
}

/// How a family reports a position.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum PositionEncoding {
    /// Bit 15 is a direction bit and the magnitude is the low 15 bits.
    Signed15,
    /// The raw word is the position.
    Unsigned,
}

/// What a family can do beyond the operations every servo shares.
///
/// The bare operations — reading position, speed, load, voltage and temperature, writing a
/// position — are on [`Servo`] unconditionally. Refusing an unsupported one means producing
/// the same [`Error::Unsupported`] those entry points always have, which is what these flags
/// decide.
#[derive(Clone, Copy)]
pub(crate) struct Capabilities {
    /// Mode register: wheel and position selection.
    pub modes: bool,
    /// Signed wheel speed, carried in the motion frame.
    pub wheel_speed: bool,
    /// PWM output.
    pub pwm: bool,
    /// Torque mode and torque writes.
    pub torque_mode: bool,
    /// Offset calibration.
    pub calibrate_offset: bool,
}

/// Everything that varies by family, in one place.
///
/// A static table rather than a trait object or a type parameter: [`Servo`] is `Clone` and
/// lives on 50 Hz polling paths, so a boxed value would add an allocation per handle and a
/// type parameter would change the public shape of the type. Every field can be built in a
/// `const` context, which is what lets each family keep its table beside its register list.
pub(crate) struct ModelSpec {
    /// Byte order of 16-bit words, fixed when the device handle is built.
    pub order: ByteOrder,
    /// EEPROM unlock register.
    pub lock: u8,
    /// How a reported position is decoded.
    pub position_encoding: PositionEncoding,
    /// First register of a motion frame.
    pub motion_address: u8,
    /// Bytes per motion entry in a synchronous write.
    pub motion_entry_len: usize,
    /// Address of the present-current register.
    pub present_current: u8,
    pub capabilities: Capabilities,
    /// This family's motion frame, including its own field checks.
    pub encode_motion: fn(&Motion) -> Result<Vec<u8>>,
}

impl Family {
    /// The only place in the crate a family turns into behaviour.
    pub(crate) fn spec(self) -> &'static ModelSpec {
        match self {
            Family::SmsSts => &sms_sts::SPEC,
            Family::Scscl => &scscl::SPEC,
            Family::Hls => &hls::SPEC,
        }
    }
}

/// The mode register, at 33 on both families that have one.
///
/// Not in the shared table, because SCSCL has no mode register at all; and not in either
/// model module, because `extended_mode` and `torque_mode` each serve two families. Whether
/// the write may happen at all is `Capabilities::modes`.
const MODE: u8 = sms_sts::registers::MODE;

/// The 7-byte little-endian motion frame SMS/STS and HLS share: acceleration, 15-bit
/// sign-magnitude position, torque, speed.
///
/// The two families differ only in which fields they accept, and that is checked before this
/// is called.
pub(crate) fn encode_motion_le7(m: &Motion) -> Result<Vec<u8>> {
    let position = encode_sign_magnitude(m.position, 15)?;
    let mut p = vec![m.acceleration];
    p.extend_from_slice(&position.to_le_bytes());
    p.extend_from_slice(&m.torque.to_le_bytes());
    p.extend_from_slice(&m.speed.to_le_bytes());
    Ok(p)
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

/// The 15-byte feedback block every servo family reports from [`PRESENT_POSITION`], filled in
/// by the application.
///
/// The crate does not decode the block itself: its layout is a family fact — byte order,
/// position encoding, and where in the 15 bytes each field sits — and a whole-block decoder
/// would have to guess the one callers already know. Read the bytes (`read`, or
/// [`Bus::sync_read_decoded`] across several devices) and decode them with the field codecs
/// ([`crate::decode_sign_magnitude`]); `examples/hls/sync_read.rs` and
/// `examples/sms_sts/sync_read.rs` each carry a worked decode for their own family and use it
/// in a 50 Hz loop.
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
        let spec = family.spec();
        Self {
            device: Device::new(bus, spec.order, spec.lock),
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
    /// This servo's family table. The one thing `Servo` needs to know about its family.
    fn spec(&self) -> &'static ModelSpec {
        self.family.spec()
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
        match self.spec().position_encoding {
            PositionEncoding::Signed15 => signed15(v),
            PositionEncoding::Unsigned => i32::from(v),
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
        Ok(signed15(self.read_u16(
            r,
            id,
            self.spec().present_current,
        )?))
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
    fn motion_bytes(&self, m: &Motion) -> Result<Vec<u8>> {
        if !protocol::valid_id(m.id, true) {
            return Err(Error::InvalidArgument("motion ID"));
        }
        (self.spec().encode_motion)(m)
    }
    pub fn write_position(&self, r: &Request, m: Motion) -> Result<()> {
        let p = self.motion_bytes(&m)?;
        self.write(r, m.id, self.spec().motion_address, &p)
    }
    pub fn reg_write_position(&self, r: &Request, m: Motion) -> Result<()> {
        let p = self.motion_bytes(&m)?;
        self.reg_write(r, m.id, self.spec().motion_address, &p)
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
        let spec = self.spec();
        self.bus
            .sync_write(r, spec.motion_address, spec.motion_entry_len, &entries?)
    }
    pub fn wheel_mode(&self, r: &Request, id: u8) -> Result<()> {
        self.extended_mode(r, id, 1)
    }
    pub fn position_mode(&self, r: &Request, id: u8) -> Result<()> {
        self.extended_mode(r, id, 0)
    }
    fn extended_mode(&self, r: &Request, id: u8, mode: u8) -> Result<()> {
        if !self.spec().capabilities.modes {
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
        if !self.spec().capabilities.wheel_speed {
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
        if !self.spec().capabilities.pwm {
            return Err(Error::Unsupported("PWM mode is SCSCL-only"));
        }
        self.write(r, id, MIN_ANGLE, &[0; 4])
    }
    pub fn write_pwm(&self, r: &Request, id: u8, value: i32) -> Result<()> {
        if !self.spec().capabilities.pwm {
            return Err(Error::Unsupported("PWM is SCSCL-only"));
        }
        self.write_u16(
            r,
            id,
            scscl::registers::GOAL_TIME,
            encode_sign_magnitude(value, 10)?,
        )
    }
    /// Gate for an operation only one family has, keeping the message these entry points have
    /// always produced.
    fn require(&self, supported: bool) -> Result<()> {
        if supported {
            Ok(())
        } else {
            Err(Error::Unsupported("HLS-only operation"))
        }
    }
    pub fn torque_mode(&self, r: &Request, id: u8) -> Result<()> {
        self.require(self.spec().capabilities.torque_mode)?;
        self.write_u8(r, id, MODE, 2)
    }
    pub fn write_torque(&self, r: &Request, id: u8, value: i32) -> Result<()> {
        self.require(self.spec().capabilities.torque_mode)?;
        self.write_u16(
            r,
            id,
            hls::registers::GOAL_TORQUE,
            encode_sign_magnitude(value, 15)?,
        )
    }
    pub fn calibrate_offset(&self, r: &Request, id: u8, position: u16) -> Result<()> {
        self.require(self.spec().capabilities.calibrate_offset)?;
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

// SMS / STS ---------------------------------------------------------------------------------

/// SMS / STS servos.
///
/// Little-endian words, 15-bit sign-magnitude positions, and a 7-byte motion frame running
/// from the acceleration to the goal speed. Wheel speed is that same frame with a speed and
/// no position, so it goes through `Servo::write_speed`; this family has no separate speed
/// register. Torque is a HLS-only field and is refused here.
pub mod sms_sts {
    use super::{encode_motion_le7, Capabilities, ModelSpec, Motion, PositionEncoding};
    use crate::{ByteOrder, Error, Result};

    /// Addresses this family places differently from the other two.
    pub mod registers {
        /// Position offset, in EEPROM.
        pub const OFFSET: u8 = 31;
        /// Mode of operation: `0` position, `1` wheel.
        pub const MODE: u8 = 33;
        /// Start of the motion frame — the acceleration byte.
        pub const ACCELERATION: u8 = 41;
        /// The official name is **open-loop PWM speed**; the SDK writes torque to this
        /// address. Same address, different name.
        pub const GOAL_TIME: u8 = 44;
        /// Maximum torque, in EEPROM.
        pub const TORQUE_LIMIT: u8 = 48;
        /// EEPROM lock register.
        pub const LOCK: u8 = 55;
        /// Present current draw.
        pub const PRESENT_CURRENT: u8 = 69;
    }

    pub(crate) const SPEC: ModelSpec = ModelSpec {
        order: ByteOrder::Little,
        lock: registers::LOCK,
        position_encoding: PositionEncoding::Signed15,
        motion_address: registers::ACCELERATION,
        motion_entry_len: 7,
        present_current: registers::PRESENT_CURRENT,
        capabilities: Capabilities {
            modes: true,
            wheel_speed: true,
            pwm: false,
            torque_mode: false,
            calibrate_offset: false,
        },
        encode_motion,
    };

    /// Acceleration, position, torque and speed, all little-endian. Time and torque belong to
    /// other families, so a nonzero one is a caller mistake rather than something to ignore.
    fn encode_motion(m: &Motion) -> Result<Vec<u8>> {
        if m.time != 0 || m.torque != 0 {
            return Err(Error::InvalidArgument("unsupported motion time/torque"));
        }
        encode_motion_le7(m)
    }
}

// SCS / SCSCL -------------------------------------------------------------------------------

/// SCS / SCSCL servos.
///
/// Big-endian words, positions reported as plain unsigned 16-bit values, and a 6-byte motion
/// frame carrying a running time and a speed instead of an acceleration and a torque. PWM
/// output is the one thing this family has that the others do not, and it has none of theirs:
/// no mode register, no wheel speed, no synchronous read.
pub mod scscl {
    use super::{Capabilities, ModelSpec, Motion, PositionEncoding};
    use crate::{ByteOrder, Error, Result};

    /// Addresses this family places differently from the other two.
    pub mod registers {
        /// EEPROM lock register. The other families put theirs at 55.
        pub const LOCK: u8 = 48;
        /// The official name is **running time**. The other families carry their time field
        /// elsewhere in the motion frame; this one has a register for it.
        pub const GOAL_TIME: u8 = 44;
        /// Present current.
        ///
        /// **Not in the official memory map** for this family. Kept at its historical value
        /// so `Servo::read_current` behaves exactly as it always has.
        pub const PRESENT_CURRENT: u8 = 69;
    }

    pub(crate) const SPEC: ModelSpec = ModelSpec {
        order: ByteOrder::Big,
        lock: registers::LOCK,
        position_encoding: PositionEncoding::Unsigned,
        motion_address: crate::registers::GOAL_POSITION,
        motion_entry_len: 6,
        present_current: registers::PRESENT_CURRENT,
        capabilities: Capabilities {
            modes: false,
            wheel_speed: false,
            pwm: true,
            torque_mode: false,
            calibrate_offset: false,
        },
        encode_motion,
    };

    /// Position, running time and speed, all big-endian — bytes in the order they go on the
    /// wire. Acceleration and torque are fields the other families' frames have and this
    /// one's does not.
    fn encode_motion(m: &Motion) -> Result<Vec<u8>> {
        if !(0..=65535).contains(&m.position) || m.acceleration != 0 || m.torque != 0 {
            return Err(Error::InvalidArgument("SCSCL motion fields"));
        }
        let mut p = (m.position as u16).to_be_bytes().to_vec();
        p.extend_from_slice(&m.time.to_be_bytes());
        p.extend_from_slice(&m.speed.to_be_bytes());
        Ok(p)
    }
}

// HLS ---------------------------------------------------------------------------------------

/// HLS servos.
///
/// Little-endian words and 15-bit sign-magnitude positions, like SMS/STS, but with the full
/// feature set: mode register, wheel speed, torque mode, and the offset-calibration command.
/// The `RESET` instruction every device speaks is on [`crate::Device`], not here.
pub mod hls {
    use super::{encode_motion_le7, Capabilities, ModelSpec, Motion, PositionEncoding};
    use crate::{ByteOrder, Error, Result};

    /// Addresses this family places differently from the other two.
    pub mod registers {
        /// Position offset, in EEPROM.
        pub const OFFSET: u8 = 31;
        /// Mode of operation: `0` position, `1` wheel, `2` torque.
        pub const MODE: u8 = 33;
        /// Start of the motion frame — the acceleration byte.
        pub const ACCELERATION: u8 = 41;
        /// The official name is **target current**; the SDK writes torque to this address.
        /// Same address, different name.
        pub const GOAL_TORQUE: u8 = 44;
        /// Maximum torque, in EEPROM.
        pub const TORQUE_LIMIT: u8 = 48;
        /// EEPROM lock register.
        pub const LOCK: u8 = 55;
        /// Present current draw.
        pub const PRESENT_CURRENT: u8 = 69;
    }

    pub(crate) const SPEC: ModelSpec = ModelSpec {
        order: ByteOrder::Little,
        lock: registers::LOCK,
        position_encoding: PositionEncoding::Signed15,
        motion_address: registers::ACCELERATION,
        motion_entry_len: 7,
        present_current: registers::PRESENT_CURRENT,
        capabilities: Capabilities {
            modes: true,
            wheel_speed: true,
            pwm: false,
            torque_mode: true,
            calibrate_offset: true,
        },
        encode_motion,
    };

    /// Acceleration, position, torque and speed, all little-endian. Unlike SMS/STS, torque is
    /// a real field here — the running time is the one that belongs to another family.
    fn encode_motion(m: &Motion) -> Result<Vec<u8>> {
        if m.time != 0 {
            return Err(Error::InvalidArgument("unsupported motion time/torque"));
        }
        encode_motion_le7(m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each table's byte order, lock register, position decoding, motion frame shape and
    /// capability set, pinned against the family it describes. A field read off the wrong row
    /// changes wire behaviour silently, which is what these assertions are for.
    #[test]
    fn sms_sts_spec() {
        let s = &sms_sts::SPEC;
        assert_eq!(s.order, ByteOrder::Little);
        assert_eq!(s.lock, 55);
        assert!(matches!(s.position_encoding, PositionEncoding::Signed15));
        assert_eq!(s.motion_address, 41);
        assert_eq!(s.motion_entry_len, 7);
        assert_eq!(s.present_current, 69);
        let c = s.capabilities;
        assert!(c.modes && c.wheel_speed);
        assert!(!c.pwm && !c.torque_mode && !c.calibrate_offset);
    }
    #[test]
    fn sms_sts_motion_frame() {
        let m = Motion {
            id: 1,
            position: -100,
            speed: 200,
            acceleration: 3,
            ..Motion::default()
        };
        let p = (sms_sts::SPEC.encode_motion)(&m).unwrap();
        assert_eq!(p.len(), 7);
        assert_eq!(p[0], 3);
        assert_eq!(&p[1..3], &(100u16 | 0x8000).to_le_bytes());
        assert_eq!(&p[3..5], &0u16.to_le_bytes());
        assert_eq!(&p[5..7], &200u16.to_le_bytes());
        assert!((sms_sts::SPEC.encode_motion)(&Motion { time: 1, ..m }).is_err());
        assert!((sms_sts::SPEC.encode_motion)(&Motion { torque: 1, ..m }).is_err());
    }
    #[test]
    fn scscl_spec() {
        let s = &scscl::SPEC;
        assert_eq!(s.order, ByteOrder::Big);
        assert_eq!(s.lock, 48);
        assert!(matches!(s.position_encoding, PositionEncoding::Unsigned));
        assert_eq!(s.motion_address, 42);
        assert_eq!(s.motion_entry_len, 6);
        assert_eq!(s.present_current, 69);
        let c = s.capabilities;
        assert!(c.pwm);
        assert!(!c.modes && !c.wheel_speed);
        assert!(!c.torque_mode && !c.calibrate_offset);
    }
    #[test]
    fn scscl_motion_frame() {
        let m = Motion {
            id: 1,
            position: 1000,
            time: 5,
            speed: 300,
            ..Motion::default()
        };
        // Position, running time and speed, big-endian, in that order.
        assert_eq!(
            (scscl::SPEC.encode_motion)(&m).unwrap(),
            [0x03, 0xE8, 0x00, 0x05, 0x01, 0x2C]
        );
        assert!((scscl::SPEC.encode_motion)(&Motion {
            acceleration: 1,
            ..m
        })
        .is_err());
        assert!((scscl::SPEC.encode_motion)(&Motion { torque: 1, ..m }).is_err());
        assert!((scscl::SPEC.encode_motion)(&Motion {
            position: 65536,
            ..m
        })
        .is_err());
        assert!((scscl::SPEC.encode_motion)(&Motion { position: -1, ..m }).is_err());
    }
    #[test]
    fn hls_spec() {
        let s = &hls::SPEC;
        assert_eq!(s.order, ByteOrder::Little);
        assert_eq!(s.lock, 55);
        assert!(matches!(s.position_encoding, PositionEncoding::Signed15));
        assert_eq!(s.motion_address, 41);
        assert_eq!(s.motion_entry_len, 7);
        assert_eq!(s.present_current, 69);
        let c = s.capabilities;
        assert!(c.modes && c.wheel_speed);
        assert!(c.torque_mode && c.calibrate_offset);
        assert!(!c.pwm);
    }
    #[test]
    fn hls_motion_frame_accepts_torque() {
        let m = Motion {
            id: 1,
            position: -100,
            speed: 200,
            acceleration: 3,
            torque: 500,
            ..Motion::default()
        };
        let p = (hls::SPEC.encode_motion)(&m).unwrap();
        assert_eq!(p.len(), 7);
        assert_eq!(&p[1..3], &(100u16 | 0x8000).to_le_bytes());
        assert_eq!(&p[3..5], &500u16.to_le_bytes());
        assert!((hls::SPEC.encode_motion)(&Motion { time: 1, ..m }).is_err());
    }
}
