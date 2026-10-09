//! The wire protocol: what goes on the bus, and what a reply means.
//!
//! Everything here sits at one level — between the bytes a port carries and the device
//! models above it:
//!
//! * **Packet layout** — the `FF FF` header, the length byte, the checksum, and the
//!   instruction codes that fill them in.
//! * **Field codecs** — how a device packs a signed number into a 16-bit register:
//!   sign-magnitude with a direction bit, the fixed 15- and 10-bit forms, and IEEE 754
//!   binary16 for the IMU's quaternion.
//! * **Register and instruction access** — [`Device`], which turns an address and a byte
//!   count into a typed read or write over a [`Bus`] and sends the whole-device instructions
//!   ([`Device::ping`], [`Device::reset`]); and [`ByteOrder`], which decides how two bytes
//!   become one number.
//!
//! Nothing here knows about a particular servo model; that is the layer above.

use crate::{Bus, DeviceError, Error, Request, Result};

pub const BROADCAST_ID: u8 = 0xFE;
pub const MAX_ID: u8 = 0xFC;
pub const MAX_PACKET_SIZE: usize = 250;
pub const PING: u8 = 1;
pub const READ: u8 = 2;
pub const WRITE: u8 = 3;
pub const REG_WRITE: u8 = 4;
pub const ACTION: u8 = 5;
pub const RESET: u8 = 0x0A;
pub const OFFSET_CALIBRATION: u8 = 0x0B;
pub const SYNC_READ: u8 = 0x82;
pub const SYNC_WRITE: u8 = 0x83;

/// Model number register — the value a `PING` is answered with, at address 3 on every device
/// this SDK speaks to.
///
/// It lives in this layer rather than in `registers` because [`Device::ping`] reads it, and
/// reaching up into the model layer's address table from here would point the dependency the
/// wrong way.
pub const MODEL: u8 = 3;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Status {
    pub id: u8,
    pub flags: u8,
    pub data: Vec<u8>,
}
impl Status {
    pub fn device_error(&self) -> Option<DeviceError> {
        (self.flags != 0).then_some(DeviceError {
            id: self.id,
            flags: self.flags,
        })
    }
}
pub(crate) fn valid_id(id: u8, broadcast: bool) -> bool {
    id <= MAX_ID || broadcast && id == BROADCAST_ID
}
pub(crate) fn register_range(address: u8, length: usize, max: usize) -> Result<()> {
    if length == 0 || length > max || length > 256 - usize::from(address) {
        return Err(Error::InvalidArgument("register address/length"));
    }
    Ok(())
}
pub fn encode_packet(id: u8, instruction: u8, params: &[u8]) -> Result<Vec<u8>> {
    // Sized up front: `params.len() + 6` is the exact packet length, so the two growths
    // the previous form paid (`vec![...]` at capacity 5, then `push` past it) are gone.
    let mut p = Vec::with_capacity(params.len() + 6);
    encode_packet_into(&mut p, id, instruction, params)?;
    Ok(p)
}
/// [`encode_packet`], written into a reused buffer.
///
/// `out` is cleared first and keeps its capacity, so a caller that holds one buffer
/// across a poll loop stops allocating packets after its first round.
pub(crate) fn encode_packet_into(
    out: &mut Vec<u8>,
    id: u8,
    instruction: u8,
    params: &[u8],
) -> Result<()> {
    if !valid_id(id, true) || params.len() > MAX_PACKET_SIZE - 6 {
        return Err(Error::InvalidArgument("ID or packet size"));
    }
    out.clear();
    out.reserve(params.len() + 6);
    out.extend_from_slice(&[255, 255, id, (params.len() + 2) as u8, instruction]);
    out.extend_from_slice(params);
    let sum = out[2..].iter().fold(0u8, |s, v| s.wrapping_add(*v));
    out.push(!sum);
    Ok(())
}
/// Encode a broadcast SYNC_READ request — instruction, address, length, and the ID list.
///
/// Separate from [`encode_packet_into`] only so the ID list never has to be assembled
/// into a `params` vector first: `ids` goes into the packet directly, which is one fewer
/// buffer on the path a 50 Hz loop takes every tick.
///
/// Range and ID validation is the caller's ([`register_range`], [`valid_id`]);
/// [`crate::Bus::sync_read`] does both before getting here.
pub(crate) fn encode_sync_read(out: &mut Vec<u8>, address: u8, length: usize, ids: &[u8]) {
    debug_assert!(length <= 256 - usize::from(address));
    out.clear();
    out.reserve(ids.len() + 11);
    out.extend_from_slice(&[255, 255, BROADCAST_ID, 0, SYNC_READ, address, length as u8]);
    out.extend_from_slice(ids);
    // The length byte counts every byte after the id: instruction, parameters, and the
    // checksum still to come.
    out[3] = (out.len() - 3) as u8;
    let sum = out[2..].iter().fold(0u8, |s, v| s.wrapping_add(*v));
    out.push(!sum);
}
/// Read a status packet's header in place, returning `(id, flags)`.
///
/// [`decode_status`] hands back an owned `Status` with a `Vec` payload; this one borrows,
/// and the difference is what lets a read loop pass the payload straight to a decoder
/// instead of allocating one `Vec` per device per round just to hand it on. See
/// [`crate::Bus::sync_read_decoded`].
pub(crate) fn validate_status(p: &[u8]) -> Result<(u8, u8)> {
    if p.len() < 6
        || p.len() > MAX_PACKET_SIZE
        || p[0..2] != [255, 255]
        || !valid_id(p[2], false)
        || usize::from(p[3]) + 4 != p.len()
        || p[4] > 127
    {
        return Err(Error::Packet("header, ID, length or status"));
    }
    if p[2..].iter().fold(0u8, |s, v| s.wrapping_add(*v)) != 255 {
        return Err(Error::Checksum);
    }
    Ok((p[2], p[4]))
}
pub fn decode_status(p: &[u8]) -> Result<Status> {
    let (id, flags) = validate_status(p)?;
    Ok(Status {
        id,
        flags,
        data: p[5..p.len() - 1].to_vec(),
    })
}

// Field codecs ------------------------------------------------------------------------------
//
// A device packs a signed quantity into a 16-bit register in one of two ways, and neither is
// two's complement: sign-magnitude with a direction bit, or a fixed-width truncated form.
// `half_float` belongs here for the same reason the others do — it is how the IMU stores a
// quaternion in two bytes — even though it has nothing to do with sign-magnitude.

/// FEETECH direction-bit decoding, not two's complement. sign_bit must be 0..15.
pub fn decode_sign_magnitude(raw: u16, sign_bit: u8) -> Result<i32> {
    if sign_bit > 15 {
        return Err(Error::InvalidArgument("sign bit"));
    }
    let mask = 1u16 << sign_bit;
    Ok(if raw & mask != 0 {
        -i32::from(raw & !mask)
    } else {
        i32::from(raw)
    })
}
pub fn encode_sign_magnitude(value: i32, sign_bit: u8) -> Result<u16> {
    if sign_bit > 15 {
        return Err(Error::InvalidArgument("sign bit"));
    }
    let limit = (1i32 << sign_bit) - 1;
    if value < -limit || value > limit {
        return Err(Error::InvalidArgument("signed magnitude overflow"));
    }
    Ok(if value < 0 {
        (-value) as u16 | (1 << sign_bit)
    } else {
        value as u16
    })
}
pub(crate) fn signed15(raw: u16) -> i32 {
    if raw & 0x8000 != 0 {
        -i32::from(raw & 0x7fff)
    } else {
        i32::from(raw)
    }
}
pub(crate) fn signed10(raw: u16) -> i32 {
    if raw & 0x0400 != 0 {
        -i32::from(raw & !0x0400)
    } else {
        i32::from(raw)
    }
}

/// IEEE 754 binary16 to f64, including negative zero, subnormals, NaN and infinity.
pub fn half_float(raw: u16) -> f64 {
    let sign = if raw & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = i32::from((raw >> 10) & 31);
    let frac = f64::from(raw & 1023);
    match exp {
        31 => {
            if frac != 0.0 {
                f64::NAN
            } else {
                sign * f64::INFINITY
            }
        }
        0 => sign * frac * 2f64.powi(-24),
        _ => sign * (1.0 + frac / 1024.0) * 2f64.powi(exp - 15),
    }
}

// Register access ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ByteOrder {
    Little,
    Big,
}
impl ByteOrder {
    pub(crate) fn word(self, p: &[u8]) -> u16 {
        let pair = [p[0], p[1]];
        match self {
            Self::Little => u16::from_le_bytes(pair),
            Self::Big => u16::from_be_bytes(pair),
        }
    }
    pub(crate) fn bytes(self, v: u16) -> [u8; 2] {
        match self {
            Self::Little => v.to_le_bytes(),
            Self::Big => v.to_be_bytes(),
        }
    }
}
/// Common raw-register API. Servo and Imu dereference to this type.
#[derive(Clone)]
pub struct Device {
    pub(crate) bus: Bus,
    pub(crate) order: ByteOrder,
    lock: u8,
    pub(crate) reply: bool,
}
impl Device {
    pub(crate) fn new(bus: Bus, order: ByteOrder, lock: u8) -> Self {
        Self {
            bus,
            order,
            lock,
            reply: true,
        }
    }
    pub fn byte_order(&self) -> ByteOrder {
        self.order
    }
    pub fn ping(&self, r: &Request, id: u8) -> Result<u16> {
        self.bus.ping(r, id)?;
        self.read_u16(r, id, MODEL)
    }
    /// The `RESET` instruction: clears the device's accumulated state (on HLS, for instance,
    /// the multi-turn count). **Not** a factory reset — the EEPROM is untouched.
    ///
    /// A bus command rather than a register write, so it sits here beside [`Self::ping`] and
    /// not in the servo layer: the instruction belongs to the protocol, and what a given
    /// device makes of it is that device's business.
    pub fn reset(&self, r: &Request, id: u8) -> Result<()> {
        self.bus
            .command(r, id, RESET, &[], self.reply.then_some(0))
            .map(|_| ())
    }
    pub fn read(&self, r: &Request, id: u8, address: u8, length: usize) -> Result<Vec<u8>> {
        self.bus.read(r, id, address, length)
    }
    pub fn write(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.bus
            .write_command(r, id, address, data, WRITE, self.reply)
    }
    pub fn reg_write(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.bus
            .write_command(r, id, address, data, REG_WRITE, self.reply)
    }
    pub fn read_u8(&self, r: &Request, id: u8, address: u8) -> Result<u8> {
        Ok(self.read(r, id, address, 1)?[0])
    }
    pub fn read_u16(&self, r: &Request, id: u8, address: u8) -> Result<u16> {
        Ok(self.order.word(&self.read(r, id, address, 2)?))
    }
    /// Python word-pair convention: low word first, each word in device byte order.
    /// SCSCL is therefore not conventional big-endian u32.
    pub fn read_u32(&self, r: &Request, id: u8, address: u8) -> Result<u32> {
        let p = self.read(r, id, address, 4)?;
        Ok(u32::from(self.order.word(&p)) | (u32::from(self.order.word(&p[2..])) << 16))
    }
    pub fn write_u8(&self, r: &Request, id: u8, address: u8, value: u8) -> Result<()> {
        self.write(r, id, address, &[value])
    }
    pub fn write_u16(&self, r: &Request, id: u8, address: u8, value: u16) -> Result<()> {
        self.write(r, id, address, &self.order.bytes(value))
    }
    pub fn write_u32(&self, r: &Request, id: u8, address: u8, value: u32) -> Result<()> {
        let mut p = self.order.bytes(value as u16).to_vec();
        p.extend_from_slice(&self.order.bytes((value >> 16) as u16));
        self.write(r, id, address, &p)
    }
    pub fn action(&self, r: &Request, id: u8) -> Result<()> {
        self.bus
            .command(r, id, ACTION, &[], self.reply.then_some(0))
            .map(|_| ())
    }
    pub fn lock_eprom(&self, r: &Request, id: u8) -> Result<()> {
        self.write_u8(r, id, self.lock, 1)
    }
    pub fn unlock_eprom(&self, r: &Request, id: u8) -> Result<()> {
        self.write_u8(r, id, self.lock, 0)
    }
}
