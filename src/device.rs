use crate::{protocol, registers, Bus, Request, Result};

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
        self.read_u16(r, id, registers::MODEL)
    }
    pub fn read(&self, r: &Request, id: u8, address: u8, length: usize) -> Result<Vec<u8>> {
        self.bus.read(r, id, address, length)
    }
    pub fn write(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.bus
            .write_command(r, id, address, data, protocol::WRITE, self.reply)
    }
    pub fn reg_write(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.bus
            .write_command(r, id, address, data, protocol::REG_WRITE, self.reply)
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
            .command(r, id, protocol::ACTION, &[], self.reply.then_some(0))
            .map(|_| ())
    }
    pub fn lock_eprom(&self, r: &Request, id: u8) -> Result<()> {
        self.write_u8(r, id, self.lock, 1)
    }
    pub fn unlock_eprom(&self, r: &Request, id: u8) -> Result<()> {
        self.write_u8(r, id, self.lock, 0)
    }
}
