use crate::{DeviceError, Error, Result};

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
    if !valid_id(id, true) || params.len() > MAX_PACKET_SIZE - 6 {
        return Err(Error::InvalidArgument("ID or packet size"));
    }
    let mut p = vec![255, 255, id, (params.len() + 2) as u8, instruction];
    p.extend_from_slice(params);
    let sum = p[2..].iter().fold(0u8, |s, v| s.wrapping_add(*v));
    p.push(!sum);
    Ok(p)
}
pub fn decode_status(p: &[u8]) -> Result<Status> {
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
    Ok(Status {
        id: p[2],
        flags: p[4],
        data: p[5..p.len() - 1].to_vec(),
    })
}
