use std::{error, fmt, io};

pub type Result<T> = std::result::Result<T, Error>;

/// All device status bits are retained, including unknown bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceError {
    pub id: u8,
    pub flags: u8,
}
impl DeviceError {
    pub const VOLTAGE: u8 = 1;
    pub const ANGLE: u8 = 2;
    pub const OVERHEAT: u8 = 4;
    pub const OVER_CURRENT: u8 = 8;
    pub const OVERLOAD: u8 = 32;
    pub fn has(self, flag: u8) -> bool {
        self.flags & flag != 0
    }
}
impl fmt::Display for DeviceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "device {} status 0x{:02X}", self.id, self.flags)
    }
}
impl error::Error for DeviceError {}

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    InvalidArgument(&'static str),
    /// Some bytes arrived but did not form a complete packet when partial=true.
    Timeout {
        partial: bool,
    },
    Cancelled,
    DeadlineExceeded,
    Checksum,
    Packet(&'static str),
    Device(DeviceError),
    Sample {
        id: u8,
        source: Box<Error>,
    },
    Io(io::Error),
    Closed,
    Poisoned,
    Unsupported(&'static str),
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidArgument(s) => write!(f, "invalid argument: {s}"),
            Self::Timeout { partial } => write!(f, "response timeout (partial packet: {partial})"),
            Self::Cancelled => f.write_str("request cancelled"),
            Self::DeadlineExceeded => f.write_str("request deadline exceeded"),
            Self::Checksum => f.write_str("packet checksum mismatch"),
            Self::Packet(s) => write!(f, "invalid packet: {s}"),
            Self::Device(e) => e.fmt(f),
            Self::Sample { id, source } => write!(f, "device {id} sample: {source}"),
            Self::Io(e) => write!(f, "transport: {e}"),
            Self::Closed => f.write_str("bus closed"),
            Self::Poisoned => f.write_str("bus mutex poisoned"),
            Self::Unsupported(s) => write!(f, "unsupported operation: {s}"),
        }
    }
}
impl error::Error for Error {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            Self::Device(e) => Some(e),
            Self::Sample { source, .. } => Some(source.as_ref()),
            _ => None,
        }
    }
}
impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}
