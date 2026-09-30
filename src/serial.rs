use crate::{Bus, Error, Result, Transport};
use std::{
    io::{self, Read, Write},
    time::Duration,
};

struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    write_timeout: Duration,
}
impl Read for SerialTransport {
    fn read(&mut self, p: &mut [u8]) -> io::Result<usize> {
        self.port.read(p)
    }
}
impl Write for SerialTransport {
    fn write(&mut self, p: &[u8]) -> io::Result<usize> {
        // serialport::set_timeout affects BOTH reads and writes on Windows.
        // Restore the configured write budget after short cancellation polls.
        self.port
            .set_timeout(self.write_timeout)
            .map_err(io::Error::other)?;
        self.port.write(p)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.port.flush()
    }
}
impl Transport for SerialTransport {
    fn set_read_timeout(&mut self, t: Duration) -> io::Result<()> {
        self.port.set_timeout(t).map_err(io::Error::other)
    }
    fn clear_input(&mut self) -> io::Result<()> {
        self.port
            .clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)
    }
}
impl Bus {
    /// Open an 8N1, no-flow-control port. baud=0 selects 1M. Requires an adapter
    /// handling half-duplex direction automatically; software RTS is not toggled.
    pub fn open(name: &str, baud: u32, timeout: Duration) -> Result<Self> {
        if name.is_empty() || std::time::Instant::now().checked_add(timeout).is_none() {
            return Err(Error::InvalidArgument("port name or timeout"));
        }
        let write_timeout = if timeout.is_zero() {
            Duration::from_millis(100)
        } else {
            timeout
        }
        .max(Duration::from_millis(1));
        let p = serialport::new(name, if baud == 0 { 1_000_000 } else { baud })
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(write_timeout)
            .open()
            .map_err(io::Error::other)?;
        Self::new(
            SerialTransport {
                port: p,
                write_timeout,
            },
            timeout,
        )
    }
}
