use crate::{protocol::*, Error, Result};
use std::{
    collections::{BTreeMap, BTreeSet},
    io::{self, Read, Write},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex, MutexGuard, TryLockError,
    },
    thread,
    time::{Duration, Instant},
};

/// Exclusive transport owned by a Bus. Reads must obey set_read_timeout.
/// Writes/clear_input must eventually return. Drop releases the physical port.
pub trait Transport: Read + Write + Send {
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()>;
    fn clear_input(&mut self) -> io::Result<()>;
}

/// Cooperative cancellation and optional absolute deadline, including lock wait.
/// Clones share cancellation. A cancelled request cannot be reset.
#[derive(Debug, Clone, Default)]
pub struct Request {
    cancelled: Arc<AtomicBool>,
    deadline: Option<Instant>,
}
impl Request {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn with_deadline(deadline: Instant) -> Self {
        Self {
            deadline: Some(deadline),
            ..Self::default()
        }
    }
    pub fn with_timeout(timeout: Duration) -> Result<Self> {
        Instant::now()
            .checked_add(timeout)
            .map(Self::with_deadline)
            .ok_or(Error::InvalidArgument("request timeout too large"))
    }
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
    }
    pub(crate) fn check(&self) -> Result<()> {
        if self.cancelled.load(Ordering::Acquire) {
            return Err(Error::Cancelled);
        }
        if self.deadline.is_some_and(|d| Instant::now() >= d) {
            return Err(Error::DeadlineExceeded);
        }
        Ok(())
    }
}

/// Partial results from this transaction only. Device faults do not discard other
/// replies. `missing` contains IDs with no validated response, not faulty samples.
#[derive(Debug)]
#[must_use]
pub struct Batch<T> {
    pub values: BTreeMap<u8, T>,
    pub errors: Vec<Error>,
    pub missing: Vec<u8>,
}
impl<T> Batch<T> {
    pub fn is_ok(&self) -> bool {
        self.errors.is_empty() && self.missing.is_empty()
    }
}
impl Batch<Status> {
    pub(crate) fn decode<T>(self, decode: impl Fn(&[u8]) -> Result<T>) -> Batch<T> {
        let mut out = Batch {
            values: BTreeMap::new(),
            errors: self.errors,
            missing: self.missing,
        };
        for (id, status) in self.values {
            if status.flags != 0 {
                continue;
            }
            match decode(&status.data) {
                Ok(v) => {
                    out.values.insert(id, v);
                }
                Err(e) => out.errors.push(Error::Sample {
                    id,
                    source: Box::new(e),
                }),
            }
        }
        out
    }
}

struct Inner {
    port: Option<Box<dyn Transport>>,
}
/// Clone this handle to share one physical bus. Complete transactions are locked.
#[derive(Clone)]
pub struct Bus {
    inner: Arc<Mutex<Inner>>,
    timeout: Duration,
}
impl Bus {
    /// Zero selects a 100ms total response timeout (all SyncRead replies together).
    pub fn new(port: impl Transport + 'static, timeout: Duration) -> Result<Self> {
        let timeout = if timeout.is_zero() {
            Duration::from_millis(100)
        } else {
            timeout
        };
        if Instant::now().checked_add(timeout).is_none() {
            return Err(Error::InvalidArgument("bus timeout too large"));
        }
        Ok(Self {
            inner: Arc::new(Mutex::new(Inner {
                port: Some(Box::new(port)),
            })),
            timeout,
        })
    }
    fn acquire(&self, request: &Request) -> Result<MutexGuard<'_, Inner>> {
        loop {
            request.check()?;
            match self.inner.try_lock() {
                Ok(g) => {
                    request.check()?;
                    if g.port.is_none() {
                        return Err(Error::Closed);
                    }
                    return Ok(g);
                }
                Err(TryLockError::WouldBlock) => thread::sleep(Duration::from_millis(1)),
                Err(TryLockError::Poisoned(_)) => return Err(Error::Poisoned),
            }
        }
    }
    /// Closes all clones after the current transaction. Dropping the last Bus also
    /// releases the port; this method does not send a motion/torque command.
    pub fn close(&self) -> Result<()> {
        let mut g = self.inner.lock().map_err(|_| Error::Poisoned)?;
        g.port.take();
        Ok(())
    }
    fn send(port: &mut dyn Transport, request: &Request, mut packet: &[u8]) -> Result<()> {
        request.check()?;
        port.clear_input()?;
        while !packet.is_empty() {
            request.check()?;
            match port.write(packet) {
                Ok(0) => return Err(io::Error::from(io::ErrorKind::WriteZero).into()),
                Ok(n) if n <= packet.len() => packet = &packet[n..],
                Ok(_) => return Err(Error::Packet("transport returned invalid write count")),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }
    pub(crate) fn command(
        &self,
        request: &Request,
        id: u8,
        inst: u8,
        params: &[u8],
        expected: Option<usize>,
    ) -> Result<Status> {
        let packet = encode_packet(id, inst, params)?;
        let mut g = self.acquire(request)?;
        let port = g.port.as_deref_mut().ok_or(Error::Closed)?;
        Self::send(port, request, &packet)?;
        let Some(expected) = expected.filter(|_| id != BROADCAST_ID) else {
            return Ok(Status {
                id,
                flags: 0,
                data: vec![],
            });
        };
        let mut reader = Receiver::new(port, self.timeout, request)?;
        loop {
            let status = reader.next(request)?;
            if status.id != id {
                continue;
            }
            if let Some(e) = status.device_error() {
                return Err(Error::Device(e));
            }
            if status.data.len() != expected {
                return Err(Error::Packet("unexpected response data length"));
            }
            return Ok(status);
        }
    }
    pub fn ping(&self, r: &Request, id: u8) -> Result<()> {
        if !valid_id(id, false) {
            return Err(Error::InvalidArgument("unicast ID"));
        }
        self.command(r, id, PING, &[], Some(0)).map(|_| ())
    }
    pub fn read(&self, r: &Request, id: u8, address: u8, length: usize) -> Result<Vec<u8>> {
        if !valid_id(id, false) {
            return Err(Error::InvalidArgument("unicast ID"));
        }
        register_range(address, length, MAX_PACKET_SIZE - 6)?;
        Ok(self
            .command(r, id, READ, &[address, length as u8], Some(length))?
            .data)
    }
    pub(crate) fn write_command(
        &self,
        r: &Request,
        id: u8,
        address: u8,
        data: &[u8],
        inst: u8,
        reply: bool,
    ) -> Result<()> {
        register_range(address, data.len(), MAX_PACKET_SIZE - 7)?;
        let mut p = vec![address];
        p.extend_from_slice(data);
        self.command(r, id, inst, &p, reply.then_some(0))
            .map(|_| ())
    }
    pub fn write(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.write_command(r, id, address, data, WRITE, true)
    }
    pub fn write_only(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.write_command(r, id, address, data, WRITE, false)
    }
    pub fn reg_write(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.write_command(r, id, address, data, REG_WRITE, true)
    }
    pub fn reg_write_only(&self, r: &Request, id: u8, address: u8, data: &[u8]) -> Result<()> {
        self.write_command(r, id, address, data, REG_WRITE, false)
    }
    pub fn action(&self, r: &Request, id: u8) -> Result<()> {
        self.command(r, id, ACTION, &[], Some(0)).map(|_| ())
    }
    pub fn sync_write(
        &self,
        r: &Request,
        address: u8,
        length: usize,
        entries: &[WriteEntry],
    ) -> Result<()> {
        register_range(address, length, MAX_PACKET_SIZE - 9)?;
        if entries.is_empty() || entries.len() > (MAX_PACKET_SIZE - 8) / (length + 1) {
            return Err(Error::InvalidArgument("sync write size"));
        }
        let mut seen = BTreeSet::new();
        let mut p = vec![address, length as u8];
        for e in entries {
            if !valid_id(e.id, false) || !seen.insert(e.id) || e.data.len() != length {
                return Err(Error::InvalidArgument("sync write ID or data length"));
            }
            p.push(e.id);
            p.extend_from_slice(&e.data);
        }
        self.command(r, BROADCAST_ID, SYNC_WRITE, &p, None)
            .map(|_| ())
    }
    /// Outer Err: invalid request, unavailable bus, or send failure. Once sent,
    /// reception errors are in Batch.errors so validated partial data is retained.
    pub fn sync_read(
        &self,
        r: &Request,
        address: u8,
        length: usize,
        ids: &[u8],
    ) -> Result<Batch<Status>> {
        register_range(address, length, MAX_PACKET_SIZE - 6)?;
        if ids.is_empty() || ids.len() > MAX_PACKET_SIZE - 8 {
            return Err(Error::InvalidArgument("sync read size"));
        }
        let mut seen = BTreeSet::new();
        for &id in ids {
            if !valid_id(id, false) || !seen.insert(id) {
                return Err(Error::InvalidArgument("sync read ID"));
            }
        }
        let mut p = vec![address, length as u8];
        p.extend_from_slice(ids);
        let packet = encode_packet(BROADCAST_ID, SYNC_READ, &p)?;
        let mut g = self.acquire(r)?;
        let port = g.port.as_deref_mut().ok_or(Error::Closed)?;
        Self::send(port, r, &packet)?;
        let mut out = Batch {
            values: BTreeMap::new(),
            errors: vec![],
            missing: vec![],
        };
        let mut reader = Receiver::new(port, self.timeout, r)?;
        while out.values.len() < ids.len() {
            let s = match reader.next(r) {
                Ok(s) => s,
                Err(e) => {
                    out.errors.push(e);
                    break;
                }
            };
            if !seen.contains(&s.id) || out.values.contains_key(&s.id) {
                continue;
            }
            if let Some(e) = s.device_error() {
                out.errors.push(Error::Device(e));
            }
            if s.data.len() != length {
                out.errors.push(Error::Packet("sync response length"));
                break;
            }
            out.values.insert(s.id, s);
        }
        out.missing = ids
            .iter()
            .copied()
            .filter(|id| !out.values.contains_key(id))
            .collect();
        Ok(out)
    }
}

#[derive(Debug, Clone)]
pub struct WriteEntry {
    pub id: u8,
    pub data: Vec<u8>,
}

struct Receiver<'a> {
    port: &'a mut dyn Transport,
    deadline: Instant,
    pending: Vec<u8>,
}
impl<'a> Receiver<'a> {
    fn new(port: &'a mut dyn Transport, timeout: Duration, r: &Request) -> Result<Self> {
        let d = Instant::now()
            .checked_add(timeout)
            .ok_or(Error::InvalidArgument("timeout too large"))?;
        let deadline = r.deadline.map_or(d, |rd| rd.min(d));
        Ok(Self {
            port,
            deadline,
            pending: Vec::new(),
        })
    }
    fn next(&mut self, r: &Request) -> Result<Status> {
        loop {
            r.check()?;
            if Instant::now() >= self.deadline {
                return Err(Error::Timeout {
                    partial: !self.pending.is_empty(),
                });
            }
            while self.pending.len() >= 2 {
                let Some(i) = self.pending.windows(2).position(|v| v == [255, 255]) else {
                    self.pending.drain(..self.pending.len() - 1);
                    break;
                };
                self.pending.drain(..i);
                if self.pending.len() < 4 {
                    break;
                }
                let size = usize::from(self.pending[3]) + 4;
                if !valid_id(self.pending[2], false) || !(6..=MAX_PACKET_SIZE).contains(&size) {
                    self.pending.remove(0);
                    continue;
                }
                if self.pending.len() < size {
                    break;
                }
                let s = decode_status(&self.pending[..size]);
                self.pending.drain(..size);
                return s;
            }
            let wait = self
                .deadline
                .saturating_duration_since(Instant::now())
                .clamp(Duration::from_millis(1), Duration::from_millis(5));
            self.port.set_read_timeout(wait)?;
            let mut chunk = [0u8; 256];
            match self.port.read(&mut chunk) {
                Ok(0) => thread::sleep(Duration::from_millis(1)),
                Ok(n) if n <= chunk.len() => self.pending.extend_from_slice(&chunk[..n]),
                Ok(_) => return Err(Error::Packet("transport returned invalid read count")),
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                    ) =>
                {
                    thread::sleep(Duration::from_millis(1))
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.into()),
            }
        }
    }
}
