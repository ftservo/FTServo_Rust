use crate::{protocol::*, Error, Result};
use std::{
    io::{self, Read, Write},
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex, MutexGuard, TryLockError,
    },
    thread,
    time::{Duration, Instant},
};

/// A `Duration` as whole nanoseconds, saturating rather than wrapping.
fn nanos(d: Duration) -> u64 {
    d.as_nanos().min(u128::from(u64::MAX)) as u64
}

/// `now + budget`, falling back to "now" if that would overflow.
fn deadline_in(budget: Duration) -> Instant {
    Instant::now()
        .checked_add(budget)
        .unwrap_or_else(Instant::now)
}

/// Exclusive transport owned by a Bus. Reads must obey set_read_timeout.
/// Writes/clear_input must eventually return. Drop releases the physical port.
pub trait Transport: Read + Write + Send {
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()>;
    fn clear_input(&mut self) -> io::Result<()>;
}

/// Cooperative cancellation, including lock wait. Clones share cancellation, and a
/// cancelled request cannot be reset.
///
/// There is no deadline to set here. A read's time limit comes from the protocol instead —
/// see [`Bus::command`] — and it covers the whole transaction, lock wait included, so one
/// `Request` can live across a poll loop as it is.
#[derive(Debug, Clone, Default)]
pub struct Request {
    cancelled: Arc<AtomicBool>,
}
impl Request {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
    }
    pub(crate) fn check(&self) -> Result<()> {
        if self.cancelled.load(Ordering::Acquire) {
            return Err(Error::Cancelled);
        }
        Ok(())
    }
}

/// Partial results from this transaction only. Device faults do not discard other
/// replies. `missing` contains IDs with no validated response, not faulty samples.
///
/// `values` is in **request order**, not sorted by id: `get` is a linear scan, which at
/// the sizes this bus sees (`ids.len() <= 242`) beats a tree lookup — and, the reason it
/// is a `Vec` at all, allocates nothing per element.
///
/// **Reuse contract.** A `Batch` is meant to be held by the caller and reset with
/// [`Batch::clear`] between rounds. `clear` keeps the heap capacity of every container, so
/// a loop that clears and refills the same `Batch` allocates nothing from its second round
/// on. That is what makes a 50 Hz poll loop flat, instead of a stream of small allocations
/// sitting on the I/O path.
#[derive(Debug)]
pub struct Batch<T> {
    /// `(id, value)` in request order.
    pub values: Vec<(u8, T)>,
    /// Device faults and per-sample decode errors.
    pub errors: Vec<Error>,
    /// IDs with no validated response, in request order.
    pub missing: Vec<u8>,
    /// `(id, flags)` of the frames this transaction kept, in request order.
    ///
    /// A typed reader (`Feedback`, or whatever an application's own decoder produces) carries
    /// no status code of its own, so without this a caller could not read the code of a
    /// healthy answer at all. Which frames
    /// land here follows which frames became values, and that differs between the two read
    /// paths — see [`Batch::flags`]. The value is the reply's raw `ERR` byte, normally 0.
    pub status: Vec<(u8, u8)>,
    /// [`Status`]-path only: payload buffers recycled from the previous round.
    ///
    /// Clearing `values` drops each `Status`, which would free its `data` — so the buffers
    /// are moved aside first and handed back out on the next fill. Without this the raw
    /// `Status` path would still allocate once per device per round, which is exactly the
    /// cost the typed path avoids by never building a `Status` at all.
    pub(crate) spare: Vec<Vec<u8>>,
}
impl<T> Default for Batch<T> {
    fn default() -> Self {
        Self::new()
    }
}
impl<T> Batch<T> {
    /// An empty batch. Allocate one outside the loop and pass it in.
    pub fn new() -> Self {
        Self {
            values: Vec::new(),
            errors: Vec::new(),
            missing: Vec::new(),
            status: Vec::new(),
            spare: Vec::new(),
        }
    }
    /// Empty every container, keeping their heap capacity.
    ///
    /// This is the whole reuse story: call it at the top of each round. A `Status` batch
    /// goes through [`Bus::sync_read`], which recycles the payload buffers first so the raw
    /// path does not re-allocate them either.
    pub fn clear(&mut self) {
        self.values.clear();
        self.errors.clear();
        self.missing.clear();
        self.status.clear();
    }
    pub fn is_ok(&self) -> bool {
        self.errors.is_empty() && self.missing.is_empty()
    }
    /// The value for `id`, if this transaction produced one. Linear in `values.len()`.
    pub fn get(&self, id: u8) -> Option<&T> {
        self.values.iter().find(|(v, _)| *v == id).map(|(_, v)| v)
    }
    /// The status code of `id`'s reply, if this transaction recorded one for it. Linear in
    /// `status.len()`.
    ///
    /// **What `None` means depends on which read path filled the batch**, because the two
    /// disagree on whether a faulting reply counts as a result.
    ///
    /// *Typed path* ([`Bus::sync_read_decoded`] — what the readers with their own payload
    /// types take): only a frame that decoded carries a code, so a faulting reply is in
    /// [`Batch::errors`] instead and `None` here means the device either did not answer (see
    /// [`Batch::missing`]) **or** answered with a fault.
    ///
    /// *Raw path* ([`Bus::sync_read`]): a faulting reply is still kept as a value, so it has
    /// a code here too. `None` then means only that the device did not answer.
    pub fn flags(&self, id: u8) -> Option<u8> {
        self.status.iter().find(|(v, _)| *v == id).map(|(_, f)| *f)
    }
    /// How many devices produced a value.
    pub fn len(&self) -> usize {
        self.values.len()
    }
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
}
impl Batch<Status> {
    /// Start a fresh fill: clear the result, recycling last round's payload buffers.
    ///
    /// `keep` caps the pool at this request's device count, so a bus that once answered
    /// with large payloads does not hold that memory forever.
    fn begin_fill(&mut self, keep: usize) {
        self.errors.clear();
        self.missing.clear();
        self.status.clear();
        self.spare.truncate(keep);
        for (_, status) in std::mem::take(&mut self.values) {
            if self.spare.len() < keep {
                self.spare.push(status.data);
            }
        }
    }
    /// Append one reply, taking a pooled buffer for its payload.
    fn push_reply(&mut self, id: u8, flags: u8, data: &[u8]) {
        let mut buf = self.spare.pop().unwrap_or_default();
        buf.clear();
        buf.extend_from_slice(data);
        // `status` mirrors `values` *on this path*: any answer kept as a value also has its
        // code queryable by id. The decoded path is stricter — it records only frames that
        // decoded — so `flags` does not read identically across the two; see [`Batch::flags`].
        self.status.push((id, flags));
        self.values.push((
            id,
            Status {
                id,
                flags,
                data: buf,
            },
        ));
    }
}

/// How long one byte occupies the wire: 1 start bit + 8 data bits + 1 stop bit.
///
/// This is what turns "read n bytes" into a duration, so it follows the **host-side** baud
/// rate rather than being a constant — 10 µs at 1 Mbaud, 87 µs at 115 200. Zero means
/// 1 Mbaud, the spelling [`Bus::open`] accepts.
pub(crate) fn byte_time(baud: u32) -> Duration {
    let baud = if baud == 0 { 1_000_000 } else { baud };
    Duration::from_nanos(10_000_000_000 / u64::from(baud))
}

/// The fixed part of a read budget: reply latency plus the driver's delivery batching,
/// **merged into one number on purpose**.
///
/// The two cannot be told apart. Both happen between "last request byte sent" and "first
/// reply byte readable", and each carries more uncertainty than the gap between them —
/// the device's `Return Delay Time` is a register this SDK does not read, and a USB
/// adapter hands bytes over on its own 1 ms schedule. Keeping them as two constants would
/// suggest two knobs where there is only one measurement, so there is one.
///
/// It is the **base** of every read window; the byte term is added on top. See
/// [`ReadTiming::window`].
///
/// Erring large is free while devices answer: a read returns as soon as bytes arrive, so a
/// generous base costs time only when a device really is silent.
///
/// **20 ms rather than a tighter guess, because the tail is real.** On a 1 Mbaud USB adapter,
/// 1.5 ms lost roughly 5% of reads to `Timeout { partial: false }` (33 in 632 rounds) —
/// replies that had not started yet, with no checksum error and no recovery needed on the next
/// round — 5 ms still lost 1 in 449, and 20 ms lost none in 900. The batch those numbers
/// measure (device `Return Delay Time` plus the driver's delivery schedule) is a property of
/// the setup, so the base carries the margin and a tighter one is available per bus, see
/// [`Bus::set_read_delay`]. The cost is paid only where there is nothing to read: a scan across
/// 253 IDs, most of them silent, takes seconds rather than fractions of one.
pub const DEFAULT_READ_DELAY: Duration = Duration::from_millis(20);

/// The delay to hold around an EEPROM write.
///
/// Writing an EEPROM register costs the device an erase/write cycle, and it does not answer
/// the bus while that cycle runs — orders of magnitude longer than reading a register or
/// writing RAM. Hold this with [`Bus::with_read_delay`] for the duration.
///
/// It has to stay **above** [`DEFAULT_READ_DELAY`], which is what holding it is for; raising
/// the default without raising this one would turn the guard below into a no-op.
///
///
/// ```no_run
/// # use ftservo::{Bus, EEPROM_READ_DELAY};
/// # fn f(bus: &Bus, r: &ftservo::Request) -> ftservo::Result<()> {
/// {
///     let _slow = bus.with_read_delay(EEPROM_READ_DELAY);
///     bus.write(r, 1, 6, &[1])?;          // an EEPROM register
/// }
/// // restored, including on an early `?` return or a panic
/// # Ok(())
/// # }
/// ```
pub const EEPROM_READ_DELAY: Duration = Duration::from_millis(50);

/// One read's allowance: `delay + bytes_in_hand × byte_time`.
///
/// Composed by [`Bus`] from its baud rate and the delay currently in force — the latter is
/// mutable so an EEPROM write can raise it, see [`Bus::set_read_delay`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct ReadTiming {
    delay: Duration,
    byte_time: Duration,
}
impl ReadTiming {
    /// How long to wait for the next read, given how much is already in hand.
    ///
    /// The window follows the bytes that have arrived rather than a predicted total. A device
    /// already committing `n` bytes to the wire is given `n` bytes' worth of time to commit
    /// the next one, on top of the fixed allowance. Before anything arrives `n` is zero and
    /// the window is just the delay — which is what waiting for a reply that has not started
    /// needs.
    fn window(self, in_hand: usize) -> Duration {
        self.delay + self.byte_time * in_hand as u32
    }
}

struct Inner {
    port: Option<Box<dyn Transport>>,
    /// Outbound packet buffer, kept across calls so encoding stops allocating after the
    /// first round. See [`encode_packet_into`].
    tx: Vec<u8>,
    /// Bytes accumulated from devices, kept for the same reason and cleared at the start
    /// of every transaction so a late reply cannot be read as this one's.
    rx: Vec<u8>,
}
/// Clone this handle to share one physical bus. Complete transactions are locked.
#[derive(Clone)]
pub struct Bus {
    inner: Arc<Mutex<Inner>>,
    /// What a device is given before it counts as silent, normally [`DEFAULT_READ_DELAY`],
    /// in whole nanoseconds.
    ///
    /// Deliberately **outside** `inner`: a transaction's budget has to be known *before*
    /// waiting for that lock, so everything the budget needs must be readable without it.
    /// Reading it through the transaction lock would deadlock — the holder may be parked
    /// in a `read` for the whole budget, which is exactly when contention matters.
    ///
    /// Shared by every clone, which is what makes a shared port behave consistently.
    read_delay: Arc<AtomicU64>,
    /// One byte on the wire at this bus's rate — see [`byte_time`]. Immutable: it follows
    /// the port, not the operation.
    byte_time: Duration,
}
impl Bus {
    /// Create a handle over `port`, assuming a 1 Mbaud link.
    ///
    /// Assumes 1 Mbaud, which is what every device in this SDK's target families ships at.
    /// A port driven at another rate wants [`Self::with_baud`].
    ///
    /// `timeout` is **retired from the read path**: how long a reply may take is computed
    /// from the protocol instead, see [`Self::command`]. The argument is kept so existing
    /// call sites keep compiling, and is ignored here; [`Self::open`] is the constructor
    /// that still has a use for it, in the write direction.
    pub fn new(port: impl Transport + 'static, timeout: Duration) -> Result<Self> {
        Self::with_baud(port, 1_000_000, timeout)
    }
    /// [`Self::new`], with the host-side baud rate spelled out.
    ///
    /// The rate does not configure the port — the transport already has one — it fixes how
    /// long a byte is taken to occupy the wire, and therefore what a reply of a given length
    /// is allowed to take. Zero means 1 Mbaud, matching [`Self::open`].
    ///
    /// `timeout` is ignored, for the same reason as in [`Self::new`].
    pub fn with_baud(port: impl Transport + 'static, baud: u32, timeout: Duration) -> Result<Self> {
        let _ = timeout;
        Ok(Self {
            inner: Arc::new(Mutex::new(Inner {
                port: Some(Box::new(port)),
                tx: Vec::new(),
                rx: Vec::new(),
            })),
            read_delay: Arc::new(AtomicU64::new(nanos(DEFAULT_READ_DELAY))),
            byte_time: byte_time(baud),
        })
    }
    /// What a device is currently given before it counts as silent.
    pub fn read_delay(&self) -> Duration {
        Duration::from_nanos(self.read_delay.load(Ordering::Relaxed))
    }
    /// Set what a device is given before it counts as silent.
    ///
    /// Raise it around an EEPROM write — see [`EEPROM_READ_DELAY`] — and lower it again
    /// afterwards. [`Self::with_read_delay`] does both around a scope and is the better
    /// default: a delay left raised makes every later read wait it out, which on a poll
    /// loop is a stalled loop rather than a slow one.
    ///
    /// Visible to every clone of this bus, and in force from the next transaction on.
    pub fn set_read_delay(&self, delay: Duration) {
        self.read_delay.store(nanos(delay), Ordering::Relaxed);
    }
    /// Raise the device delay for as long as the returned guard lives, then restore what
    /// was in force before.
    ///
    /// Restoring the previous value rather than a fixed default is what makes nesting
    /// behave: an inner scope dropping does not clear an outer scope's setting.
    pub fn with_read_delay(&self, delay: Duration) -> ReadDelayGuard<'_> {
        let restore = Duration::from_nanos(self.read_delay.swap(nanos(delay), Ordering::Relaxed));
        ReadDelayGuard { bus: self, restore }
    }
    /// Snapshot the read timing for a transaction.
    ///
    /// Taken **before** the lock, and without taking it: the allowance has to be known before
    /// waiting for the transaction lock, so nothing it needs may live behind that lock.
    fn timing(&self) -> ReadTiming {
        ReadTiming {
            delay: Duration::from_nanos(self.read_delay.load(Ordering::Relaxed)),
            byte_time: self.byte_time,
        }
    }
    /// Take the bus, giving up at `deadline`.
    ///
    /// The deadline is the transaction's own read budget, so contention cannot eat the time
    /// the reply needs — and since a transaction is milliseconds at most, the holder is
    /// usually about to finish. A flat `sleep(1ms)` before each retry would add that
    /// millisecond to every contention — 5% of a 50 Hz period — so spin briefly first and
    /// only then hand the CPU over.
    ///
    /// Without this bound the loop would not end: cancelling the request is the only other
    /// way out, and that is the caller's decision rather than a time limit.
    fn acquire(&self, request: &Request, deadline: Instant) -> Result<MutexGuard<'_, Inner>> {
        let mut spins = 0u32;
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
                Err(TryLockError::WouldBlock) => {
                    if Instant::now() >= deadline {
                        // A cancellation landing at the same moment is the more specific
                        // answer, so ask before blaming the clock.
                        request.check()?;
                        return Err(Error::Timeout { partial: false });
                    }
                    // Wrapping: this counter only paces the hand-over, and the `deadline`
                    // above is what actually ends the loop.
                    spins = spins.wrapping_add(1);
                    if spins % 64 == 0 {
                        thread::yield_now();
                    } else {
                        std::hint::spin_loop();
                    }
                }
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
    /// Run one transaction and return the addressed device's answer.
    ///
    /// **Low-level.** `inst` is a raw protocol instruction byte and `expected` is how many
    /// payload bytes the reply should carry — `None` for broadcasts, which are never
    /// answered. [`Self::ping`], [`Self::read`], [`Self::write`] and friends fill both in for
    /// their own instruction; reach for this directly when you want the reply's status code,
    /// which those wrappers discard.
    ///
    /// **The read window is computed, not configured:** every read is allowed `read_delay`
    /// plus the time the bytes already in hand would have taken — see [`ReadTiming::window`].
    /// The wait for the lock is bounded by one frame's worth of the same thing, see
    /// [`Self::acquire`].
    ///
    /// A reply from a *different* device is [`Error::UnexpectedId`]: a unicast transaction
    /// has exactly one legitimate answerer, so anything else means the bus is out of step.
    pub fn command(
        &self,
        request: &Request,
        id: u8,
        inst: u8,
        params: &[u8],
        expected: Option<usize>,
    ) -> Result<Status> {
        // Bounded here rather than trusted: this is a public entry point, and an out-of-range
        // `expected` would overflow the budget arithmetic and describe a reply the protocol
        // cannot carry anyway. The wrappers all pass through `register_range`; this is the
        // same ceiling, for callers who come here directly.
        if expected.is_some_and(|n| n > MAX_PACKET_SIZE - 6) {
            return Err(Error::InvalidArgument("expected reply length"));
        }
        let frame_len = expected.unwrap_or(0) + 6;
        let timing = self.timing();
        // The lock wait is sized by one frame: whoever holds the bus is mid-transaction, so
        // that is the longest they can plausibly still be busy.
        let mut g = self.acquire(request, deadline_in(timing.window(frame_len)))?;
        let Inner { port, tx, rx, .. } = &mut *g;
        let port = port.as_deref_mut().ok_or(Error::Closed)?;
        // Drop last round's leftovers before anything is written, so a late reply cannot be
        // mistaken for this one's. `send` does the same for the driver's own buffer.
        rx.clear();
        encode_packet_into(tx, id, inst, params)?;
        Self::send(port, request, tx)?;
        let Some(expected) = expected.filter(|_| id != BROADCAST_ID) else {
            return Ok(Status {
                id,
                flags: 0,
                data: Vec::new(),
            });
        };
        let mut reader = Receiver::new(port, rx, timing, frame_len);
        let frame = reader.next(request)?;
        if frame.id != id {
            return Err(Error::UnexpectedId {
                expected: id,
                got: frame.id,
            });
        }
        if frame.len != expected {
            return Err(Error::Packet("unexpected response data length"));
        }
        Ok(Status {
            id,
            flags: frame.flags,
            data: reader.payload(frame).to_vec(),
        })
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
        let mut seen = [false; 256];
        let mut p = vec![address, length as u8];
        for e in entries {
            if !valid_id(e.id, false) || seen[usize::from(e.id)] || e.data.len() != length {
                return Err(Error::InvalidArgument("sync write ID or data length"));
            }
            seen[usize::from(e.id)] = true;
            p.push(e.id);
            p.extend_from_slice(&e.data);
        }
        self.command(r, BROADCAST_ID, SYNC_WRITE, &p, None)
            .map(|_| ())
    }
    /// Validate a sync-read request, recording membership in `seen`.
    ///
    /// Both read paths need the same three checks and then the same membership test while
    /// receiving, so the array filled in here is handed to the receive loop rather than
    /// rebuilt there.
    fn check_sync_read(
        address: u8,
        length: usize,
        ids: &[u8],
        seen: &mut [bool; 256],
    ) -> Result<()> {
        register_range(address, length, MAX_PACKET_SIZE - 6)?;
        if ids.is_empty() || ids.len() > MAX_PACKET_SIZE - 8 {
            return Err(Error::InvalidArgument("sync read size"));
        }
        for &id in ids {
            if !valid_id(id, false) || seen[usize::from(id)] {
                return Err(Error::InvalidArgument("sync read ID"));
            }
            seen[usize::from(id)] = true;
        }
        Ok(())
    }
    /// Synchronous read of one register block from several devices, in one broadcast
    /// transaction. Results are written into `out`, which the caller keeps across rounds.
    ///
    /// **Reuse.** `out` is cleared and refilled with its heap capacity intact — and, for
    /// the `Status` payloads, with the previous round's buffers recycled through the
    /// internal pool. From the second round on a loop calling this allocates nothing.
    ///
    /// Outer `Err`: invalid request, unavailable bus, or send failure. Once sent,
    /// reception errors land in `out.errors` so validated partial data is retained;
    /// `out.values` and `out.missing` say which devices answered.
    pub fn sync_read(
        &self,
        r: &Request,
        address: u8,
        length: usize,
        ids: &[u8],
        out: &mut Batch<Status>,
    ) -> Result<()> {
        let mut seen = [false; 256];
        Self::check_sync_read(address, length, ids, &mut seen)?;
        out.begin_fill(ids.len());

        // Each read gets its own window, sized by what is already in hand — see
        // `ReadTiming::window`. The wait for the lock is sized by one frame.
        let timing = self.timing();
        let frame_len = length + 6;
        let mut g = self.acquire(r, deadline_in(timing.window(frame_len)))?;
        let Inner { port, tx, rx, .. } = &mut *g;
        let port = port.as_deref_mut().ok_or(Error::Closed)?;
        // Before the write, not after: a reply left from the previous round must not be read
        // as this one's. `send` clears the driver's buffer for the same reason.
        rx.clear();
        encode_sync_read(tx, address, length, ids);
        Self::send(port, r, tx)?;

        // Timed from here rather than from before the send — see `command`.
        let mut reader = Receiver::new(port, rx, timing, frame_len);
        let mut accepted = [false; 256];
        let mut received = 0usize;
        while received < ids.len() {
            let frame = match reader.next(r) {
                Ok(f) => f,
                // A corrupt frame has already been dropped and the buffer resynchronised, so
                // this is not the end of the round: record it and keep waiting for the other
                // devices. Only a timeout or a transport error ends the round early.
                Err(e @ Error::Checksum) => {
                    out.errors.push(e);
                    continue;
                }
                Err(e) => {
                    out.errors.push(e);
                    break;
                }
            };
            if !seen[usize::from(frame.id)] || accepted[usize::from(frame.id)] {
                continue;
            }
            if frame.len != length {
                out.errors.push(Error::Packet("sync response length"));
                break;
            }
            accepted[usize::from(frame.id)] = true;
            received += 1;
            out.push_reply(frame.id, frame.flags, reader.payload(frame));
        }
        out.missing
            .extend(ids.iter().copied().filter(|id| !accepted[usize::from(*id)]));
        Ok(())
    }
    /// [`Self::sync_read`], with each payload handed to `decode` straight out of the
    /// receive buffer.
    ///
    /// No intermediate `Status` and no payload copy: this is the path a typed reader takes
    /// (`examples/imu/sync_read.rs` and the feedback block read in
    /// `examples/hls/sync_read.rs`), and the reason a
    /// 50 Hz loop over them allocates nothing after its first round — the bytes never
    /// leave the buffer the transport wrote them into.
    ///
    /// A reply carrying a device fault is decoded like any other: `ERR != 0` reports that the
    /// device is unhappy, not that the transaction failed, so it comes back beside the value
    /// it belongs to. Read it with [`Batch::flags`]; on the raw path it is [`Status::flags`].
    pub fn sync_read_decoded<T>(
        &self,
        r: &Request,
        address: u8,
        length: usize,
        ids: &[u8],
        decode: impl Fn(&[u8]) -> Result<T>,
        out: &mut Batch<T>,
    ) -> Result<()> {
        let mut seen = [false; 256];
        Self::check_sync_read(address, length, ids, &mut seen)?;
        out.clear();

        // Each read gets its own window — see `sync_read` and `ReadTiming::window`.
        let timing = self.timing();
        let frame_len = length + 6;
        let mut g = self.acquire(r, deadline_in(timing.window(frame_len)))?;
        let Inner { port, tx, rx, .. } = &mut *g;
        let port = port.as_deref_mut().ok_or(Error::Closed)?;
        // Before the write, not after: a reply left from the previous round must not be read
        // as this one's.
        rx.clear();
        encode_sync_read(tx, address, length, ids);
        Self::send(port, r, tx)?;

        // Timed from here rather than from before the send — see `command`.
        let mut reader = Receiver::new(port, rx, timing, frame_len);
        let mut accepted = [false; 256];
        let mut received = 0usize;
        while received < ids.len() {
            let frame = match reader.next(r) {
                Ok(f) => f,
                // A corrupt frame has already been dropped and the buffer resynchronised, so
                // this is not the end of the round: record it and keep waiting for the other
                // devices. Only a timeout or a transport error ends the round early.
                Err(e @ Error::Checksum) => {
                    out.errors.push(e);
                    continue;
                }
                Err(e) => {
                    out.errors.push(e);
                    break;
                }
            };
            if !seen[usize::from(frame.id)] || accepted[usize::from(frame.id)] {
                continue;
            }
            if frame.len != length {
                out.errors.push(Error::Packet("sync response length"));
                break;
            }
            accepted[usize::from(frame.id)] = true;
            received += 1;
            match decode(reader.payload(frame)) {
                Ok(v) => {
                    // Recorded beside the value it belongs to: only a frame that decoded gets
                    // a code here, so `status` never disagrees with `values`.
                    out.status.push((frame.id, frame.flags));
                    out.values.push((frame.id, v));
                }
                Err(e) => out.errors.push(Error::Sample {
                    id: frame.id,
                    source: Box::new(e),
                }),
            }
        }
        out.missing
            .extend(ids.iter().copied().filter(|id| !accepted[usize::from(*id)]));
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct WriteEntry {
    pub id: u8,
    pub data: Vec<u8>,
}

/// One status frame, located in the receive buffer rather than copied out of it.
///
/// Offsets instead of a slice is what lets [`Receiver::next`] be called again while the
/// caller still holds the previous frame: the borrow happens in [`Receiver::payload`] and
/// ends with the decode.
#[derive(Debug, Clone, Copy)]
struct Frame {
    id: u8,
    flags: u8,
    start: usize,
    len: usize,
}

/// Locates whole status frames in a transport's byte stream.
///
/// Nothing here decides how long a reply *may* take: every read gets a window derived from
/// what is already in hand — see [`ReadTiming::window`] — and frame boundaries come from the
/// bytes themselves. It collects bytes and finds frames.
struct Receiver<'a> {
    port: &'a mut dyn Transport,
    pending: &'a mut Vec<u8>,
    timing: ReadTiming,
    /// Total length of one frame (payload + 6): the most pieces it can be split into.
    frame_len: usize,
    /// Prefix already handed to the caller, reclaimed on the next call.
    consumed: usize,
}
impl<'a> Receiver<'a> {
    fn new(
        port: &'a mut dyn Transport,
        pending: &'a mut Vec<u8>,
        timing: ReadTiming,
        frame_len: usize,
    ) -> Self {
        Self {
            port,
            pending,
            timing,
            frame_len,
            consumed: 0,
        }
    }
    /// The payload of a frame returned by [`Self::next`].
    fn payload(&self, f: Frame) -> &[u8] {
        &self.pending[f.start..f.start + f.len]
    }
    /// One `read`, with a window that follows what is already in hand.
    ///
    /// A timeout is an error on every platform, and a zero-length read is a transport with
    /// nothing to give and no deadline of its own — both mean the same thing here. Returning
    /// rather than spinning on them is what keeps this loop off the CPU.
    fn read_once(&mut self) -> Result<()> {
        // A zero window would ask the transport to give up before it looked, so the floor is
        // one unit of time rather than none.
        let window = self.timing.window(self.pending.len());
        self.port
            .set_read_timeout(window.max(Duration::from_micros(1)))?;
        let mut chunk = [0u8; 256];
        match self.port.read(&mut chunk) {
            Ok(0) => Err(Error::Timeout {
                partial: !self.pending.is_empty(),
            }),
            Ok(k) if k <= chunk.len() => {
                self.pending.extend_from_slice(&chunk[..k]);
                Ok(())
            }
            Ok(_) => Err(Error::Packet("transport returned invalid read count")),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
            {
                Err(Error::Timeout {
                    partial: !self.pending.is_empty(),
                })
            }
            // Interrupted means "try again", and the caller's loop does exactly that.
            Err(e) if e.kind() == io::ErrorKind::Interrupted => Ok(()),
            Err(e) => Err(e.into()),
        }
    }
    /// Take one whole frame off the front of the buffer, if it is already there.
    ///
    /// `Ok(None)` means "need more bytes": fewer than four in hand, an incomplete body, or
    /// one byte just dropped while looking for a header.
    ///
    /// `Err` means a frame was located but its contents failed validation. One byte is
    /// dropped and the buffer is left resynchronised, so the caller chooses: a unicast
    /// transaction has only one frame to look for and reports the error, while a synchronous
    /// round still has other devices outstanding and can simply call again.
    ///
    /// The frame's length comes from the header's own length byte, **not** from what the
    /// caller expects: a device that answers with the wrong number of payload bytes is a
    /// protocol error for the caller to report, not something to wait out. That is why a
    /// byte count is a budget rather than a target.
    fn take_frame(&mut self) -> Result<Option<Frame>> {
        // `at` walks forward over rejected prefixes instead of shifting the buffer once per
        // rejected byte; the single `drain` on the way out is what reclaims them. A run of
        // `FF FF` pairs with an impossible length byte would otherwise cost a memmove each.
        let mut at = 0usize;
        loop {
            let rest = &self.pending[at..];
            if rest.len() < 4 {
                self.pending.drain(..at);
                return Ok(None);
            }
            match rest.windows(2).position(|v| v == [255, 255]) {
                Some(i) => {
                    at += i;
                    if self.pending.len() - at < 4 {
                        self.pending.drain(..at);
                        return Ok(None);
                    }
                }
                None => {
                    // No header in sight. Keep one byte — it may be the first half of a header
                    // split across two reads — and look again.
                    let keep = self.pending.len() - 1;
                    self.pending.drain(..keep);
                    return Ok(None);
                }
            }
            let size = usize::from(self.pending[at + 3]) + 4;
            if !valid_id(self.pending[at + 2], false) || !(6..=MAX_PACKET_SIZE).contains(&size) {
                // Plausible bytes, impossible header: step past this `FF` and look again.
                at += 1;
                continue;
            }
            if self.pending.len() - at < size {
                self.pending.drain(..at);
                return Ok(None);
            }
            match validate_status(&self.pending[at..at + size]) {
                Ok((id, flags)) => {
                    self.pending.drain(..at);
                    self.consumed = size;
                    return Ok(Some(Frame {
                        id,
                        flags,
                        start: 5,
                        len: size - 6,
                    }));
                }
                Err(e) => {
                    // A plausible header over a bad body: the boundary was right, the contents
                    // were not. Step past one byte, resynchronise, and hand the error up. The
                    // buffer has already moved on, so a caller that keeps going does not
                    // re-read the same bytes — which is what stops one corrupt reply from
                    // ending a whole multi-device round.
                    self.pending.drain(..at + 1);
                    return Err(e);
                }
            }
        }
    }
    /// The next complete, checksum-valid status frame.
    ///
    /// **Try first, read second.** Each round looks for a whole frame already in the buffer
    /// and returns it without reading at all; only an incomplete buffer costs a `read`. One
    /// `read` of 256 bytes usually brings a whole frame in, so the header and the body are
    /// collected by the same call — there is no separate wait for the header.
    /// **Bounded.** A stream that keeps delivering bytes but never yields a frame is not a
    /// slow reply, it is a stream with no frame in it: `stalled` counts the rounds that found
    /// none, and the call gives up once it has spent more than one frame's worth of splits
    /// plus a whole noise block. Without that bound, bytes arriving forever keep this loop
    /// turning — the transaction deadline that used to cover it has been removed.
    fn next(&mut self, r: &Request) -> Result<Frame> {
        // The caller is done with the previous frame; reclaim its bytes so a long-lived
        // receiver does not grow without bound.
        if self.consumed > 0 {
            self.pending.drain(..self.consumed);
            self.consumed = 0;
        }
        let limit = self.frame_len + MAX_PACKET_SIZE;
        let mut stalled = 0usize;
        loop {
            r.check()?;
            if let Some(f) = self.take_frame()? {
                return Ok(f);
            }
            stalled += 1;
            if stalled > limit {
                return Err(Error::Packet("no frame in the whole reply"));
            }
            self.read_once()?;
        }
    }
}

/// Restores the device delay it found when dropped.
///
/// Returned by [`Bus::with_read_delay`], and the reason to prefer that over a bare
/// [`Bus::set_read_delay`]: the restore happens on the way out of a scope, including an
/// early `?` return or a panic, so a long delay cannot outlive the operation that needed it.
///
/// It restores the value in force on entry rather than a fixed default, so nesting behaves.
#[must_use = "the read delay is restored when this guard drops"]
pub struct ReadDelayGuard<'a> {
    bus: &'a Bus,
    restore: Duration,
}
impl Drop for ReadDelayGuard<'_> {
    fn drop(&mut self) {
        self.bus.set_read_delay(self.restore);
    }
}
