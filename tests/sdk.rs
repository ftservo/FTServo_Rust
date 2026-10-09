use ftservo::{protocol::*, *};
use std::{
    collections::VecDeque,
    io::{self, Read, Write},
    sync::{mpsc, Arc, Mutex},
    thread,
    time::Duration,
};

#[derive(Default)]
struct State {
    rx: VecDeque<u8>,
    tx: Vec<u8>,
    packets: Vec<Vec<u8>>,
    discarded: usize,
    dropped: bool,
    /// The last window the SDK asked the transport to wait for. The read-path budget is
    /// asserted against it.
    last_timeout: Option<Duration>,
}
type Reply = Box<dyn Fn(&[u8]) -> Vec<u8> + Send>;
struct Mock {
    state: Arc<Mutex<State>>,
    reply: Reply,
    read_size: usize,
    write_size: usize,
    zero_write: bool,
    fail: Option<&'static str>,
}
impl Mock {
    fn new(reply: impl Fn(&[u8]) -> Vec<u8> + Send + 'static) -> (Self, Arc<Mutex<State>>) {
        let state = Arc::new(Mutex::new(State::default()));
        (
            Self {
                state: state.clone(),
                reply: Box::new(reply),
                read_size: 256,
                write_size: 256,
                zero_write: false,
                fail: None,
            },
            state,
        )
    }
    fn error(&self, at: &str) -> io::Result<()> {
        if self.fail == Some(at) {
            Err(io::Error::other("driver error"))
        } else {
            Ok(())
        }
    }
}
impl Drop for Mock {
    fn drop(&mut self) {
        self.state.lock().unwrap().dropped = true;
    }
}
impl Transport for Mock {
    fn clear_input(&mut self) -> io::Result<()> {
        self.error("clear")?;
        let mut s = self.state.lock().unwrap();
        s.discarded += s.rx.len();
        s.rx.clear();
        Ok(())
    }
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.error("timeout")?;
        self.state.lock().unwrap().last_timeout = Some(timeout);
        Ok(())
    }
}
impl Read for Mock {
    fn read(&mut self, p: &mut [u8]) -> io::Result<usize> {
        self.error("read")?;
        let mut s = self.state.lock().unwrap();
        let n = p.len().min(self.read_size).min(s.rx.len());
        for slot in &mut p[..n] {
            *slot = s.rx.pop_front().unwrap();
        }
        Ok(n)
    }
}
impl Write for Mock {
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
    fn write(&mut self, p: &[u8]) -> io::Result<usize> {
        self.error("write")?;
        if self.zero_write {
            return Ok(0);
        }
        let n = p.len().min(self.write_size);
        let mut s = self.state.lock().unwrap();
        s.tx.extend_from_slice(&p[..n]);
        if s.tx.len() >= 4 && s.tx.len() == usize::from(s.tx[3]) + 4 {
            let packet = std::mem::take(&mut s.tx);
            let reply = (self.reply)(&packet);
            s.packets.push(packet);
            s.rx.extend(reply);
        }
        Ok(n)
    }
}
fn status(id: u8, flags: u8, data: &[u8]) -> Vec<u8> {
    let mut p = vec![255, 255, id, (data.len() + 2) as u8, flags];
    p.extend_from_slice(data);
    let sum: u32 = p[2..].iter().map(|&v| u32::from(v)).sum();
    p.push((!sum & 255) as u8);
    p
}
fn ack(p: &[u8]) -> Vec<u8> {
    if p[2] == BROADCAST_ID {
        vec![]
    } else {
        status(p[2], 0, &[])
    }
}
fn bus(p: Mock) -> Bus {
    Bus::new(p, Duration::from_millis(30)).unwrap()
}
/// The 15-byte servo feedback block, decoded the way an application does it: the SDK hands
/// over the bytes and the field codecs, not the block layout, because where each field sits is
/// a family fact. `unsigned_position` is SCSCL — big-endian words and a plain unsigned
/// position — where SMS/STS and HLS use little-endian words and a 15-bit direction bit.
fn decode_feedback(order: ByteOrder, unsigned_position: bool, p: &[u8]) -> Feedback {
    assert_eq!(p.len(), 15);
    let word = |i: usize| match order {
        ByteOrder::Little => u16::from_le_bytes([p[i], p[i + 1]]),
        ByteOrder::Big => u16::from_be_bytes([p[i], p[i + 1]]),
    };
    Feedback {
        position: if unsigned_position {
            i32::from(word(0))
        } else {
            decode_sign_magnitude(word(0), 15).unwrap()
        },
        speed: decode_sign_magnitude(word(2), 15).unwrap(),
        load: decode_sign_magnitude(word(4), 10).unwrap(),
        current: decode_sign_magnitude(word(13), 15).unwrap(),
        voltage: p[6],
        temperature: p[7],
        moving: p[10] != 0,
    }
}
/// The IMU's quaternion as it comes off the wire: three half floats at 0x38/0x3A/0x3C, with
/// `W` recovered arithmetically. Spelled out here for the same reason `decode_feedback` is —
/// the SDK hands the bytes over instead of decoding a block for the caller.
fn decode_quaternion(p: &[u8]) -> Result<Quaternion> {
    let word = |off: usize| u16::from_le_bytes([p[off], p[off + 1]]);
    Quaternion::from_raw(VectorRaw {
        x: word(0),
        y: word(2),
        z: word(4),
    })
}

#[test]
fn fragmented_noise_and_chunked_arrival() {
    // One byte at a time, two at a time, then all at once; a header-looking 0xFF 0xFF inside
    // the noise; and the real reply last. Where a frame ends has to come from its own length
    // byte, not from how the transport happened to split the bytes.
    for chunk in [1, 2, 256] {
        let (mut p, _) = Mock::new(|_| {
            [
                vec![9, 255, 0, 255, 255, 255, 255, 1, 1, 0],
                status(1, 0, &[0x34, 0x12]),
            ]
            .concat()
        });
        p.read_size = chunk;
        p.write_size = 2;
        assert_eq!(
            Servo::sms_sts(bus(p))
                .read_u16(&Request::new(), 1, 56)
                .unwrap(),
            0x1234
        );
    }
}
#[test]
fn reply_from_another_device_is_reported() {
    // A well-formed frame from an ID nobody asked about is not this transaction's answer: a
    // unicast request has exactly one legitimate respondent, so it is reported rather than
    // skipped. `sync_read` is the opposite case and still skips — its reply set is many
    // devices by design.
    let (p, _) = Mock::new(|_| status(2, 0, &[9, 9]));
    assert!(matches!(
        Servo::sms_sts(bus(p)).read_u16(&Request::new(), 1, 56),
        Err(Error::UnexpectedId {
            expected: 1,
            got: 2
        })
    ));
}
#[test]
fn decoded_frames_expose_their_status_code() {
    let (p, _) = Mock::new(|_| [status(1, 0, &[0; 15]), status(2, 1, &[0; 15])].concat());
    let b = bus(p);
    let mut out = Batch::new();
    b.sync_read_decoded(
        &Request::new(),
        registers::PRESENT_POSITION,
        15,
        &[1, 2],
        |p| Ok(decode_feedback(ByteOrder::Little, false, p)),
        &mut out,
    )
    .unwrap();
    // Both replies carry a code, healthy or not: the status byte reports on the device while
    // the round itself succeeded. `Feedback` has no field for it, so this channel is the only
    // way to see it on the typed path — and neither device is missing, because both answered.
    assert_eq!(out.flags(1), Some(0));
    assert_eq!(out.flags(2), Some(1));
    assert_eq!(out.len(), 2);
    assert!(out.missing.is_empty());
    assert!(out.errors.is_empty());
}
#[test]
fn corrupt_frame_does_not_end_the_round() {
    // Device 2's reply is damaged in transit (its checksum byte flipped), device 1's is
    // intact. The corrupt frame is reported and dropped, but the round keeps going: device 1
    // is still read rather than being written off as missing because of its neighbour.
    let (p, _) = Mock::new(|_| {
        let mut bad = status(2, 0, &[0x11]);
        let last = bad.len() - 1;
        bad[last] ^= 0xFF;
        [bad, status(1, 0, &[0x22])].concat()
    });
    let mut out = Batch::new();
    bus(p)
        .sync_read(&Request::new(), 56, 1, &[1, 2], &mut out)
        .unwrap();
    assert_eq!(out.get(1).unwrap().data, vec![0x22]);
    assert!(!out.missing.contains(&1));
    assert!(out.missing.contains(&2));
    assert!(out.errors.iter().any(|e| matches!(e, Error::Checksum)));
}
#[test]
fn a_stream_without_a_frame_gives_up() {
    // 0x00 can never begin a frame, and `read_size = 1` makes each byte cost its own round of
    // `take_frame`, so nothing is ever found. The guard has to end this: without it the loop
    // keeps reading as long as the transport keeps handing bytes over, which it does here for
    // 512 bytes — well past the PING threshold of 6 + 250.
    let (mut p, _) = Mock::new(|_| vec![0u8; 512]);
    p.read_size = 1;
    assert!(matches!(
        bus(p).ping(&Request::new(), 1),
        Err(Error::Packet("no frame in the whole reply"))
    ));
}
#[test]
fn timeout_checksum_length_and_device_status() {
    let (p, _) = Mock::new(|_| vec![]);
    assert!(matches!(
        bus(p).ping(&Request::new(), 1),
        Err(Error::Timeout { partial: false })
    ));
    let (p, _) = Mock::new(|_| vec![255, 255, 1, 4, 0]);
    assert!(matches!(
        bus(p).read(&Request::new(), 1, 56, 2),
        Err(Error::Timeout { partial: true })
    ));
    let (p, _) = Mock::new(|_| {
        let mut v = status(1, 0, &[1, 2]);
        *v.last_mut().unwrap() ^= 1;
        v
    });
    assert!(matches!(
        bus(p).read(&Request::new(), 1, 56, 2),
        Err(Error::Checksum)
    ));
    let (p, _) = Mock::new(|_| status(1, 0, &[1]));
    assert!(matches!(
        bus(p).read(&Request::new(), 1, 56, 2),
        Err(Error::Packet(_))
    ));
    // 33 = 0b0010_0001: a non-zero status byte is the device reporting on itself, not a failed
    // transaction. The reply comes back like any other, with the raw byte in `flags` — the SDK
    // does not name any of the bits, that is the datasheet's and the application's business.
    let (p, _) = Mock::new(|_| status(1, 33, &[]));
    let s = bus(p)
        .command(&Request::new(), 1, PING, &[], Some(0))
        .unwrap();
    assert_eq!(s.flags, 33);
    assert_eq!(s.id, 1);
    assert!(s.data.is_empty());
}
#[test]
fn sync_partial_fault_duplicate_and_out_of_order() {
    let (p, _) = Mock::new(|_| {
        [
            status(9, 0, &[9]),
            status(2, 32, &[2]),
            status(2, 0, &[8]),
            status(1, 0, &[1]),
        ]
        .concat()
    });
    let mut result = Batch::new();
    bus(p)
        .sync_read(&Request::new(), 56, 1, &[1, 2, 3], &mut result)
        .unwrap();
    assert!(!result.is_ok());
    assert_eq!(result.missing, vec![3]);
    assert_eq!(result.get(1).unwrap().data, vec![1]);
    assert_eq!(result.get(2).unwrap().flags, 32);
    // The status byte is reported with the data, not as an error: only the timeout is one.
    assert!(result
        .errors
        .iter()
        .any(|e| matches!(e, Error::Timeout { .. })));
    let (p, _) = Mock::new(|_| [status(2, 0, &[2]), status(1, 0, &[1])].concat());
    let mut result = Batch::new();
    bus(p)
        .sync_read(&Request::new(), 56, 1, &[1, 2], &mut result)
        .unwrap();
    assert!(result.is_ok());
}
#[test]
fn malformed_sync_retains_earlier_data() {
    let (p, _) = Mock::new(|_| [status(1, 0, &[1]), status(2, 1, &[])].concat());
    let mut b = Batch::new();
    bus(p)
        .sync_read(&Request::new(), 56, 1, &[1, 2], &mut b)
        .unwrap();
    assert_eq!(b.values.len(), 1);
    assert_eq!(b.missing, vec![2]);
    // Only the length mismatch is an error now. Device 2's non-zero status byte comes back
    // with its (empty) payload instead of being turned into a failure.
    assert_eq!(b.errors.len(), 1);
}
#[test]
fn validation_before_io() {
    let (p, s) = Mock::new(ack);
    let b = bus(p);
    let r = Request::new();
    assert!(b.read(&r, BROADCAST_ID, 0, 1).is_err());
    assert!(b.read(&r, 1, 255, 2).is_err());
    assert!(b.read(&r, 1, 0, 245).is_err());
    assert!(b.write(&r, 255, 0, &[1]).is_err());
    assert!(b.write(&r, 1, 0, &[]).is_err());
    let mut out = Batch::new();
    assert!(b.sync_read(&r, 0, 1, &[1, 1], &mut out).is_err());
    assert!(b.sync_read(&r, 0, 1, &[], &mut out).is_err());
    assert!(b
        .sync_write(
            &r,
            0,
            1,
            &[WriteEntry {
                id: 1,
                data: vec![1, 2]
            }]
        )
        .is_err());
    assert!(b
        .sync_write(
            &r,
            0,
            1,
            &[
                WriteEntry {
                    id: 1,
                    data: vec![1]
                },
                WriteEntry {
                    id: 1,
                    data: vec![1]
                }
            ]
        )
        .is_err());
    let sms = Servo::sms_sts(b.clone());
    let scs = Servo::scscl(b.clone());
    let hls = Servo::hls(b);
    for pos in [-32768, 32768, i32::MIN, i32::MAX] {
        assert!(sms.write_position(&r, Motion::new(1, pos)).is_err());
    }
    assert!(sms
        .write_position(
            &r,
            Motion {
                torque: 1,
                ..Motion::new(1, 0)
            }
        )
        .is_err());
    assert!(hls
        .write_position(
            &r,
            Motion {
                time: 1,
                ..Motion::new(1, 0)
            }
        )
        .is_err());
    assert!(scs.write_position(&r, Motion::new(1, -1)).is_err());
    assert!(scs.write_pwm(&r, 1, 1024).is_err());
    assert!(scs.wheel_mode(&r, 1).is_err());
    assert!(s.lock().unwrap().packets.is_empty());
}
#[test]
fn cancelled_timeout_and_closed() {
    let (p, s) = Mock::new(|_| vec![]);
    let b = bus(p);
    let r = Request::new();
    r.clone().cancel();
    assert!(matches!(b.ping(&r, 1), Err(Error::Cancelled)));
    assert!(s.lock().unwrap().packets.is_empty());
    // Nothing answers, so the transaction spends its own budget (PING's is about 1.6 ms) and
    // gives up: `Timeout` is the honest answer for "the frame never arrived", and there is no
    // caller-supplied deadline left to blame instead.
    assert!(matches!(
        b.ping(&Request::new(), 1),
        Err(Error::Timeout { .. })
    ));
    let clone = b.clone();
    b.close().unwrap();
    b.close().unwrap();
    assert!(s.lock().unwrap().dropped);
    assert!(matches!(clone.ping(&Request::new(), 1), Err(Error::Closed)));
}
#[test]
fn cancel_queued_and_active_requests() {
    let (entered_tx, entered_rx) = mpsc::channel();
    let (release_tx, release_rx) = mpsc::channel();
    let (p, _) = Mock::new(move |p| {
        entered_tx.send(()).unwrap();
        release_rx.recv().unwrap();
        ack(p)
    });
    let b = bus(p);
    let worker = b.clone();
    let join = thread::spawn(move || worker.ping(&Request::new(), 1));
    entered_rx.recv_timeout(Duration::from_secs(1)).unwrap();
    // The queued request gives up when the transaction's own budget runs out — PING's is
    // about 1.6 ms — rather than at a timeout of the caller's choosing.
    let r = Request::new();
    assert!(matches!(
        b.ping(&r, 2),
        Err(Error::Timeout { partial: false })
    ));
    release_tx.send(()).unwrap();
    join.join().unwrap().unwrap();
    let r = Request::new();
    let cancel = r.clone();
    let (p, _) = Mock::new(move |_| {
        cancel.cancel();
        vec![]
    });
    assert!(matches!(bus(p).ping(&r, 1), Err(Error::Cancelled)));
}
#[test]
fn concurrent_mixed_devices_own_entire_transaction() {
    let (mut p, s) = Mock::new(|p| status(p[2], 0, &vec![0; usize::from(p[6])]));
    p.read_size = 1;
    p.write_size = 1;
    let b = bus(p);
    let mut threads = vec![];
    for n in 0..40 {
        let b = b.clone();
        threads.push(thread::spawn(move || {
            if n % 2 == 0 {
                Servo::sms_sts(b).read_position(&Request::new(), 1).unwrap();
            } else {
                Imu::new(b).read_quaternion(&Request::new(), 2).unwrap();
            }
        }));
    }
    for j in threads {
        j.join().unwrap();
    }
    let state = s.lock().unwrap();
    assert_eq!(state.packets.len(), 40);
    assert_eq!(state.discarded, 0);
}
#[test]
fn io_errors_and_zero_write() {
    for where_ in ["clear", "timeout", "read", "write"] {
        let (mut p, _) = Mock::new(ack);
        p.fail = Some(where_);
        assert!(matches!(bus(p).ping(&Request::new(), 1), Err(Error::Io(_))));
    }
    let (mut p, _) = Mock::new(ack);
    p.zero_write = true;
    assert!(
        matches!(bus(p).ping(&Request::new(),1),Err(Error::Io(e)) if e.kind()==io::ErrorKind::WriteZero)
    );
}
#[test]
fn no_response_and_broadcast() {
    let (p, s) = Mock::new(|_| vec![]);
    let b = bus(p);
    let r = Request::new();
    b.write(&r, BROADCAST_ID, 40, &[0]).unwrap();
    b.write_only(&r, 1, 40, &[0]).unwrap();
    b.reg_write_only(&r, 1, 40, &[0]).unwrap();
    b.action(&r, BROADCAST_ID).unwrap();
    let hls = Servo::hls(b.clone()).without_write_response();
    // `reset` sits on `Device` beside `ping`, not on the servo layer: the instruction is the
    // protocol's, so any handle on the bus sends it — here an HLS servo and an IMU.
    hls.reset(&r, 1).unwrap();
    hls.calibrate_offset(&r, 1, 1024).unwrap();
    hls.action(&r, 1).unwrap();
    let imu = Imu::new(b).without_write_response();
    imu.lock_eprom(&r, 2).unwrap();
    imu.reset(&r, 2).unwrap();
    assert_eq!(s.lock().unwrap().packets.len(), 9);
}
#[test]
fn signed_and_binary16() {
    for bit in [10, 15] {
        let lim = (1 << bit) - 1;
        for v in [-lim, -1, 0, 1, lim] {
            assert_eq!(
                decode_sign_magnitude(encode_sign_magnitude(v, bit).unwrap(), bit).unwrap(),
                v
            );
        }
    }
    assert!(encode_sign_magnitude(-32768, 15).is_err());
    assert!(encode_sign_magnitude(1, 16).is_err());
    assert!(decode_sign_magnitude(1, 16).is_err());
    for (raw, want) in [
        (0, 0.0),
        (0x3c00, 1.0),
        (0xc000, -2.0),
        (1, 2f64.powi(-24)),
        (0x0400, 2f64.powi(-14)),
        (0x7bff, 65504.0),
    ] {
        assert_eq!(half_float(raw), want);
    }
    assert!(half_float(0x8000).is_sign_negative());
    assert!(half_float(0x7e00).is_nan());
    assert_eq!(half_float(0xfc00), f64::NEG_INFINITY);
}
#[test]
fn hls_negative_position_and_modes() {
    let (p, s) = Mock::new(ack);
    let b = bus(p);
    let h = Servo::hls(b.clone());
    let r = Request::new();
    h.write_position(&r, Motion::new(1, -1234)).unwrap();
    assert_eq!(&s.lock().unwrap().packets[0][7..9], &[0xd2, 0x84]);
    h.enable_torque(&r, 1, true).unwrap();
    h.wheel_mode(&r, 1).unwrap();
    h.position_mode(&r, 1).unwrap();
    h.torque_mode(&r, 1).unwrap();
    h.write_torque(&r, 1, -300).unwrap();
    let packets = &s.lock().unwrap().packets;
    assert_eq!(&packets[4][5..7], &[hls::registers::MODE, 2]);
    assert_eq!(
        &packets[5][5..8],
        &[hls::registers::GOAL_TORQUE, 0x2c, 0x81]
    );
}
#[test]
fn feedback_orders_and_convenience() {
    for family in [Family::SmsSts, Family::Scscl, Family::Hls] {
        let mut data = [0; 15];
        let word = |v: u16| {
            if family == Family::Scscl {
                v.to_be_bytes()
            } else {
                v.to_le_bytes()
            }
        };
        for (off, v) in [(0, 0x800a), (2, 0x8002), (4, 0x403), (13, 0x8004)] {
            data[off..off + 2].copy_from_slice(&word(v));
        }
        data[6] = 120;
        data[7] = 30;
        data[10] = 1;
        let (p, _) = Mock::new(move |p| {
            let start = usize::from(p[5] - 56);
            status(p[2], 0, &data[start..start + usize::from(p[6])])
        });
        let s = Servo::new(bus(p), family);
        let r = Request::new();
        let raw = s.read(&r, 1, registers::PRESENT_POSITION, 15).unwrap();
        let f = decode_feedback(s.byte_order(), family == Family::Scscl, &raw);
        let position = if family == Family::Scscl { 0x800a } else { -10 };
        assert_eq!(
            f,
            Feedback {
                position,
                speed: -2,
                load: -3,
                current: -4,
                voltage: 120,
                temperature: 30,
                moving: true
            }
        );
        assert_eq!(s.read_position_speed(&r, 1).unwrap(), (position, -2));
        assert_eq!(s.read_position(&r, 1).unwrap(), position);
        assert_eq!(s.read_speed(&r, 1).unwrap(), -2);
        assert_eq!(s.read_load(&r, 1).unwrap(), -3);
        assert_eq!(s.read_current(&r, 1).unwrap(), -4);
        assert_eq!(s.read_voltage(&r, 1).unwrap(), 120);
        assert_eq!(s.read_temperature(&r, 1).unwrap(), 30);
        assert!(s.read_moving(&r, 1).unwrap());
    }
}
#[test]
fn imu_stride_quaternion_and_batches() {
    let mut data = vec![0; 23];
    data[0..2].copy_from_slice(&0x3800u16.to_le_bytes());
    for off in [8, 11, 14, 17, 20] {
        data[off] = 0xff;
    }
    for (off, v) in [
        (6, 0x8001u16),
        (9, 2),
        (12, 0x8003),
        (15, 4),
        (18, 0x8005),
        (21, 6),
    ] {
        data[off..off + 2].copy_from_slice(&v.to_le_bytes());
    }
    let copied = data.clone();
    let (p, _) = Mock::new(move |p| {
        let start = usize::from(p[5] - 56);
        status(p[2], 0, &copied[start..start + usize::from(p[6])])
    });
    let i = Imu::new(bus(p));
    let r = Request::new();
    let quaternion = i.read_quaternion(&r, 3).unwrap();
    let gyro = i.read_gyro(&r, 3).unwrap();
    let acceleration = i.read_acceleration(&r, 3).unwrap();
    assert_eq!(gyro.signed(), Vector { x: -1, y: 2, z: -3 });
    assert_eq!(acceleration.signed(), Vector { x: 4, y: -5, z: 6 });
    assert!((quaternion.w - 0.75f64.sqrt()).abs() < 1e-12);
    assert_eq!(quaternion.raw, i.read_quaternion_raw(&r, 3).unwrap());
    assert_eq!(
        Quaternion::from_raw(VectorRaw {
            x: 0x3c00,
            y: 0x3c00,
            z: 0
        })
        .unwrap()
        .w,
        0.0
    );
    assert!(Quaternion::from_raw(VectorRaw {
        x: 0x7e00,
        y: 0,
        z: 0
    })
    .is_err());
    let (p, _) = Mock::new(|p| {
        let good = vec![0; usize::from(p[6])];
        let mut bad = good.clone();
        bad[0..2].copy_from_slice(&0x7c00u16.to_le_bytes());
        [status(2, 4, &good), status(1, 0, &good), status(3, 0, &bad)].concat()
    });
    let b = bus(p);
    let mut batch = Batch::new();
    b.sync_read_decoded(
        &r,
        imu::registers::QUATERNION_X,
        6,
        &[1, 2, 3],
        decode_quaternion,
        &mut batch,
    )
    .unwrap();
    // Device 2's non-zero status byte no longer stops it from being decoded, so it is a value
    // like device 1; only device 3's payload is undecodable.
    assert_eq!(batch.values.len(), 2);
    assert_eq!(batch.get(1).unwrap().w, 1.0);
    assert_eq!(batch.flags(2), Some(4));
    assert_eq!(batch.errors.len(), 1);
    assert!(batch.missing.is_empty());
    let mut batch = Batch::new();
    b.sync_read_decoded(
        &r,
        registers::PRESENT_POSITION,
        15,
        &[1, 2, 3],
        |p| Ok(decode_feedback(ByteOrder::Little, false, p)),
        &mut batch,
    )
    .unwrap();
    // Servo feedback has no such range check, so those same two bytes decode into something:
    // all three devices are values here, and nothing at all is an error.
    assert_eq!(batch.values.len(), 3);
    assert!(batch.errors.is_empty());
}
#[test]
fn model_and_big_endian_word_pair() {
    let (p, _) = Mock::new(|p| {
        if p[4] == PING {
            status(1, 0, &[])
        } else if p[6] == 2 {
            status(1, 0, &[0x12, 0x34])
        } else {
            status(1, 0, &[0x56, 0x78, 0x12, 0x34])
        }
    });
    let s = Servo::scscl(bus(p));
    let r = Request::new();
    assert_eq!(s.ping(&r, 1).unwrap(), 0x1234);
    assert_eq!(s.read_u32(&r, 1, 42).unwrap(), 0x12345678);
    assert_eq!(s.byte_order(), ByteOrder::Big);
}
#[test]
fn packet_bounds_and_arbitrary_input() {
    assert_eq!(encode_packet(1, WRITE, &[0; 244]).unwrap().len(), 250);
    assert!(encode_packet(1, WRITE, &[0; 245]).is_err());
    assert!(decode_status(&status(254, 0, &[])).is_err());
    let mut seed = 0x12345678u32;
    for n in 0..30_000 {
        let mut bytes = vec![0; (n % 300) as usize];
        for b in &mut bytes {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            *b = seed as u8;
        }
        if let Ok(s) = decode_status(&bytes) {
            assert_eq!(s.data.len() + 6, bytes.len());
            assert!(s.id <= MAX_ID);
        }
    }
}

#[test]
fn command_rejects_an_impossible_expected_length() {
    // `command` is public, so `expected` is caller input. An out-of-range value has to be an
    // argument error — not an overflowed budget, and not a reply length the protocol cannot
    // carry. The wrappers bound theirs through `register_range`; this is the direct entry.
    let (p, _) = Mock::new(ack);
    let b = bus(p);
    assert!(matches!(
        b.command(&Request::new(), 1, 2, &[56, 1], Some(usize::MAX)),
        Err(Error::InvalidArgument(_))
    ));
    assert!(matches!(
        b.command(&Request::new(), 1, 2, &[56, 1], Some(245)),
        Err(Error::InvalidArgument(_))
    ));
    // The ceiling itself is legal and fails later — on the reply, not the argument.
    assert!(matches!(
        b.command(&Request::new(), 1, 2, &[56, 1], Some(244)),
        Err(Error::Packet(_))
    ));
}

/// The read delay is bus state, not per-handle state: every clone sees a change, and a
/// guard restores the value it found rather than a fixed default, so nesting behaves.
#[test]
fn read_delay_is_shared_and_restored() {
    let (p, _) = Mock::new(ack);
    let bus = bus(p);
    let clone = bus.clone();

    assert_eq!(bus.read_delay(), DEFAULT_READ_DELAY);

    // Visible through every handle on the same port.
    clone.set_read_delay(EEPROM_READ_DELAY);
    assert_eq!(bus.read_delay(), EEPROM_READ_DELAY);

    // A guard restores what was in force, not the default.
    {
        let _slow = bus.with_read_delay(Duration::from_secs(1));
        assert_eq!(bus.read_delay(), Duration::from_secs(1));
    }
    assert_eq!(bus.read_delay(), EEPROM_READ_DELAY);

    // Nested guards unwind to the outer setting.
    {
        let _outer = bus.with_read_delay(Duration::from_millis(5));
        {
            let _inner = bus.with_read_delay(Duration::from_millis(9));
            assert_eq!(bus.read_delay(), Duration::from_millis(9));
        }
        assert_eq!(bus.read_delay(), Duration::from_millis(5));
    }
    assert_eq!(bus.read_delay(), EEPROM_READ_DELAY);
}

/// The delay reaches the transport: the same read asks for a wider window once the delay
/// is raised, which is what makes it a read-path budget rather than a bookkeeping value.
#[test]
fn the_read_delay_reaches_the_transport() {
    let (p, s) = Mock::new(|_| status(1, 0, &[0x34, 0x12]));
    let bus = bus(p);

    bus.read(&Request::new(), 1, 56, 2).unwrap();
    let normal = s.lock().unwrap().last_timeout.unwrap();
    assert!(
        normal >= DEFAULT_READ_DELAY,
        "a read window of {normal:?} is below the default delay"
    );

    bus.set_read_delay(EEPROM_READ_DELAY);
    bus.read(&Request::new(), 1, 56, 2).unwrap();
    let slow = s.lock().unwrap().last_timeout.unwrap();
    assert!(
        slow >= EEPROM_READ_DELAY,
        "a read window of {slow:?} is below the raised delay"
    );
    assert!(
        normal < slow,
        "raising the delay did not widen the window ({normal:?} then {slow:?})"
    );
}

/// The window follows what is already in hand rather than a predicted total: each read buys
/// the fixed delay plus the time the bytes collected so far would have taken.
#[test]
fn the_read_window_follows_the_bytes_in_hand() {
    // One byte per read: the last of the eight starts with seven already in hand.
    let (mut p, s) = Mock::new(|_| status(1, 0, &[0x34, 0x12]));
    p.read_size = 1;
    bus(p).read(&Request::new(), 1, 56, 2).unwrap();
    let dribbled = s.lock().unwrap().last_timeout.unwrap();

    // All eight in one read: nothing is in hand when that only read is issued.
    let (p, s) = Mock::new(|_| status(1, 0, &[0x34, 0x12]));
    bus(p).read(&Request::new(), 1, 56, 2).unwrap();
    let whole = s.lock().unwrap().last_timeout.unwrap();

    assert!(
        dribbled > whole,
        "a dribbled read ({dribbled:?}) did not get a wider window than a whole one ({whole:?})"
    );
    // The fixed delay is counted on every read, so neither window can fall below it.
    assert!(dribbled >= DEFAULT_READ_DELAY);
    assert!(whole >= DEFAULT_READ_DELAY);
}
