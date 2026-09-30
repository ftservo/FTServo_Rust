use ftservo::{protocol::*, registers::*, *};
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
    fn set_read_timeout(&mut self, _: Duration) -> io::Result<()> {
        self.error("timeout")
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

#[test]
fn fragmented_noise_and_wrong_id() {
    for chunk in [1, 2, 256] {
        let (mut p, _) = Mock::new(|_| {
            [
                vec![9, 255, 0, 255, 255, 255, 255, 1, 1, 0],
                status(2, 0, &[9, 9]),
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
fn timeout_checksum_length_and_device_fault() {
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
    let (p, _) = Mock::new(|_| status(1, 33, &[]));
    let e = bus(p).ping(&Request::new(), 1).unwrap_err();
    if let Error::Device(e) = e {
        assert!(e.has(DeviceError::VOLTAGE));
        assert!(e.has(DeviceError::OVERLOAD));
        assert_eq!(e.id, 1);
    } else {
        panic!("{e}");
    }
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
    let result = bus(p)
        .sync_read(&Request::new(), 56, 1, &[1, 2, 3])
        .unwrap();
    assert!(!result.is_ok());
    assert_eq!(result.missing, vec![3]);
    assert_eq!(result.values[&1].data, vec![1]);
    assert_eq!(result.values[&2].flags, 32);
    assert!(result.errors.iter().any(|e| matches!(e, Error::Device(_))));
    assert!(result
        .errors
        .iter()
        .any(|e| matches!(e, Error::Timeout { .. })));
    let (p, _) = Mock::new(|_| [status(2, 0, &[2]), status(1, 0, &[1])].concat());
    assert!(bus(p)
        .sync_read(&Request::new(), 56, 1, &[1, 2])
        .unwrap()
        .is_ok());
}
#[test]
fn malformed_sync_retains_earlier_data() {
    let (p, _) = Mock::new(|_| [status(1, 0, &[1]), status(2, 1, &[])].concat());
    let b = bus(p).sync_read(&Request::new(), 56, 1, &[1, 2]).unwrap();
    assert_eq!(b.values.len(), 1);
    assert_eq!(b.missing, vec![2]);
    assert_eq!(b.errors.len(), 2);
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
    assert!(b.sync_read(&r, 0, 1, &[1, 1]).is_err());
    assert!(b.sync_read(&r, 0, 1, &[]).is_err());
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
    assert!(matches!(
        scs.sync_read_feedback(&r, &[1]),
        Err(Error::Unsupported(_))
    ));
    assert!(sms.reset(&r, 1).is_err());
    assert!(scs.wheel_mode(&r, 1).is_err());
    assert!(s.lock().unwrap().packets.is_empty());
}
#[test]
fn cancelled_deadline_and_closed() {
    let (p, s) = Mock::new(|_| vec![]);
    let b = bus(p);
    let r = Request::new();
    r.clone().cancel();
    assert!(matches!(b.ping(&r, 1), Err(Error::Cancelled)));
    assert!(s.lock().unwrap().packets.is_empty());
    let r = Request::with_timeout(Duration::from_millis(3)).unwrap();
    assert!(matches!(b.ping(&r, 1), Err(Error::DeadlineExceeded)));
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
    let r = Request::with_timeout(Duration::from_millis(3)).unwrap();
    assert!(matches!(b.ping(&r, 2), Err(Error::DeadlineExceeded)));
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
                Servo::sms_sts(b).read_feedback(&Request::new(), 1).unwrap();
            } else {
                Imu::new(b).read_sample(&Request::new(), 2).unwrap();
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
    hls.reset(&r, 1).unwrap();
    hls.calibrate_offset(&r, 1, 1024).unwrap();
    hls.action(&r, 1).unwrap();
    Imu::new(b)
        .without_write_response()
        .lock_eprom(&r, 2)
        .unwrap();
    assert_eq!(s.lock().unwrap().packets.len(), 8);
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
    assert_eq!(&packets[4][5..7], &[MODE, 2]);
    assert_eq!(&packets[5][5..8], &[GOAL_TORQUE, 0x2c, 0x81]);
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
        let f = s.read_feedback(&r, 1).unwrap();
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
    let sample = i.read_sample(&r, 3).unwrap();
    assert_eq!(sample.gyro.signed(), Vector { x: -1, y: 2, z: -3 });
    assert_eq!(sample.acceleration.signed(), Vector { x: 4, y: -5, z: 6 });
    assert!((sample.quaternion.w - 0.75f64.sqrt()).abs() < 1e-12);
    assert_eq!(i.read_gyro(&r, 3).unwrap(), sample.gyro);
    assert_eq!(i.read_acceleration(&r, 3).unwrap(), sample.acceleration);
    assert_eq!(i.read_quaternion(&r, 3).unwrap(), sample.quaternion);
    assert!(decode_imu_sample(&data[..22]).is_err());
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
    let batch = Imu::new(b.clone())
        .sync_read_samples(&r, &[1, 2, 3])
        .unwrap();
    assert_eq!(batch.values.len(), 1);
    assert_eq!(batch.values[&1].quaternion.w, 1.0);
    assert_eq!(batch.errors.len(), 2);
    assert!(batch.missing.is_empty());
    let batch = Servo::hls(b).sync_read_feedback(&r, &[1, 2, 3]).unwrap();
    assert_eq!(batch.values.len(), 2);
    assert_eq!(batch.errors.len(), 1);
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
