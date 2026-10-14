//! Socket behavior only, against a loopback fake X-Plane. Protocol behavior is
//! tested in `protocol/tests.rs`.

use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use super::{SEND_BUFFER, XPlaneUdpBridge};
use crate::codec::test_bytes::{cmnd, dref, reply, rref};
use crate::datarefs::{overrides, position};
use crate::rate::hz;
use crate::{DatarefSink, DatarefSource, Latest, commands};

const TIMEOUT: Duration = Duration::from_secs(1);

/// A fake X-Plane: requests arrive on one socket, replies leave from another,
/// as X-Plane's come from 49001 rather than 49000.
struct FakeSim {
    requests: UdpSocket,
    replies: UdpSocket,
}

impl FakeSim {
    fn new() -> Self {
        let requests = UdpSocket::bind("127.0.0.1:0").unwrap();
        requests.set_read_timeout(Some(TIMEOUT)).unwrap();
        let replies = UdpSocket::bind("127.0.0.1:0").unwrap();
        Self { requests, replies }
    }

    fn addr(&self) -> SocketAddr {
        self.requests.local_addr().unwrap()
    }

    /// The next request and where it came from; panics after `TIMEOUT`.
    fn recv(&self) -> (Vec<u8>, SocketAddr) {
        let mut buf = [0; 1024];
        let (len, from) = self.requests.recv_from(&mut buf).expect("a request");
        (buf[..len].to_vec(), from)
    }

    fn assert_quiet(&self) {
        self.requests
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();
        let mut buf = [0; 1024];
        let got = self.requests.recv_from(&mut buf);
        assert!(got.is_err(), "unexpected request {:?}", got.map(|(n, _)| n));
    }
}

/// Polls into `latest` until `done` holds; panics after `TIMEOUT`.
fn poll_until(
    bridge: &mut XPlaneUdpBridge,
    latest: &mut Latest,
    done: impl Fn(&XPlaneUdpBridge) -> bool,
) {
    let deadline = Instant::now() + TIMEOUT;
    while !done(bridge) {
        assert!(Instant::now() < deadline, "timed out polling");
        if let Some(frame) = bridge.poll().unwrap() {
            latest.apply(&frame);
        }
    }
}

#[test]
fn binds_unspecified_address_of_target_family() {
    let cases: [(&str, &str); 2] = [("127.0.0.1:49000", "0.0.0.0"), ("[::1]:49000", "::")];
    for (target, local) in cases {
        let bridge = XPlaneUdpBridge::connect(target).unwrap();
        let bound = bridge.sock.local_addr().unwrap();
        assert_eq!(bound.ip().to_string(), local, "target {target}");
        assert_ne!(bound.port(), 0, "target {target}");
    }
}

#[test]
fn enlarges_send_buffer() {
    let bridge = XPlaneUdpBridge::connect("127.0.0.1:49000").unwrap();
    let size = socket2::SockRef::from(&bridge.sock)
        .send_buffer_size()
        .unwrap();
    assert!(size >= SEND_BUFFER, "send buffer {size}");
}

#[test]
fn resolves_hostname_to_ipv4() {
    let sim = FakeSim::new();
    let mut bridge = XPlaneUdpBridge::connect(("localhost", sim.addr().port())).unwrap();
    bridge.subscribe(&position::PHI, hz(100)).unwrap();
    assert_eq!(sim.recv().0, rref(100, 0, &position::PHI.path()));
}

#[test]
fn sends_each_call_as_one_datagram() {
    let sim = FakeSim::new();
    let mut bridge = XPlaneUdpBridge::connect(sim.addr()).unwrap();

    bridge.subscribe(&position::PHI, hz(100)).unwrap();
    assert_eq!(sim.recv().0, rref(100, 0, &position::PHI.path()));
    bridge.write(&overrides::THROTTLES, true).unwrap();
    assert_eq!(
        sim.recv().0,
        dref(1.0, "sim/operation/override/override_throttles")
    );
    bridge.command(&commands::PAUSE_TOGGLE).unwrap();
    assert_eq!(sim.recv().0, cmnd(commands::PAUSE_TOGGLE.path()));
    let theta = bridge.subscribe(&position::THETA, hz(50)).unwrap();
    assert_eq!(sim.recv().0, rref(50, 1, &position::THETA.path()));
    bridge.unsubscribe(theta).unwrap();
    assert_eq!(sim.recv().0, rref(0, 1, &position::THETA.path()));
    bridge.unsubscribe(theta).unwrap();
    bridge.resubscribe().unwrap();
    assert_eq!(sim.recv().0, rref(100, 0, &position::PHI.path()));
    sim.assert_quiet();
}

#[test]
fn drains_replies_from_another_port() {
    let sim = FakeSim::new();
    let mut bridge = XPlaneUdpBridge::connect(sim.addr()).unwrap();
    let phi = bridge.subscribe(&position::PHI, hz(100)).unwrap();
    let theta = bridge.subscribe(&position::THETA, hz(100)).unwrap();
    let (_, from) = sim.recv();
    sim.recv();

    for records in [[(0, 1.0)], [(1, 2.0)], [(0, 3.0)]] {
        sim.replies.send_to(&reply(&records), from).unwrap();
    }
    let mut latest = Latest::default();
    poll_until(&mut bridge, &mut latest, |b| b.stats().packets == 3);
    assert_eq!(latest.get(phi).map(|(v, _)| v), Some(3.0));
    assert_eq!(latest.get(theta).map(|(v, _)| v), Some(2.0));
    assert_eq!(bridge.poll().unwrap(), None);
}

#[test]
fn poll_is_none_before_anything_arrives() {
    let sim = FakeSim::new();
    let mut bridge = XPlaneUdpBridge::connect(sim.addr()).unwrap();
    bridge.subscribe(&position::PHI, hz(100)).unwrap();
    assert_eq!(bridge.poll().unwrap(), None);
    assert_eq!(bridge.stats().packets, 0);
}

#[test]
fn drop_unsubscribes_active() {
    let sim = FakeSim::new();
    let mut bridge = XPlaneUdpBridge::connect(sim.addr()).unwrap();
    bridge.subscribe(&position::PHI, hz(100)).unwrap();
    let theta = bridge.subscribe(&position::THETA, hz(100)).unwrap();
    bridge.unsubscribe(theta).unwrap();
    for _ in 0..3 {
        sim.recv();
    }

    drop(bridge);
    assert_eq!(sim.recv().0, rref(0, 0, &position::PHI.path()));
    sim.assert_quiet();
}

/// Settles the Windows `ConnectionReset` question for any OS CI runs on: after
/// sending to a closed port, `poll` keeps returning `Ok`.
#[test]
fn poll_survives_send_to_closed_port() {
    let closed = UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap();
    let mut bridge = XPlaneUdpBridge::connect(closed).unwrap();
    bridge.subscribe(&position::PHI, hz(100)).unwrap();
    let deadline = Instant::now() + Duration::from_millis(200);
    while Instant::now() < deadline {
        assert_eq!(bridge.poll().unwrap(), None);
    }
}

/// The fact `poll`'s `ConnectionReset` arm handles: Windows reports a `send_to`
/// to a closed port on the next `recv_from` of an unconnected socket.
#[cfg(windows)]
#[test]
fn windows_reports_connection_reset() {
    use std::io::ErrorKind;

    let closed = UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap();
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock.set_read_timeout(Some(TIMEOUT)).unwrap();
    sock.send_to(b"ping", closed).unwrap();
    let err = sock.recv_from(&mut [0; 16]).unwrap_err();
    assert_eq!(err.kind(), ErrorKind::ConnectionReset);
}

#[test]
fn is_send() {
    fn assert_send<T: Send>() {}
    assert_send::<XPlaneUdpBridge>();
}

/// A send that fails for a reason other than a full buffer reaches the caller;
/// on drop it is logged, not raised. Unix rejects a send to port 0; Windows
/// accepts it.
#[cfg(unix)]
#[test]
fn send_error_reaches_caller() {
    let mut bridge = XPlaneUdpBridge::connect("127.0.0.1:0").unwrap();
    let err = bridge.subscribe(&position::PHI, hz(100)).unwrap_err();
    assert!(matches!(err, crate::XPlaneError::Io(_)), "{err:?}");
    assert!(bridge.write(&overrides::THROTTLES, true).is_err());
    assert!(bridge.command(&commands::PAUSE_TOGGLE).is_err());
    assert!(bridge.resubscribe().is_err());
    assert_eq!(bridge.stats().dropped_sends, 0);
    drop(bridge);
}
