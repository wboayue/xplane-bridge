use std::time::{Duration, Instant};

use super::*;
use crate::commands;
use crate::datarefs::{engine, overrides, position};

const PHI: &str = "sim/flightmodel/position/phi";
const THETA: &str = "sim/flightmodel/position/theta";

fn hz(hz: u16) -> Rate {
    Rate::hz(hz).unwrap()
}

fn padded(path: &str, width: usize) -> Vec<u8> {
    let mut bytes = path.as_bytes().to_vec();
    bytes.resize(width, 0);
    bytes
}

/// The expected `RREF` request; `freq = 0` unsubscribes.
fn rref(freq: i32, index: i32, path: &str) -> Vec<u8> {
    [
        b"RREF\0".as_slice(),
        &freq.to_le_bytes(),
        &index.to_le_bytes(),
        &padded(path, 400),
    ]
    .concat()
}

/// An `RREF,` reply carrying `records`.
fn reply(records: &[(i32, f32)]) -> Vec<u8> {
    let mut bytes = b"RREF,".to_vec();
    for (index, value) in records {
        bytes.extend_from_slice(&index.to_le_bytes());
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    bytes
}

fn bytes(datagrams: &[Datagram]) -> Vec<&[u8]> {
    datagrams.iter().map(Datagram::as_bytes).collect()
}

#[test]
fn subscribe_encodes_rref() {
    let mut proto = Protocol::new();
    let (phi, dg) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    assert_eq!(dg.as_bytes(), rref(100, 0, PHI));

    let element = engine::THRO_USE.at(3).unwrap();
    let (_, dg) = proto.subscribe(&element, hz(20)).unwrap();
    assert_eq!(
        dg.as_bytes(),
        rref(20, 1, "sim/flightmodel/engine/ENGN_thro_use[3]")
    );

    let (again, dg) = proto.subscribe(&position::PHI, hz(5)).unwrap();
    assert_eq!(again, phi);
    assert_eq!(dg.as_bytes(), rref(5, 0, PHI));
}

#[test]
fn unsubscribe_encodes_freq_zero_once() {
    let mut proto = Protocol::new();
    let (phi, _) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    assert_eq!(proto.unsubscribe(phi).unwrap().as_bytes(), rref(0, 0, PHI));
    assert_eq!(proto.unsubscribe(phi), None);
}

#[test]
fn write_encodes_dref() {
    let proto = Protocol::new();
    let dref = |value: f32, path: &str| {
        [
            b"DREF\0".as_slice(),
            &value.to_le_bytes(),
            &padded(path, 500),
        ]
        .concat()
    };
    let cases = [
        (proto.write(&position::PHI, -1.5), dref(-1.5, PHI)),
        (
            proto.write(&overrides::THROTTLES, true),
            dref(1.0, "sim/operation/override/override_throttles"),
        ),
        (
            proto.write(&engine::THRO_USE.at(15).unwrap(), 0.5),
            dref(0.5, "sim/flightmodel/engine/ENGN_thro_use[15]"),
        ),
    ];
    for (got, expected) in cases {
        assert_eq!(got.as_bytes(), expected);
    }
}

#[test]
fn command_encodes_cmnd() {
    let dg = Protocol::new().command(&commands::RESET_FLIGHT);
    assert_eq!(dg.as_bytes(), b"CMND\0sim/operation/reset_flight\0");
}

#[test]
fn merges_split_replies() {
    let mut proto = Protocol::new();
    let (phi, _) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    let (theta, _) = proto.subscribe(&position::THETA, hz(100)).unwrap();
    let t0 = Instant::now();
    let t1 = t0 + Duration::from_millis(1);
    proto.handle(&reply(&[(0, 1.0)]), t0);
    proto.handle(&reply(&[(1, 2.0)]), t1);
    let frame = proto.finish().unwrap();
    assert_eq!(frame.get(phi), Some(1.0));
    assert_eq!(frame.get(theta), Some(2.0));
    assert_eq!(frame.received_at(), t1);
    assert_eq!(proto.finish(), None);
    assert_eq!(proto.stats().packets, 2);
}

#[test]
fn counts_malformed_and_keeps_going() {
    let mut proto = Protocol::new();
    let (phi, _) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    let t = Instant::now();
    let truncated = &reply(&[(0, 9.0)])[..12];
    for bad in [b"".as_slice(), b"RREF", b"DATA*\0\0\0", truncated] {
        proto.handle(bad, t);
    }
    proto.handle(&reply(&[(0, 1.0)]), t);
    assert_eq!(proto.finish().unwrap().get(phi), Some(1.0));
    assert_eq!(
        proto.stats(),
        Stats {
            packets: 1,
            malformed: 4,
            ..Stats::default()
        }
    );
}

#[test]
fn drops_records_for_inactive_index() {
    let mut proto = Protocol::new();
    let (phi, _) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    let (theta, _) = proto.subscribe(&position::THETA, hz(100)).unwrap();
    proto.unsubscribe(theta);
    proto.handle(&reply(&[(0, 1.0), (1, 2.0), (99, 3.0)]), Instant::now());
    let frame = proto.finish().unwrap();
    assert_eq!(frame.get(phi), Some(1.0));
    assert_eq!(frame.get(theta), None);
    assert_eq!(proto.stats().dropped_records, 2);
}

#[test]
fn finish_none_when_only_dropped_records() {
    let mut proto = Protocol::new();
    proto.handle(&reply(&[(7, 1.0)]), Instant::now());
    proto.handle(&reply(&[]), Instant::now());
    assert_eq!(proto.finish(), None);
    assert_eq!(
        proto.stats(),
        Stats {
            packets: 2,
            dropped_records: 1,
            ..Stats::default()
        }
    );
}

#[test]
fn close_unsubscribes_exactly_active() {
    let mut proto = Protocol::new();
    let (phi, _) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    let (theta, _) = proto.subscribe(&position::THETA, hz(100)).unwrap();
    let (psi, _) = proto.subscribe(&position::PSI, hz(100)).unwrap();
    proto.unsubscribe(psi);
    let closed = proto.close();
    assert_eq!(bytes(&closed), [rref(0, 0, PHI), rref(0, 1, THETA)]);
    assert!(!proto.subscriptions().is_active(phi));
    assert!(!proto.subscriptions().is_active(theta));
    assert!(proto.close().is_empty());
}

#[test]
fn resubscribe_resends_active_at_current_rate() {
    let mut proto = Protocol::new();
    assert!(proto.resubscribe().is_empty());
    assert_eq!(proto.stats().resubscribes, 1);

    let (phi, _) = proto.subscribe(&position::PHI, hz(10)).unwrap();
    let (theta, _) = proto.subscribe(&position::THETA, hz(100)).unwrap();
    let (psi, _) = proto.subscribe(&position::PSI, hz(100)).unwrap();
    proto.subscribe(&position::PHI, hz(50)).unwrap();
    proto.unsubscribe(psi);
    let resent = proto.resubscribe();
    assert_eq!(bytes(&resent), [rref(50, 0, PHI), rref(100, 1, THETA)]);
    assert_eq!(proto.stats().resubscribes, 2);

    proto.handle(&reply(&[(0, 1.0), (1, 2.0)]), Instant::now());
    let frame = proto.finish().unwrap();
    assert_eq!(frame.get(phi), Some(1.0));
    assert_eq!(frame.get(theta), Some(2.0));
}
