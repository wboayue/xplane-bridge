use std::time::{Duration, Instant};

use super::*;
use crate::codec::test_bytes::{cmnd, dref, reply, rref};
use crate::commands;
use crate::datarefs::{engine, overrides, position};
use crate::rate::hz;

fn bytes(datagrams: &[Datagram]) -> Vec<&[u8]> {
    datagrams.iter().map(Datagram::as_bytes).collect()
}

#[test]
fn subscribe_encodes_rref() {
    let mut proto = Protocol::new();
    let (phi, dg) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    assert_eq!(dg.as_bytes(), rref(100, 0, &position::PHI.path()));

    let element = engine::THRO_USE.at(3).unwrap();
    let (_, dg) = proto.subscribe(&element, hz(20)).unwrap();
    assert_eq!(
        dg.as_bytes(),
        rref(20, 1, "sim/flightmodel/engine/ENGN_thro_use[3]")
    );

    let (again, dg) = proto.subscribe(&position::PHI, hz(5)).unwrap();
    assert_eq!(again, phi);
    assert_eq!(dg.as_bytes(), rref(5, 0, &position::PHI.path()));
}

#[test]
fn unsubscribe_encodes_freq_zero_once() {
    let mut proto = Protocol::new();
    let (phi, _) = proto.subscribe(&position::PHI, hz(100)).unwrap();
    assert_eq!(
        proto.unsubscribe(phi).unwrap().as_bytes(),
        rref(0, 0, &position::PHI.path())
    );
    assert_eq!(proto.unsubscribe(phi), None);
}

#[test]
fn write_encodes_dref() {
    let proto = Protocol::new();
    let cases = [
        (
            proto.write(&position::PHI, -1.5),
            dref(-1.5, &position::PHI.path()),
        ),
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
    assert_eq!(dg.as_bytes(), cmnd(commands::RESET_FLIGHT.path()));
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
    assert_eq!(
        bytes(&closed),
        [
            rref(0, 0, &position::PHI.path()),
            rref(0, 1, &position::THETA.path())
        ]
    );
    assert!(!proto.subscriptions().is_active(phi));
    assert!(!proto.subscriptions().is_active(theta));
    assert!(proto.close().is_empty());
}

#[test]
fn counts_dropped_sends() {
    let mut proto = Protocol::new();
    proto.dropped_send();
    proto.dropped_send();
    assert_eq!(proto.stats().dropped_sends, 2);
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
    assert_eq!(
        bytes(&resent),
        [
            rref(50, 0, &position::PHI.path()),
            rref(100, 1, &position::THETA.path())
        ]
    );
    assert_eq!(proto.stats().resubscribes, 2);

    proto.handle(&reply(&[(0, 1.0), (1, 2.0)]), Instant::now());
    let frame = proto.finish().unwrap();
    assert_eq!(frame.get(phi), Some(1.0));
    assert_eq!(frame.get(theta), Some(2.0));
}
