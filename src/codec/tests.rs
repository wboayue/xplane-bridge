use super::test_bytes::{cmnd as cmnd_bytes, dref as dref_bytes, reply, rref};
use super::*;
use crate::path::Path;

const PHI: &str = "sim/flightmodel/position/phi";
const THRO_USE: &str = "sim/flightmodel/engine/ENGN_thro_use";

fn path(s: &str) -> Path {
    Path::new(s).unwrap()
}

fn max_path() -> String {
    "a".repeat(Path::MAX_LEN)
}

/// A base path whose element `index` is exactly `Path::MAX_LEN` bytes.
fn longest_base(index: u16) -> String {
    let suffix = format!("[{index}]").len();
    "a".repeat(Path::MAX_LEN - suffix)
}

/// `(rate in Hz, index, base path, element, expected wire path)`.
type RrefCase<'a> = (Option<u16>, i32, &'a str, Option<u16>, &'a str);

#[test]
fn encodes_rref() {
    let max = max_path();
    let max_base = longest_base(0);
    let max_element = format!("{max_base}[0]");
    // `None` rate: unsubscribe, sent as `freq = 0`.
    let cases: &[RrefCase<'_>] = &[
        (Some(100), 0, PHI, None, PHI),
        (None, 7, PHI, None, PHI),
        (Some(100), -1, PHI, None, PHI),
        (
            Some(50),
            3,
            THRO_USE,
            Some(15),
            "sim/flightmodel/engine/ENGN_thro_use[15]",
        ),
        (Some(1), 1, &max, None, &max),
        (Some(1), 1, &max_base, Some(0), &max_element),
    ];
    for &(hz, index, base, element, wire) in cases {
        let base = path(base);
        let expected = rref(i32::from(hz.unwrap_or(0)), index, wire);
        let wire_path = base.wire(element);
        let got = match hz {
            Some(hz) => subscribe(Rate::hz(hz).unwrap(), index, wire_path),
            None => unsubscribe(index, wire_path),
        };
        assert_eq!(got.as_bytes(), expected, "path {wire}");
        assert_eq!(got.as_bytes().len(), 413);
        assert_eq!(got.as_bytes().last(), Some(&0), "path {wire} is terminated");
    }
}

#[test]
fn encodes_dref() {
    let max = max_path();
    let cases: &[(f32, &str, Option<u16>, &str)] = &[
        (-1.5, PHI, None, PHI),
        (0.0, PHI, None, PHI),
        (
            1.0,
            THRO_USE,
            Some(3),
            "sim/flightmodel/engine/ENGN_thro_use[3]",
        ),
        (f32::MAX, &max, None, &max),
    ];
    for &(value, base, element, wire) in cases {
        let base = path(base);
        let expected = dref_bytes(value, wire);
        let got = dref(value, base.wire(element));
        assert_eq!(got.as_bytes(), expected, "path {wire}");
        assert_eq!(got.as_bytes().len(), 509);
    }
}

#[test]
fn encodes_cmnd() {
    let max = max_path();
    for wire in ["sim/operation/pause_toggle", &max] {
        let expected = cmnd_bytes(wire);
        let got = cmnd(path(wire).wire(None));
        assert_eq!(got.as_bytes(), expected, "path {wire}");
    }
    assert_eq!(cmnd(path(&max).wire(None)).as_bytes().len(), 405);
}

#[test]
fn path_too_long_for_its_field_is_sent_empty() {
    // Unreachable through public API: path rules keep elements within 399 bytes.
    let max = path(&max_path());
    let over = max.wire(Some(5));
    let rref = subscribe(Rate::hz(1).unwrap(), 1, over);
    assert_eq!(rref.as_bytes().len(), 413);
    assert!(rref.as_bytes()[13..].iter().all(|&b| b == 0));
    assert_eq!(cmnd(over).as_bytes(), b"CMND\0\0");
}

#[test]
fn decodes_rref_reply() {
    let full: Vec<_> = (0..183).map(|i| (i, i as f32 * 0.5)).collect();
    let cases: &[(&str, Vec<(i32, f32)>)] = &[
        ("empty", vec![]),
        ("one record", vec![(3, 12.5)]),
        ("raw values", vec![(-1, f32::NAN), (i32::MAX, -0.0)]),
        ("183 records", full),
    ];
    for (name, records) in cases {
        let bytes = reply(records);
        let got: Vec<_> = decode_rref(&bytes).unwrap().collect();
        // Compare bits so NaN and -0.0 are checked exactly.
        let bits = |rs: &[(i32, f32)]| {
            rs.iter()
                .map(|&(i, v)| (i, v.to_bits()))
                .collect::<Vec<_>>()
        };
        assert_eq!(bits(&got), bits(records), "{name}");
        assert_eq!(decode_rref(&bytes).unwrap().len(), records.len(), "{name}");
    }
    assert_eq!(reply(&cases[3].1).len(), 1469);
}

#[test]
fn rejects_malformed_reply() {
    let record = [0u8; 8];
    let cases: &[(&str, Vec<u8>, DecodeError)] = &[
        ("empty", vec![], DecodeError::BadHeader),
        ("short header", b"RRE".to_vec(), DecodeError::BadHeader),
        (
            "request header",
            [b"RREF\0".as_slice(), &record].concat(),
            DecodeError::BadHeader,
        ),
        (
            "other packet",
            [b"DATA*".as_slice(), &record].concat(),
            DecodeError::BadHeader,
        ),
        (
            "partial record",
            [b"RREF,".as_slice(), &record[..7]].concat(),
            DecodeError::Truncated { len: 12 },
        ),
        (
            "extra byte",
            [b"RREF,".as_slice(), &record, &[0]].concat(),
            DecodeError::Truncated { len: 14 },
        ),
    ];
    for (name, bytes, expected) in cases {
        assert_eq!(decode_rref(bytes).err(), Some(*expected), "{name}");
    }
}
