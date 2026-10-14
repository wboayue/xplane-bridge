use super::*;
use crate::path::Path;

const PHI: &str = "sim/flightmodel/position/phi";
const THRO_USE: &str = "sim/flightmodel/engine/ENGN_thro_use";

fn path(s: &str) -> Path {
    Path::new(s).unwrap()
}

fn padded(path: &str, width: usize) -> Vec<u8> {
    let mut bytes = path.as_bytes().to_vec();
    bytes.resize(width, 0);
    bytes
}

/// A base path whose element `index` is exactly `Path::MAX_LEN` bytes.
fn longest_base(index: u16) -> String {
    let suffix = format!("[{index}]").len();
    "a".repeat(Path::MAX_LEN - suffix)
}

#[test]
fn encodes_rref() {
    let max = "a".repeat(Path::MAX_LEN);
    let max_base = longest_base(0);
    let max_element = format!("{max_base}[0]");
    let cases: &[(u16, i32, &str, Option<u16>, &str)] = &[
        (100, 0, PHI, None, PHI),
        (0, 7, PHI, None, PHI),
        (100, -1, PHI, None, PHI),
        (
            50,
            3,
            THRO_USE,
            Some(15),
            "sim/flightmodel/engine/ENGN_thro_use[15]",
        ),
        (1, 1, &max, None, &max),
        (1, 1, &max_base, Some(0), &max_element),
    ];
    for &(freq, index, base, element, wire) in cases {
        let base = path(base);
        let expected = [
            b"RREF\0".as_slice(),
            &i32::from(freq).to_le_bytes(),
            &index.to_le_bytes(),
            &padded(wire, 400),
        ]
        .concat();
        let got = rref(freq, index, base.wire(element));
        assert_eq!(got.as_bytes(), expected, "path {wire}");
        assert_eq!(got.as_bytes().len(), 413);
        assert_eq!(got.as_bytes().last(), Some(&0), "path {wire} is terminated");
    }
}

#[test]
fn encodes_dref() {
    let max = "a".repeat(Path::MAX_LEN);
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
        let expected = [
            b"DREF\0".as_slice(),
            &value.to_le_bytes(),
            &padded(wire, 500),
        ]
        .concat();
        let got = dref(value, base.wire(element));
        assert_eq!(got.as_bytes(), expected, "path {wire}");
        assert_eq!(got.as_bytes().len(), 509);
    }
}

#[test]
fn encodes_cmnd() {
    let max = "a".repeat(Path::MAX_LEN);
    for wire in ["sim/operation/pause_toggle", &max] {
        let expected = [b"CMND\0".as_slice(), wire.as_bytes(), b"\0"].concat();
        let got = cmnd(path(wire).wire(None));
        assert_eq!(got.as_bytes(), expected, "path {wire}");
    }
    assert_eq!(cmnd(path(&max).wire(None)).as_bytes().len(), 405);
}

fn reply(records: &[(i32, f32)]) -> Vec<u8> {
    let mut bytes = b"RREF,".to_vec();
    for (index, value) in records {
        bytes.extend(index.to_le_bytes());
        bytes.extend(value.to_le_bytes());
    }
    bytes
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
