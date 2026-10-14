//! Expected wire bytes for tests, built from the layouts in `design.md`
//! "Protocol", not from the encoders, so tests check the codec independently.

/// An `RREF` request; `freq = 0` unsubscribes.
pub(crate) fn rref(freq: i32, index: i32, path: &str) -> Vec<u8> {
    [
        b"RREF\0".as_slice(),
        &freq.to_le_bytes(),
        &index.to_le_bytes(),
        &padded(path, 400),
    ]
    .concat()
}

/// A `DREF` writing `value` to `path`.
pub(crate) fn dref(value: f32, path: &str) -> Vec<u8> {
    [
        b"DREF\0".as_slice(),
        &value.to_le_bytes(),
        &padded(path, 500),
    ]
    .concat()
}

/// A `CMND` triggering `path`.
pub(crate) fn cmnd(path: &str) -> Vec<u8> {
    [b"CMND\0".as_slice(), path.as_bytes(), b"\0"].concat()
}

/// An `RREF,` reply carrying `records`.
pub(crate) fn reply(records: &[(i32, f32)]) -> Vec<u8> {
    let mut bytes = b"RREF,".to_vec();
    for (index, value) in records {
        bytes.extend(index.to_le_bytes());
        bytes.extend(value.to_le_bytes());
    }
    bytes
}

fn padded(path: &str, width: usize) -> Vec<u8> {
    let mut bytes = path.as_bytes().to_vec();
    bytes.resize(width, 0);
    bytes
}
