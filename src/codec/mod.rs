//! Packet encode and decode: bytes only, no state. Layouts are in `design.md`
//! "Protocol".

use std::slice::ChunksExact;

use crate::path::WirePath;

mod datagram;

pub use datagram::Datagram;

const HEADER_LEN: usize = 5;
/// Width of the null-padded `RREF` path field.
pub(crate) const RREF_PATH_LEN: usize = 400;
/// Width of the null-padded `DREF` path field.
const DREF_PATH_LEN: usize = 500;
const RREF_LEN: usize = HEADER_LEN + 4 + 4 + RREF_PATH_LEN;
const DREF_LEN: usize = HEADER_LEN + 4 + DREF_PATH_LEN;
const RECORD_LEN: usize = 8;

const RREF_HEADER: &[u8; HEADER_LEN] = b"RREF\0";
const DREF_HEADER: &[u8; HEADER_LEN] = b"DREF\0";
const CMND_HEADER: &[u8; HEADER_LEN] = b"CMND\0";
const RREF_REPLY_HEADER: &[u8; HEADER_LEN] = b"RREF,";

/// Why a received datagram is not an `RREF,` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub(crate) enum DecodeError {
    /// It does not start with `RREF,`, including anything shorter than that.
    #[error("not an RREF reply")]
    BadHeader,
    /// Its length is not the header plus whole 8-byte records.
    #[error("RREF reply is {len} bytes, not 5 + 8k")]
    Truncated { len: usize },
}

/// Subscribe `path` under `index` at `freq` per second; `freq = 0` unsubscribes.
pub(crate) fn rref(freq: u16, index: i32, path: WirePath<'_>) -> Datagram {
    Datagram::build(|w| {
        w.put(RREF_HEADER);
        w.put(&i32::from(freq).to_le_bytes());
        w.put(&index.to_le_bytes());
        w.put_path(path);
        w.pad_to(RREF_LEN);
    })
}

/// Write `value` to `path`.
pub(crate) fn dref(value: f32, path: WirePath<'_>) -> Datagram {
    Datagram::build(|w| {
        w.put(DREF_HEADER);
        w.put(&value.to_le_bytes());
        w.put_path(path);
        w.pad_to(DREF_LEN);
    })
}

/// Trigger the command at `path`.
pub(crate) fn cmnd(path: WirePath<'_>) -> Datagram {
    Datagram::build(|w| {
        w.put(CMND_HEADER);
        w.put_path(path);
        w.put(&[0]);
    })
}

/// The `(index, value)` records of an `RREF,` reply, in packet order.
pub(crate) fn decode_rref(bytes: &[u8]) -> Result<Records<'_>, DecodeError> {
    let body = bytes
        .strip_prefix(RREF_REPLY_HEADER.as_slice())
        .ok_or(DecodeError::BadHeader)?;
    if body.len() % RECORD_LEN != 0 {
        return Err(DecodeError::Truncated { len: bytes.len() });
    }
    Ok(Records(body.chunks_exact(RECORD_LEN)))
}

/// Raw `(index, value)` records; values are converted by the caller.
pub(crate) struct Records<'a>(ChunksExact<'a, u8>);

impl Iterator for Records<'_> {
    type Item = (i32, f32);

    fn next(&mut self) -> Option<Self::Item> {
        let (index, value) = self.0.next()?.split_first_chunk::<4>()?;
        let value = value.first_chunk::<4>()?;
        Some((i32::from_le_bytes(*index), f32::from_le_bytes(*value)))
    }
}

#[cfg(test)]
mod tests;
