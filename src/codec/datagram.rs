use std::fmt::{self, Write as _};

use super::DREF_LEN;
use crate::path::WirePath;

/// The largest packet, `DREF`.
const CAP: usize = DREF_LEN;

/// One outgoing UDP payload, ready to send. A fixed buffer; no heap.
#[derive(Clone, Copy)]
pub struct Datagram {
    buf: [u8; CAP],
    len: usize,
}

impl Datagram {
    /// A datagram from what `fill` writes; unwritten bytes are zero.
    pub(super) fn build(fill: impl FnOnce(&mut Writer<'_>)) -> Self {
        let mut buf = [0; CAP];
        let mut writer = Writer {
            buf: &mut buf,
            len: 0,
        };
        fill(&mut writer);
        let len = writer.len;
        Self { buf, len }
    }

    /// The payload to send.
    pub fn as_bytes(&self) -> &[u8] {
        self.buf.get(..self.len).unwrap_or_default()
    }
}

impl PartialEq for Datagram {
    fn eq(&self, other: &Self) -> bool {
        self.as_bytes() == other.as_bytes()
    }
}

impl Eq for Datagram {}

impl fmt::Debug for Datagram {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Datagram")
            .field(&format_args!("b\"{}\"", self.as_bytes().escape_ascii()))
            .finish()
    }
}

/// Appends to a datagram's buffer. A write that does not fit is dropped whole;
/// path rules make every packet fit, so that never happens.
pub(super) struct Writer<'a> {
    buf: &'a mut [u8; CAP],
    len: usize,
}

impl Writer<'_> {
    pub(super) fn put(&mut self, bytes: &[u8]) {
        let end = self.len + bytes.len();
        if let Some(dst) = self.buf.get_mut(self.len..end) {
            dst.copy_from_slice(bytes);
            self.len = end;
        }
    }

    pub(super) fn put_path(&mut self, path: WirePath<'_>) {
        // Only fails on overflow, which path rules rule out.
        let _ = write!(self, "{path}");
    }

    /// Extends to `len` with the zeros already in the buffer: null padding.
    pub(super) fn pad_to(&mut self, len: usize) {
        self.len = self.len.max(len.min(CAP));
    }
}

impl fmt::Write for Writer<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let before = self.len;
        self.put(s.as_bytes());
        if self.len == before && !s.is_empty() {
            return Err(fmt::Error);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn equality_ignores_bytes_past_len() {
        let mut a = Datagram::build(|w| w.put(b"CMND\0"));
        let b = Datagram::build(|w| w.put(b"CMND\0"));
        a.buf[CAP - 1] = 0xFF;
        assert_eq!(a, b);
        assert_ne!(a, Datagram::build(|w| w.put(b"CMND\0x")));
    }

    #[test]
    fn drops_write_that_does_not_fit() {
        let d = Datagram::build(|w| {
            w.put(&[1; CAP - 1]);
            w.put(&[2, 2]);
            w.put(&[3]);
        });
        assert_eq!(d.as_bytes().len(), CAP);
        assert_eq!(d.as_bytes().last(), Some(&3));
    }
}
