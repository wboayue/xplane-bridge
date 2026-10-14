use std::fmt::{self, Write as _};

use super::DREF_LEN;

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

/// Appends to a zeroed buffer. Nothing written ever spills past its own field.
pub(super) struct Writer<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Writer<'_> {
    /// Appends `bytes`, or nothing if they do not fit.
    pub(super) fn put(&mut self, bytes: &[u8]) -> bool {
        let end = self.len + bytes.len();
        let Some(dst) = self.buf.get_mut(self.len..end) else {
            return false;
        };
        dst.copy_from_slice(bytes);
        self.len = end;
        true
    }

    /// Writes `text` into a field of exactly `width` bytes, null-padded with at
    /// least one terminator.
    pub(super) fn put_padded(&mut self, text: impl fmt::Display, width: usize) {
        self.put_bounded(text, width.saturating_sub(1));
        self.advance(width);
    }

    /// Writes `text` and a terminator, taking at most `max` bytes in all.
    pub(super) fn put_terminated(&mut self, text: impl fmt::Display, max: usize) {
        let written = self.put_bounded(text, max.saturating_sub(1));
        self.advance(written + 1);
    }

    /// Writes `text` within the next `max` bytes without advancing, and returns
    /// its length. Text that does not fit leaves the field empty, never
    /// truncated; path rules keep that from happening.
    fn put_bounded(&mut self, text: impl fmt::Display, max: usize) -> usize {
        let end = (self.len + max).min(self.buf.len());
        let Some(dst) = self.buf.get_mut(self.len..end) else {
            return 0;
        };
        let mut field = Writer { buf: dst, len: 0 };
        if write!(field, "{text}").is_err() {
            field.buf.fill(0);
            return 0;
        }
        field.len
    }

    fn advance(&mut self, n: usize) {
        self.len = (self.len + n).min(self.buf.len());
    }
}

impl fmt::Write for Writer<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        if self.put(s.as_bytes()) {
            Ok(())
        } else {
            Err(fmt::Error)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn equality_ignores_bytes_past_len() {
        let mut a = Datagram::build(|w| {
            w.put(b"CMND\0");
        });
        let b = Datagram::build(|w| {
            w.put(b"CMND\0");
        });
        a.buf[CAP - 1] = 0xFF;
        assert_eq!(a, b);
        let c = Datagram::build(|w| {
            w.put(b"CMND\0x");
        });
        assert_ne!(a, c);
    }

    #[test]
    fn drops_write_that_does_not_fit() {
        let d = Datagram::build(|w| {
            assert!(w.put(&[1; CAP - 1]));
            assert!(!w.put(&[2, 2]));
            assert!(w.put(&[3]));
        });
        assert_eq!(d.as_bytes().len(), CAP);
        assert_eq!(d.as_bytes().last(), Some(&3));
    }

    #[test]
    fn keeps_text_inside_its_field() {
        type Put = fn(&mut Writer<'_>, &str, usize);
        let padded: Put = |w, text, width| w.put_padded(text, width);
        let terminated: Put = |w, text, max| w.put_terminated(text, max);
        let cases: &[(&str, Put, &str, &[u8])] = &[
            ("padded fits", padded, "ab", b"ab\0\0|"),
            ("padded exactly fits", padded, "abc", b"abc\0|"),
            ("padded too long", padded, "abcd", b"\0\0\0\0|"),
            ("terminated fits", terminated, "ab", b"ab\0|"),
            ("terminated exactly fits", terminated, "abc", b"abc\0|"),
            ("terminated too long", terminated, "abcd", b"\0|"),
        ];
        for &(name, put, text, expected) in cases {
            let d = Datagram::build(|w| {
                put(w, text, 4);
                w.put(b"|");
            });
            assert_eq!(d.as_bytes(), expected, "{name}");
        }
    }
}
