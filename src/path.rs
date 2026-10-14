use std::borrow::Cow;
use std::fmt;

use crate::PathError;
use crate::codec::RREF_PATH_LEN;

/// A validated dataref or command path: the one place path rules live.
///
/// Non-empty ASCII, no NUL, at most [`MAX_LEN`](Self::MAX_LEN) bytes.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub(crate) struct Path(Cow<'static, str>);

impl Path {
    /// The `RREF` path field minus its terminator.
    pub(crate) const MAX_LEN: usize = RREF_PATH_LEN - 1;

    /// Validates a runtime path.
    pub(crate) fn new(path: impl Into<String>) -> Result<Self, PathError> {
        let path = path.into();
        check(&path, 0)?;
        Ok(Self(Cow::Owned(path)))
    }

    /// Constructor for constants. Call it only in a `const` item, so an invalid
    /// path fails the build instead of panicking.
    pub(crate) const fn from_static(path: &'static str) -> Self {
        expect_valid(path, check(path, 0))
    }

    /// Constructor for the base of an array constant of `len` elements: as
    /// [`from_static`](Self::from_static), and the longest element path,
    /// `path[len - 1]`, must fit too. Call it only in a `const` item.
    pub(crate) const fn from_static_array(path: &'static str, len: usize) -> Self {
        if len == 0 {
            panic!("array has no elements");
        }
        if len - 1 > u16::MAX as usize {
            panic!("array has more than 65536 elements");
        }
        expect_valid(path, check(path, element_suffix_len(len - 1)))
    }

    pub(crate) fn as_str(&self) -> &str {
        &self.0
    }

    /// The path as sent: itself, or element `index` of the array it names.
    pub(crate) fn wire(&self, index: Option<u16>) -> WirePath<'_> {
        WirePath { base: self, index }
    }
}

/// A path as sent on the wire: `base`, or `base[i]` for an array element.
///
/// Its `Display` is the one place the element suffix is formatted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct WirePath<'a> {
    base: &'a Path,
    index: Option<u16>,
}

impl fmt::Display for WirePath<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.base.as_str())?;
        match self.index {
            Some(i) => write!(f, "[{i}]"),
            None => Ok(()),
        }
    }
}

/// Length of the `[i]` suffix an array element adds to its base path.
const fn element_suffix_len(index: usize) -> usize {
    let mut digits = 1;
    let mut rest = index / 10;
    while rest > 0 {
        digits += 1;
        rest /= 10;
    }
    digits + 2
}

const fn expect_valid(path: &'static str, checked: Result<(), PathError>) -> Path {
    match checked {
        Ok(()) => Path(Cow::Borrowed(path)),
        Err(PathError::Empty) => panic!("path is empty"),
        Err(PathError::TooLong { .. }) => panic!("path is longer than Path::MAX_LEN"),
        Err(PathError::NotAscii) => panic!("path is not ASCII"),
        Err(PathError::ContainsNul) => panic!("path contains NUL"),
    }
}

// const so `from_static` can run it at compile time. `suffix` is the length of
// an element suffix the path must leave room for.
const fn check(path: &str, suffix: usize) -> Result<(), PathError> {
    let bytes = path.as_bytes();
    if bytes.is_empty() {
        return Err(PathError::Empty);
    }
    if bytes.len() + suffix > Path::MAX_LEN {
        return Err(PathError::TooLong {
            len: bytes.len() + suffix,
            max: Path::MAX_LEN,
        });
    }
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            0 => return Err(PathError::ContainsNul),
            0x80..=0xFF => return Err(PathError::NotAscii),
            _ => {}
        }
        i += 1;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::XPlaneError;

    #[test]
    fn enforces_path_rules() {
        let max = "a".repeat(Path::MAX_LEN);
        let over = "a".repeat(Path::MAX_LEN + 1);
        let cases: &[(&str, Result<(), PathError>)] = &[
            ("sim/flightmodel/position/phi", Ok(())),
            (&max, Ok(())),
            ("", Err(PathError::Empty)),
            (&over, Err(PathError::TooLong { len: 400, max: 399 })),
            ("sim/é", Err(PathError::NotAscii)),
            ("sim/a\0b", Err(PathError::ContainsNul)),
        ];
        for (path, expected) in cases {
            let got = Path::new(*path).map(|p| assert_eq!(p.as_str(), *path));
            assert_eq!(got, *expected, "path {path:?}");
        }
    }

    #[test]
    fn static_constructor_matches_runtime_constructor() {
        const PHI: Path = Path::from_static("sim/flightmodel/position/phi");
        let runtime = Path::new("sim/flightmodel/position/phi").unwrap();
        assert_eq!(PHI, runtime);
    }

    #[test]
    #[should_panic(expected = "path is empty")]
    fn from_static_rejects_invalid_path() {
        Path::from_static("");
    }

    #[test]
    fn formats_wire_path() {
        let base = Path::new("sim/a").unwrap();
        let cases = [
            (None, "sim/a"),
            (Some(0), "sim/a[0]"),
            (Some(65535), "sim/a[65535]"),
        ];
        for (index, expected) in cases {
            assert_eq!(base.wire(index).to_string(), expected);
        }
    }

    #[test]
    fn counts_element_suffix() {
        for (index, len) in [(0, 3), (9, 3), (10, 4), (99, 4), (100, 5), (65535, 7)] {
            assert_eq!(element_suffix_len(index), len, "index {index}");
        }
    }

    #[test]
    fn array_base_leaves_room_for_last_element() {
        // "[0]" for one element, "[10]" for eleven.
        let one = "a".repeat(Path::MAX_LEN - 3).leak();
        let eleven = "a".repeat(Path::MAX_LEN - 4).leak();
        assert_eq!(Path::from_static_array(one, 1).as_str(), one);
        assert_eq!(Path::from_static_array(eleven, 11).as_str(), eleven);
    }

    #[test]
    #[should_panic(expected = "path is longer than Path::MAX_LEN")]
    fn array_base_rejects_no_room_for_last_element() {
        Path::from_static_array("a".repeat(Path::MAX_LEN - 3).leak(), 11);
    }

    #[test]
    #[should_panic(expected = "array has no elements")]
    fn array_base_rejects_empty_array() {
        Path::from_static_array("sim/a", 0);
    }

    #[test]
    #[should_panic(expected = "array has more than 65536 elements")]
    fn array_base_rejects_index_past_u16() {
        Path::from_static_array("sim/a", 65537);
    }

    #[test]
    fn path_error_converts_to_invalid_path() {
        let err = XPlaneError::from(PathError::Empty);
        assert!(matches!(err, XPlaneError::InvalidPath(PathError::Empty)));
        // Cause is reported once, via `source`, not repeated in `Display`.
        assert_eq!(err.to_string(), "invalid path");
        let source = std::error::Error::source(&err).map(ToString::to_string);
        assert_eq!(source.as_deref(), Some("path is empty"));
    }
}
