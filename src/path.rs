use std::borrow::Cow;

use crate::PathError;

/// A validated dataref or command path: the one place path rules live.
///
/// Non-empty ASCII, no NUL, at most [`MAX_LEN`](Self::MAX_LEN) bytes.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub(crate) struct Path(Cow<'static, str>);

#[cfg_attr(
    not(test),
    expect(dead_code, reason = "used by Dataref and Command (#7)")
)]
impl Path {
    /// The 400-byte `RREF` path field minus its terminator.
    pub(crate) const MAX_LEN: usize = 399;

    /// Validates a runtime path.
    pub(crate) fn new(path: impl Into<String>) -> Result<Self, PathError> {
        let path = path.into();
        check(&path)?;
        Ok(Self(Cow::Owned(path)))
    }

    /// Constructor for constants; use through [`static_path!`], which forces const
    /// evaluation so an invalid path fails the build instead of panicking.
    pub(crate) const fn from_static(path: &'static str) -> Self {
        match check(path) {
            Ok(()) => Self(Cow::Borrowed(path)),
            Err(PathError::Empty) => panic!("path is empty"),
            Err(PathError::TooLong { .. }) => panic!("path is longer than Path::MAX_LEN"),
            Err(PathError::NotAscii) => panic!("path is not ASCII"),
            Err(PathError::ContainsNul) => panic!("path contains NUL"),
        }
    }

    pub(crate) fn as_str(&self) -> &str {
        &self.0
    }
}

/// A [`Path`] from a `&'static str` const expression, checked at compile time.
#[cfg_attr(not(test), expect(unused_macros, reason = "used by constants (#9)"))]
macro_rules! static_path {
    ($path:expr) => {
        const { $crate::path::Path::from_static($path) }
    };
}
#[expect(unused_imports, reason = "used by constants (#9)")]
pub(crate) use static_path;

// const so `from_static` can run it at compile time.
const fn check(path: &str) -> Result<(), PathError> {
    let bytes = path.as_bytes();
    if bytes.is_empty() {
        return Err(PathError::Empty);
    }
    if bytes.len() > Path::MAX_LEN {
        return Err(PathError::TooLong {
            len: bytes.len(),
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
    fn macro_matches_runtime_constructor() {
        const PHI: Path = static_path!("sim/flightmodel/position/phi");
        const PHI_CONCAT: Path = static_path!(concat!("sim/flightmodel/position/", "phi"));
        let runtime = Path::new("sim/flightmodel/position/phi").unwrap();
        assert_eq!(PHI, runtime);
        assert_eq!(PHI_CONCAT, runtime);
    }

    #[test]
    #[should_panic(expected = "path is empty")]
    fn from_static_rejects_invalid_path() {
        Path::from_static("");
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
