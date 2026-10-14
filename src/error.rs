/// Errors returned by bridge calls.
///
/// On the per-tick path (`poll`, `write`) only [`Io`](Self::Io) is possible.
#[derive(Debug, thiserror::Error)]
pub enum XPlaneError {
    /// Socket bind, send, or receive failed.
    #[error("socket I/O: {0}")]
    Io(#[from] std::io::Error),
    /// A runtime dataref or command path breaks a path rule.
    #[error("invalid path: {0}")]
    InvalidPath(#[from] PathError),
    /// The `i32` subscription index space is exhausted.
    #[error("subscription index space exhausted")]
    TooManySubscriptions,
}

/// Why a dataref or command path was rejected.
///
/// A path must be non-empty ASCII with no NUL, at most 399 bytes, so it fits
/// the 400-byte `RREF` path field with a terminator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum PathError {
    /// The path is empty.
    #[error("path is empty")]
    Empty,
    /// The path is longer than `max` bytes.
    #[error("path is {len} bytes, max {max}")]
    TooLong {
        /// Length of the rejected path, in bytes.
        len: usize,
        /// Maximum accepted length, in bytes.
        max: usize,
    },
    /// The path contains a non-ASCII byte.
    #[error("path is not ASCII")]
    NotAscii,
    /// The path contains a NUL byte, which would end it early on the wire.
    #[error("path contains NUL")]
    ContainsNul,
}
