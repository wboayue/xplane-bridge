//! Command names.

use crate::XPlaneError;
use crate::path::Path;

/// A command, triggered with a `CMND`. Its path is validated at construction.
///
/// ```
/// use xplane_bridge::Command;
///
/// let pause = Command::new("sim/operation/pause_toggle")?;
/// assert_eq!(pause.path(), "sim/operation/pause_toggle");
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Command {
    path: Path,
}

impl Command {
    /// A command by runtime path, for anything without a constant. Build it once,
    /// not per tick.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::InvalidPath`] if `path` breaks a path rule (see
    /// [`PathError`](crate::PathError)).
    ///
    /// ```
    /// use xplane_bridge::Command;
    ///
    /// assert!(Command::new("sim/operation/pause_toggle").is_ok());
    /// assert!(Command::new("").is_err());
    /// ```
    pub fn new(path: impl Into<String>) -> Result<Self, XPlaneError> {
        Ok(Self {
            path: Path::new(path)?,
        })
    }

    /// Constructor for constants. Call it only in a `const` item, so an invalid
    /// path fails the build.
    #[cfg_attr(not(test), expect(dead_code, reason = "used by constants (#9)"))]
    pub(crate) const fn from_static(path: &'static str) -> Self {
        Self {
            path: Path::from_static(path),
        }
    }

    /// The command path.
    ///
    /// ```
    /// use xplane_bridge::Command;
    ///
    /// let pause = Command::new("sim/operation/pause_toggle")?;
    /// assert_eq!(pause.path(), "sim/operation/pause_toggle");
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn path(&self) -> &str {
        self.path.as_str()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PathError;

    #[test]
    fn static_constructor_matches_runtime_constructor() {
        const PAUSE: Command = Command::from_static("sim/operation/pause_toggle");
        assert_eq!(PAUSE, Command::new("sim/operation/pause_toggle").unwrap());
    }

    #[test]
    fn new_rejects_invalid_path() {
        let err = Command::new("sim/\0").unwrap_err();
        assert!(matches!(
            err,
            XPlaneError::InvalidPath(PathError::ContainsNul)
        ));
    }
}
