//! Command names.
//!
//! ```
//! use xplane_bridge::commands;
//!
//! assert_eq!(commands::PAUSE_TOGGLE.path(), "sim/operation/pause_toggle");
//! ```

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

/// Toggles pause; read the state from
/// [`datarefs::time::PAUSED`](crate::datarefs::time::PAUSED).
pub const PAUSE_TOGGLE: Command = Command::from_static("sim/operation/pause_toggle");
/// Resets the flight to its most recent start.
pub const RESET_FLIGHT: Command = Command::from_static("sim/operation/reset_flight");

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PathError;
    use crate::xplane_files;

    /// Every command constant. Add new constants here.
    const ALL: [Command; 2] = [PAUSE_TOGGLE, RESET_FLIGHT];

    #[test]
    fn static_constructor_matches_runtime_constructor() {
        assert_eq!(
            PAUSE_TOGGLE,
            Command::new("sim/operation/pause_toggle").unwrap()
        );
    }

    #[test]
    fn all_lists_every_constant() {
        let defined = include_str!("commands.rs")
            .lines()
            .filter(|line| line.starts_with("pub const "))
            .count();
        assert_eq!(ALL.len(), defined);
    }

    #[test]
    fn every_constant_passes_path_rules() {
        for command in ALL {
            assert!(Path::new(command.path()).is_ok(), "path {}", command.path());
        }
    }

    #[test]
    #[ignore = "needs an X-Plane 12 install at $XPLANE_DIR"]
    fn constants_match_commands_txt() {
        let listed = xplane_files::commands();
        for command in ALL {
            assert!(
                listed.iter().any(|c| c == command.path()),
                "{} is not in Commands.txt",
                command.path()
            );
        }
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
