//! `sim/joystick/`: pilot stick and pedal inputs.
//!
//! Writes take effect only while
//! [`overrides::JOYSTICK`](super::overrides::JOYSTICK) is set.
//!
//! ```
//! use xplane_bridge::datarefs::joystick;
//!
//! assert_eq!(joystick::YOKE_ROLL_RATIO.path(), "sim/joystick/yoke_roll_ratio");
//! ```

use super::{Dataref, Writable};

/// Roll input, −1 (full left) to 1 (full right).
pub const YOKE_ROLL_RATIO: Dataref<f32, Writable> =
    Dataref::from_static("sim/joystick/yoke_roll_ratio");
/// Pitch input, −1 (full down) to 1 (full up).
pub const YOKE_PITCH_RATIO: Dataref<f32, Writable> =
    Dataref::from_static("sim/joystick/yoke_pitch_ratio");
/// Yaw input, −1 (full left) to 1 (full right).
pub const YOKE_HEADING_RATIO: Dataref<f32, Writable> =
    Dataref::from_static("sim/joystick/yoke_heading_ratio");
