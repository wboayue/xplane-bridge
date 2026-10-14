//! `sim/time/`: simulator clock and pause state.
//!
//! ```
//! use xplane_bridge::datarefs::time;
//!
//! assert_eq!(time::PAUSED.path(), "sim/time/paused");
//! ```

use super::{Dataref, ReadOnly, Writable};

/// Whether the simulator is paused. Change it with
/// [`commands::PAUSE_TOGGLE`](crate::commands::PAUSE_TOGGLE).
pub const PAUSED: Dataref<bool, ReadOnly> = Dataref::from_static("sim/time/paused");
/// Seconds since the simulator started.
pub const TOTAL_RUNNING_TIME_SEC: Dataref<f32, Writable> =
    Dataref::from_static("sim/time/total_running_time_sec");
/// Smoothed seconds per simulator frame; the frame rate is its reciprocal.
pub const FRAMERATE_PERIOD: Dataref<f32, ReadOnly> =
    Dataref::from_static("sim/time/framerate_period");
