//! `sim/flightmodel/engine/`: per-engine controls.
//!
//! ```
//! use xplane_bridge::datarefs::engine;
//!
//! let first = engine::THRO_USE.at(0).expect("in range");
//! assert_eq!(first.path(), "sim/flightmodel/engine/ENGN_thro_use[0]");
//! ```

use super::{DatarefArray, Writable};

/// Throttle per engine, ratio 0 to 1. Takes effect only while
/// [`overrides::THROTTLES`](super::overrides::THROTTLES) is set.
pub const THRO_USE: DatarefArray<f32, Writable, 16> =
    DatarefArray::from_static("sim/flightmodel/engine/ENGN_thro_use");
