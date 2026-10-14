//! `sim/flightmodel2/wing/`: control surface deflections per wing, in degrees.
//!
//! Index `n` is the wing's `_wing/n` in the model's `.acf`; which wings carry
//! which surfaces is per model. Writes take effect only while
//! [`overrides::CONTROL_SURFACES`](super::overrides::CONTROL_SURFACES) is set,
//! and a written deflection holds until rewritten. The legacy
//! `sim/flightmodel/controls/*_def` datarefs alias these values with other
//! numbering; see [`design.md`](https://github.com/wboayue/xplane-bridge/blob/main/design.md#relationship-to-stable-flight).
//!
//! ```
//! use xplane_bridge::datarefs::wing;
//!
//! let surface = wing::AILERON1_DEG.at(2).expect("in range");
//! assert_eq!(surface.path(), "sim/flightmodel2/wing/aileron1_deg[2]");
//! ```

use super::{DatarefArray, Writable};

/// Aileron set 1, positive trailing edge down.
pub const AILERON1_DEG: DatarefArray<f32, Writable, 48> =
    DatarefArray::from_static("sim/flightmodel2/wing/aileron1_deg");
/// Aileron set 2, positive trailing edge down.
pub const AILERON2_DEG: DatarefArray<f32, Writable, 48> =
    DatarefArray::from_static("sim/flightmodel2/wing/aileron2_deg");
/// Elevator set 1, positive trailing edge down.
pub const ELEVATOR1_DEG: DatarefArray<f32, Writable, 48> =
    DatarefArray::from_static("sim/flightmodel2/wing/elevator1_deg");
/// Elevator set 2, positive trailing edge down.
pub const ELEVATOR2_DEG: DatarefArray<f32, Writable, 48> =
    DatarefArray::from_static("sim/flightmodel2/wing/elevator2_deg");
/// Rudder set 1, positive trailing edge right; the sign mirrors between left
/// and right fins.
pub const RUDDER1_DEG: DatarefArray<f32, Writable, 48> =
    DatarefArray::from_static("sim/flightmodel2/wing/rudder1_deg");
/// Rudder set 2, positive trailing edge right; the sign mirrors between left
/// and right fins.
pub const RUDDER2_DEG: DatarefArray<f32, Writable, 48> =
    DatarefArray::from_static("sim/flightmodel2/wing/rudder2_deg");
