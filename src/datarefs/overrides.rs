//! `sim/operation/override/`: flags that hand control from X-Plane to the caller.
//!
//! Set to `true` to take control, back to `false` to release it; X-Plane does not
//! release them when the bridge stops.
//!
//! ```
//! use xplane_bridge::datarefs::overrides;
//!
//! assert_eq!(overrides::THROTTLES.path(), "sim/operation/override/override_throttles");
//! ```

use super::{Dataref, Writable};

/// Throttles follow [`engine::THRO_USE`](super::engine::THRO_USE).
pub const THROTTLES: Dataref<bool, Writable> =
    Dataref::from_static("sim/operation/override/override_throttles");
/// Stick, yoke, and pedals follow the [`joystick`](super::joystick) ratios;
/// disables auto-coordination.
pub const JOYSTICK: Dataref<bool, Writable> =
    Dataref::from_static("sim/operation/override/override_joystick");
/// Individual control surface deflections are written directly.
pub const CONTROL_SURFACES: Dataref<bool, Writable> =
    Dataref::from_static("sim/operation/override/override_control_surfaces");
