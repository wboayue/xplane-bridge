//! `sim/flightmodel/position/`: attitude, rates, and position.
//!
//! Local coordinates are X-Plane's OpenGL frame: x east, y up, z south, origin at
//! [`LAT_REF`]/[`LON_REF`].
//!
//! ```
//! use xplane_bridge::datarefs::position;
//!
//! assert_eq!(position::PHI.path(), "sim/flightmodel/position/phi");
//! ```

use super::{Dataref, DatarefArray, ReadOnly, Writable};

/// Roll, degrees, positive right wing down.
pub const PHI: Dataref<f32, Writable> = Dataref::from_static("sim/flightmodel/position/phi");
/// Pitch, degrees, positive nose up.
pub const THETA: Dataref<f32, Writable> = Dataref::from_static("sim/flightmodel/position/theta");
/// True heading, degrees, clockwise from north.
pub const PSI: Dataref<f32, Writable> = Dataref::from_static("sim/flightmodel/position/psi");

/// Roll rate about the body x axis, deg/s, positive right wing down.
pub const P: Dataref<f32, Writable> = Dataref::from_static("sim/flightmodel/position/P");
/// Pitch rate about the body y axis, deg/s, positive nose up.
pub const Q: Dataref<f32, Writable> = Dataref::from_static("sim/flightmodel/position/Q");
/// Yaw rate about the body z axis, deg/s, positive nose right.
pub const R: Dataref<f32, Writable> = Dataref::from_static("sim/flightmodel/position/R");

/// Local x (east), meters. `double` in X-Plane, sent as `f32`: steps of about
/// 1 mm at 10 km from the origin.
pub const LOCAL_X: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_x");
/// Local y (up), meters. `double` in X-Plane, sent as `f32`: steps of about
/// 1 mm at 10 km from the origin.
pub const LOCAL_Y: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_y");
/// Local z (south), meters. `double` in X-Plane, sent as `f32`: steps of about
/// 1 mm at 10 km from the origin.
pub const LOCAL_Z: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_z");

/// Local velocity x (east), m/s.
pub const LOCAL_VX: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_vx");
/// Local velocity y (up), m/s.
pub const LOCAL_VY: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_vy");
/// Local velocity z (south), m/s.
pub const LOCAL_VZ: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_vz");

/// Local acceleration x (east), m/s².
pub const LOCAL_AX: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_ax");
/// Local acceleration y (up), m/s².
pub const LOCAL_AY: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_ay");
/// Local acceleration z (south), m/s².
pub const LOCAL_AZ: Dataref<f32, Writable> =
    Dataref::from_static("sim/flightmodel/position/local_az");

/// Latitude, degrees, positive north. `double` in X-Plane, sent as `f32`: steps
/// of about 0.4 m at 47°, too coarse for control; use [`LOCAL_X`]/[`LOCAL_Z`].
pub const LATITUDE: Dataref<f32, ReadOnly> =
    Dataref::from_static("sim/flightmodel/position/latitude");
/// Longitude, degrees, positive east. `double` in X-Plane, sent as `f32`: steps
/// of up to about 0.8 m, too coarse for control; use [`LOCAL_X`]/[`LOCAL_Z`].
pub const LONGITUDE: Dataref<f32, ReadOnly> =
    Dataref::from_static("sim/flightmodel/position/longitude");
/// Elevation above MSL, meters. `double` in X-Plane, sent as `f32`: steps of
/// about 1 mm at 8 km.
pub const ELEVATION: Dataref<f32, ReadOnly> =
    Dataref::from_static("sim/flightmodel/position/elevation");

/// Latitude of the local frame's origin, degrees.
pub const LAT_REF: Dataref<f32, ReadOnly> =
    Dataref::from_static("sim/flightmodel/position/lat_ref");
/// Longitude of the local frame's origin, degrees.
pub const LON_REF: Dataref<f32, ReadOnly> =
    Dataref::from_static("sim/flightmodel/position/lon_ref");

/// Attitude as a quaternion `[w, x, y, z]`: rotation from north-east-down to
/// the body frame, the same yaw-pitch-roll as [`PSI`], [`THETA`], [`PHI`]. Not
/// relative to the OpenGL local frame.
pub const QUATERNION: DatarefArray<f32, Writable, 4> =
    DatarefArray::from_static("sim/flightmodel/position/q");
