//! Typed dataref names.
//!
//! A [`Dataref`] carries its value type and access in its type, so reading
//! returns the right Rust type and writing a read-only dataref, or a value of the
//! wrong type, does not compile.

use std::borrow::Cow;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

use crate::XPlaneError;
use crate::path::{Path, WirePath};

pub mod engine;
pub mod joystick;
pub mod overrides;
pub mod position;
pub mod time;
mod value;

pub use value::Value;

/// Access marker: the dataref can only be read. The default.
pub enum ReadOnly {}

/// Access marker: the dataref can be written with a `DREF`.
///
/// Writing needs `Writable` and a value of the dataref's type:
///
/// ```
/// use xplane_bridge::{Dataref, Value, Writable};
///
/// // Stands in for the bridge's write call, which has the same bounds.
/// fn write<T: Value>(_: &Dataref<T, Writable>, _: T) {}
///
/// let throttles = Dataref::<bool, Writable>::new("sim/operation/override/override_throttles")?;
/// write(&throttles, true);
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
///
/// A read-only dataref does not compile:
///
/// ```compile_fail,E0308
/// use xplane_bridge::{Dataref, Value, Writable};
///
/// fn write<T: Value>(_: &Dataref<T, Writable>, _: T) {}
///
/// let latitude = Dataref::<f32>::new("sim/flightmodel/position/latitude")?;
/// write(&latitude, 47.0);
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
///
/// Nor does a value of the wrong type:
///
/// ```compile_fail,E0308
/// use xplane_bridge::{Dataref, Value, Writable};
///
/// fn write<T: Value>(_: &Dataref<T, Writable>, _: T) {}
///
/// let throttles = Dataref::<bool, Writable>::new("sim/operation/override/override_throttles")?;
/// write(&throttles, 1.0_f32);
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
pub enum Writable {}

/// A dataref of value type `T`: a scalar, or one element of an array.
///
/// The path is validated at construction, so a `Dataref` always fits a packet.
///
/// ```
/// use xplane_bridge::{Dataref, Writable};
///
/// let roll = Dataref::<f32, Writable>::new("sim/flightmodel/position/phi")?;
/// assert_eq!(roll.path(), "sim/flightmodel/position/phi");
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
pub struct Dataref<T: Value, A = ReadOnly> {
    path: Path,
    index: Option<u16>,
    _t: PhantomData<fn() -> (T, A)>,
}

impl<T: Value, A> Dataref<T, A> {
    /// A dataref by runtime path, for anything without a constant. Build it once,
    /// not per tick.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::InvalidPath`] if `path` breaks a path rule (see
    /// [`PathError`](crate::PathError)).
    ///
    /// ```
    /// use xplane_bridge::Dataref;
    ///
    /// let airspeed = Dataref::<f32>::new("sim/flightmodel/position/indicated_airspeed")?;
    /// assert!(Dataref::<f32>::new("").is_err());
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn new(path: impl Into<String>) -> Result<Self, XPlaneError> {
        Ok(Self::from_path(Path::new(path)?, None))
    }

    /// Constructor for constants. Call it only in a `const` item, so an invalid
    /// path fails the build.
    pub(crate) const fn from_static(path: &'static str) -> Self {
        Self::from_path(Path::from_static(path), None)
    }

    const fn from_path(path: Path, index: Option<u16>) -> Self {
        Self {
            path,
            index,
            _t: PhantomData,
        }
    }

    /// The wire path: `path`, or `path[i]` for an array element.
    ///
    /// ```
    /// use xplane_bridge::Dataref;
    ///
    /// let pitch = Dataref::<f32>::new("sim/flightmodel/position/theta")?;
    /// assert_eq!(pitch.path(), "sim/flightmodel/position/theta");
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn path(&self) -> Cow<'_, str> {
        match self.index {
            None => Cow::Borrowed(self.path.as_str()),
            Some(_) => Cow::Owned(self.wire_path().to_string()),
        }
    }

    /// The wire path, for encoding without allocating.
    pub(crate) fn wire_path(&self) -> WirePath<'_> {
        self.path.wire(self.index)
    }
}

impl<T: Value, A> Clone for Dataref<T, A> {
    fn clone(&self) -> Self {
        Self::from_path(self.path.clone(), self.index)
    }
}

impl<T: Value, A> fmt::Debug for Dataref<T, A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Dataref")
            .field("path", &self.path.as_str())
            .field("index", &self.index)
            .finish()
    }
}

impl<T: Value, A> PartialEq for Dataref<T, A> {
    fn eq(&self, other: &Self) -> bool {
        self.path == other.path && self.index == other.index
    }
}

impl<T: Value, A> Eq for Dataref<T, A> {}

impl<T: Value, A> Hash for Dataref<T, A> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.path.hash(state);
        self.index.hash(state);
    }
}

/// A fixed-length array dataref of `N` elements, such as `ENGN_thro_use[16]`.
///
/// Its constants are checked at build time so every element path fits a packet.
pub struct DatarefArray<T: Value, A, const N: usize> {
    base: Path,
    _t: PhantomData<fn() -> (T, A)>,
}

impl<T: Value, A, const N: usize> DatarefArray<T, A, N> {
    /// Constructor for constants. Call it only in a `const` item, so an invalid
    /// path, `N = 0`, or `N > 65536` fails the build.
    pub(crate) const fn from_static(base: &'static str) -> Self {
        Self {
            base: Path::from_static_array(base, N),
            _t: PhantomData,
        }
    }

    /// Element `i`, or `None` if `i >= N`.
    pub fn at(&self, i: usize) -> Option<Dataref<T, A>> {
        if i >= N {
            return None;
        }
        let index = u16::try_from(i).ok()?;
        Some(Dataref::from_path(self.base.clone(), Some(index)))
    }

    /// All `N` elements in order.
    pub fn iter(&self) -> impl Iterator<Item = Dataref<T, A>> + use<T, A, N> {
        // Owns a clone of the base so the iterator outlives a constant's temporary.
        let base = self.base.clone();
        (0..N).filter_map(move |i| {
            let index = u16::try_from(i).ok()?;
            Some(Dataref::from_path(base.clone(), Some(index)))
        })
    }
}

impl<T: Value, A, const N: usize> Clone for DatarefArray<T, A, N> {
    fn clone(&self) -> Self {
        Self {
            base: self.base.clone(),
            _t: PhantomData,
        }
    }
}

impl<T: Value, A, const N: usize> fmt::Debug for DatarefArray<T, A, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DatarefArray")
            .field("base", &self.base.as_str())
            .field("len", &N)
            .finish()
    }
}

impl<T: Value, A, const N: usize> PartialEq for DatarefArray<T, A, N> {
    fn eq(&self, other: &Self) -> bool {
        self.base == other.base
    }
}

impl<T: Value, A, const N: usize> Eq for DatarefArray<T, A, N> {}

impl<T: Value, A, const N: usize> Hash for DatarefArray<T, A, N> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.base.hash(state);
    }
}

#[cfg(test)]
mod constants_tests;
#[cfg(test)]
mod tests;
