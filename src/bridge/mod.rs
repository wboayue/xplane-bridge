//! The traits consumers depend on, so they never name a driver.

use crate::{Command, Dataref, Frame, Rate, Stats, Sub, Value, Writable, XPlaneError};

/// Reads simulator state: subscriptions and the frames they produce.
///
/// For static dispatch; take `impl DatarefSource`, never a driver type.
///
/// ```
/// use xplane_bridge::testing::FakeXPlane;
/// use xplane_bridge::{DatarefSource, Rate, XPlaneError, datarefs};
///
/// /// Roll from the next frame, if one arrived.
/// fn roll(source: &mut impl DatarefSource) -> Result<Option<f32>, XPlaneError> {
///     let phi = source.subscribe(&datarefs::position::PHI, Rate::hz(100).expect("nonzero"))?;
///     Ok(source.poll()?.and_then(|frame| frame.get(phi)))
/// }
///
/// assert_eq!(roll(&mut FakeXPlane::new())?, None);
/// # Ok::<(), XPlaneError>(())
/// ```
pub trait DatarefSource {
    /// Subscribes one dataref at `rate`; one call per dataref keeps each handle's
    /// type. Subscribing again returns the existing handle at the new rate.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] if the request cannot be sent;
    /// [`XPlaneError::TooManySubscriptions`] if the index space is exhausted.
    fn subscribe<T: Value, A>(
        &mut self,
        dataref: &Dataref<T, A>,
        rate: Rate,
    ) -> Result<Sub<T>, XPlaneError>;

    /// Stops streaming `sub`. An unknown or already-removed handle is ignored.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] if the request cannot be sent.
    fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Result<(), XPlaneError>;

    /// Everything that arrived since the last call, merged; `None` if nothing did.
    /// Never blocks.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] on a receive failure. Malformed input is never an error.
    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError>;

    /// Resends every active subscription, for a restarted or late-started X-Plane.
    /// Handles stay valid; repeating it is harmless.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] if a request cannot be sent.
    fn resubscribe(&mut self) -> Result<(), XPlaneError>;

    /// Counters since the source was created.
    fn stats(&self) -> Stats;
}

/// Drives the aircraft: dataref writes and commands, fire-and-forget.
///
/// For static dispatch; take `impl DatarefSink`, never a driver type.
///
/// ```
/// use xplane_bridge::testing::FakeXPlane;
/// use xplane_bridge::{DatarefSink, XPlaneError, datarefs};
///
/// /// Takes over the engines and sets every throttle.
/// fn throttle(sink: &mut impl DatarefSink, value: f32) -> Result<(), XPlaneError> {
///     sink.write(&datarefs::overrides::THROTTLES, true)?;
///     for engine in datarefs::engine::THRO_USE.iter() {
///         sink.write(&engine, value)?;
///     }
///     Ok(())
/// }
///
/// let mut fake = FakeXPlane::new();
/// throttle(&mut fake, 0.5)?;
/// assert_eq!(fake.last_write(&datarefs::overrides::THROTTLES), Some(true));
/// # Ok::<(), XPlaneError>(())
/// ```
pub trait DatarefSink {
    /// Writes `value` to `dataref`; only [`Writable`] datarefs compile.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] if the `DREF` cannot be sent.
    fn write<T: Value>(
        &mut self,
        dataref: &Dataref<T, Writable>,
        value: T,
    ) -> Result<(), XPlaneError>;

    /// Triggers `command`.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] if the `CMND` cannot be sent.
    fn command(&mut self, command: &Command) -> Result<(), XPlaneError>;
}
