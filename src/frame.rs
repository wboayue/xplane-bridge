//! Values from one drain, and values held across drains.

use std::collections::HashMap;
use std::time::Instant;

use crate::{Sub, Value};

/// Values that arrived in one drain, newest per subscription.
///
/// Holds only what arrived: a subscription absent from it (a rate below the poll
/// rate, or a split reply that skipped it) reads `None`. Use [`Latest`] for held
/// values.
///
/// ```
/// use std::time::Instant;
/// use xplane_bridge::Frame;
///
/// let now = Instant::now();
/// assert_eq!(Frame::empty(now).received_at(), now);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    // Raw wire values; `get` converts with the handle's type.
    values: HashMap<i32, f32>,
    received_at: Instant,
}

impl Frame {
    /// The value for `sub`, or `None` if it did not arrive in this frame.
    pub fn get<T: Value>(&self, sub: Sub<T>) -> Option<T> {
        self.values.get(&sub.index()).copied().map(T::from_wire)
    }

    /// Arrival of the last datagram that added a value.
    pub fn received_at(&self) -> Instant {
        self.received_at
    }

    /// A frame with no values, for fakes and tests; add values with [`with`](Self::with).
    pub fn empty(received_at: Instant) -> Self {
        Self {
            values: HashMap::new(),
            received_at,
        }
    }

    /// This frame with `value` for `sub`, for fakes and tests:
    /// `Frame::empty(t).with(phi, 1.5).with(armed, true)`.
    pub fn with<T: Value>(mut self, sub: Sub<T>, value: T) -> Self {
        self.values.insert(sub.index(), value.to_wire());
        self
    }

    /// Raw `(index, value)` pairs.
    fn values(&self) -> impl Iterator<Item = (i32, f32)> + '_ {
        self.values.iter().map(|(&index, &value)| (index, value))
    }
}

/// Merges the records of one drain into a [`Frame`]; later records win.
///
/// Knows nothing of subscriptions: `Protocol` filters records before pushing.
/// Its map is cleared, not freed, between frames, so pushing allocates only while
/// a frame grows past the largest so far.
#[derive(Debug)]
pub(crate) struct FrameBuilder {
    values: HashMap<i32, f32>,
    received_at: Option<Instant>,
}

#[cfg_attr(not(test), expect(dead_code, reason = "used by Protocol (#12)"))]
impl FrameBuilder {
    pub(crate) fn new() -> Self {
        Self {
            values: HashMap::new(),
            received_at: None,
        }
    }

    /// Folds in one datagram's records, received at `at`. A datagram with no
    /// records leaves the pending frame unchanged.
    pub(crate) fn push(&mut self, records: impl IntoIterator<Item = (i32, f32)>, at: Instant) {
        let mut added = false;
        for (index, value) in records {
            self.values.insert(index, value);
            added = true;
        }
        if added {
            self.received_at = Some(at);
        }
    }

    /// The merged frame, or `None` if no record was pushed since the last call.
    pub(crate) fn finish(&mut self) -> Option<Frame> {
        let received_at = self.received_at.take()?;
        let values = self.values.clone();
        self.values.clear();
        Some(Frame {
            values,
            received_at,
        })
    }
}

/// The last known value of each subscription and when it arrived, across frames.
///
/// Values stay after unsubscribing; indices are never reused, so they are never
/// read as another subscription's.
///
/// ```
/// use std::time::Instant;
/// use xplane_bridge::{Frame, Latest};
///
/// let mut latest = Latest::default();
/// latest.apply(&Frame::empty(Instant::now()));
/// ```
#[derive(Clone, Debug, Default)]
pub struct Latest {
    values: HashMap<i32, (f32, Instant)>,
}

impl Latest {
    /// Records every value in `frame`, stamped with its [`received_at`](Frame::received_at).
    pub fn apply(&mut self, frame: &Frame) {
        for (index, value) in frame.values() {
            self.values.insert(index, (value, frame.received_at));
        }
    }

    /// The last value for `sub` and the arrival time of its frame, or `None` if none
    /// has arrived.
    pub fn get<T: Value>(&self, sub: Sub<T>) -> Option<(T, Instant)> {
        self.values
            .get(&sub.index())
            .map(|&(value, at)| (T::from_wire(value), at))
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use approx::assert_relative_eq;

    use super::*;
    use crate::datarefs::{overrides, position};
    use crate::{Rate, Subscriptions};

    fn hz() -> Rate {
        Rate::hz(10).unwrap()
    }

    /// Handles for roll and pitch, at indices 0 and 1.
    fn subs() -> (Sub<f32>, Sub<f32>) {
        let mut subs = Subscriptions::new();
        let phi = subs.insert(&position::PHI, hz()).unwrap();
        let theta = subs.insert(&position::THETA, hz()).unwrap();
        (phi, theta)
    }

    #[test]
    fn merges_records_across_packets() {
        let (phi, theta) = subs();
        let t = Instant::now();
        let mut builder = FrameBuilder::new();
        builder.push([(phi.index(), 1.0)], t);
        builder.push([(theta.index(), 2.0)], t);
        let frame = builder.finish().unwrap();
        assert_eq!(frame.get(phi), Some(1.0));
        assert_eq!(frame.get(theta), Some(2.0));
    }

    #[test]
    fn newest_record_wins() {
        let (phi, _) = subs();
        let t = Instant::now();
        let mut builder = FrameBuilder::new();
        builder.push([(phi.index(), 1.0), (phi.index(), 2.0)], t);
        assert_eq!(builder.finish().unwrap().get(phi), Some(2.0));

        builder.push([(phi.index(), 1.0)], t);
        builder.push([(phi.index(), 3.0)], t);
        assert_eq!(builder.finish().unwrap().get(phi), Some(3.0));
    }

    #[test]
    fn received_at_is_last_packet_with_records() {
        let (phi, theta) = subs();
        let t0 = Instant::now();
        let (t1, t2) = (t0 + Duration::from_millis(1), t0 + Duration::from_millis(2));
        let mut builder = FrameBuilder::new();
        builder.push([(phi.index(), 1.0)], t0);
        builder.push([(theta.index(), 2.0)], t1);
        builder.push([], t2);
        assert_eq!(builder.finish().unwrap().received_at(), t1);
    }

    #[test]
    fn finish_is_none_without_records() {
        let mut builder = FrameBuilder::new();
        assert_eq!(builder.finish(), None);
        builder.push([], Instant::now());
        assert_eq!(builder.finish(), None);
    }

    #[test]
    fn finish_resets_builder() {
        let (phi, theta) = subs();
        let t = Instant::now();
        let mut builder = FrameBuilder::new();
        builder.push([(phi.index(), 1.0)], t);
        assert!(builder.finish().is_some());
        assert_eq!(builder.finish(), None);
        builder.push([(theta.index(), 2.0)], t);
        assert_eq!(builder.finish().unwrap().get(phi), None);
    }

    #[test]
    fn get_is_none_when_absent() {
        let (phi, theta) = subs();
        let frame = Frame::empty(Instant::now()).with(phi, 1.0);
        assert_eq!(frame.get(theta), None);
    }

    #[test]
    fn with_round_trips_each_value_type() {
        let mut subs = Subscriptions::new();
        let phi = subs.insert(&position::PHI, hz()).unwrap();
        let throttles = subs.insert(&overrides::THROTTLES, hz()).unwrap();
        let count = subs
            .insert(&crate::Dataref::<i32>::new("sim/a").unwrap(), hz())
            .unwrap();
        let frame = Frame::empty(Instant::now())
            .with(phi, -12.5)
            .with(throttles, true)
            .with(count, 7);
        assert_relative_eq!(frame.get(phi).unwrap(), -12.5);
        assert_eq!(frame.get(throttles), Some(true));
        assert_eq!(frame.get(count), Some(7));
    }

    #[test]
    fn latest_holds_values_across_frames() {
        let (phi, theta) = subs();
        let t0 = Instant::now();
        let t1 = t0 + Duration::from_millis(1);
        let mut latest = Latest::default();
        assert_eq!(latest.get(phi), None);

        latest.apply(&Frame::empty(t0).with(phi, 1.0).with(theta, 2.0));
        latest.apply(&Frame::empty(t1).with(phi, 3.0));
        assert_eq!(latest.get(phi), Some((3.0, t1)));
        assert_eq!(latest.get(theta), Some((2.0, t0)));
    }
}
