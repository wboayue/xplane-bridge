//! A scripted X-Plane for testing consumers without a simulator. Feature `testing`.

use std::collections::VecDeque;
use std::io;

use crate::{
    Command, Dataref, DatarefSink, DatarefSource, Frame, Protocol, Rate, Stats, Sub, Subscriptions,
    Value, Writable, XPlaneError,
};

/// A [`DatarefSource`] and [`DatarefSink`] that replays queued frames and records
/// what is sent.
///
/// Subscriptions go through a real [`Protocol`], so handles, re-subscribe reuse,
/// and [`Stats::resubscribes`] behave as with X-Plane. Frames are returned as
/// queued, bypassing [`Protocol::handle`], so `Stats::packets`,
/// `Stats::malformed`, and `Stats::dropped_records` stay 0.
///
/// ```
/// use std::time::Instant;
/// use xplane_bridge::testing::FakeXPlane;
/// use xplane_bridge::{DatarefSource, Frame, Rate, datarefs};
///
/// let mut fake = FakeXPlane::new();
/// let phi = fake.subscribe(&datarefs::position::PHI, Rate::hz(100).expect("nonzero"))?;
/// fake.push_frame(Frame::empty(Instant::now()).with(phi, 12.5));
///
/// assert_eq!(fake.poll()?.and_then(|frame| frame.get(phi)), Some(12.5));
/// assert_eq!(fake.poll()?, None);
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
#[derive(Debug, Default)]
pub struct FakeXPlane {
    proto: Protocol,
    // `None` is a poll with nothing new.
    polls: VecDeque<Option<Frame>>,
    writes: Vec<(String, f32)>,
    commands: Vec<String>,
    error: Option<io::ErrorKind>,
}

impl FakeXPlane {
    /// A fake with no subscriptions and nothing queued.
    pub fn new() -> Self {
        Self::default()
    }

    /// Queues `frame` for a later [`poll`](DatarefSource::poll). Build it with
    /// handles from [`subscriptions`](Self::subscriptions) or `subscribe`.
    pub fn push_frame(&mut self, frame: Frame) {
        self.polls.push_back(Some(frame));
    }

    /// Queues a poll that returns `None`, as when X-Plane sent nothing. Polling
    /// with nothing queued does the same.
    ///
    /// ```
    /// use std::time::Instant;
    /// use xplane_bridge::testing::FakeXPlane;
    /// use xplane_bridge::{DatarefSource, Frame};
    ///
    /// let mut fake = FakeXPlane::new();
    /// fake.push_silence();
    /// fake.push_frame(Frame::empty(Instant::now()));
    /// assert!(fake.poll()?.is_none());
    /// assert!(fake.poll()?.is_some());
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn push_silence(&mut self) {
        self.polls.push_back(None);
    }

    /// While `Some`, every trait call returns [`XPlaneError::Io`] of `kind` and
    /// changes nothing: no frame is consumed, nothing is recorded. `None` clears it.
    ///
    /// ```
    /// use std::io::ErrorKind;
    /// use xplane_bridge::testing::FakeXPlane;
    /// use xplane_bridge::{DatarefSource, XPlaneError};
    ///
    /// let mut fake = FakeXPlane::new();
    /// fake.fail_with(Some(ErrorKind::ConnectionRefused));
    /// assert!(matches!(fake.poll(), Err(XPlaneError::Io(_))));
    /// fake.fail_with(None);
    /// assert!(fake.poll().is_ok());
    /// ```
    pub fn fail_with(&mut self, kind: Option<io::ErrorKind>) {
        self.error = kind;
    }

    /// Every write as `(wire path, wire value)`, oldest first.
    ///
    /// ```
    /// use xplane_bridge::testing::FakeXPlane;
    /// use xplane_bridge::{DatarefSink, datarefs};
    ///
    /// let mut fake = FakeXPlane::new();
    /// fake.write(&datarefs::overrides::THROTTLES, true)?;
    /// fake.write(&datarefs::overrides::THROTTLES, false)?;
    /// assert_eq!(
    ///     fake.writes(),
    ///     [
    ///         ("sim/operation/override/override_throttles".to_string(), 1.0),
    ///         ("sim/operation/override/override_throttles".to_string(), 0.0),
    ///     ]
    /// );
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn writes(&self) -> &[(String, f32)] {
        &self.writes
    }

    /// The last value written to `dataref`'s wire path, or `None` if none was.
    ///
    /// ```
    /// use xplane_bridge::testing::FakeXPlane;
    /// use xplane_bridge::{DatarefSink, datarefs};
    ///
    /// let mut fake = FakeXPlane::new();
    /// let engine = datarefs::engine::THRO_USE.at(0).expect("in range");
    /// assert_eq!(fake.last_write(&engine), None);
    /// fake.write(&engine, 0.55)?;
    /// assert_eq!(fake.last_write(&engine), Some(0.55));
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn last_write<T: Value>(&self, dataref: &Dataref<T, Writable>) -> Option<T> {
        let path = dataref.path();
        self.writes
            .iter()
            .rev()
            .find(|(written, _)| *written == path)
            .map(|&(_, value)| T::from_wire(value))
    }

    /// Every command's path, oldest first.
    ///
    /// ```
    /// use xplane_bridge::testing::FakeXPlane;
    /// use xplane_bridge::{DatarefSink, commands};
    ///
    /// let mut fake = FakeXPlane::new();
    /// fake.command(&commands::PAUSE_TOGGLE)?;
    /// assert_eq!(fake.commands(), ["sim/operation/pause_toggle"]);
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn commands(&self) -> &[String] {
        &self.commands
    }

    /// The active subscriptions, for handles to build frames with.
    ///
    /// ```
    /// use xplane_bridge::testing::FakeXPlane;
    /// use xplane_bridge::{DatarefSource, Rate, datarefs};
    ///
    /// let mut fake = FakeXPlane::new();
    /// let phi = fake.subscribe(&datarefs::position::PHI, Rate::hz(10).expect("nonzero"))?;
    /// assert_eq!(fake.subscriptions().get(&datarefs::position::PHI), Some(phi));
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn subscriptions(&self) -> &Subscriptions {
        self.proto.subscriptions()
    }

    /// The scripted error, if one is set.
    fn check(&self) -> Result<(), XPlaneError> {
        match self.error {
            Some(kind) => Err(io::Error::from(kind).into()),
            None => Ok(()),
        }
    }
}

impl DatarefSource for FakeXPlane {
    fn subscribe<T: Value, A>(
        &mut self,
        dataref: &Dataref<T, A>,
        rate: Rate,
    ) -> Result<Sub<T>, XPlaneError> {
        self.check()?;
        let (sub, _) = self.proto.subscribe(dataref, rate)?;
        Ok(sub)
    }

    fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Result<(), XPlaneError> {
        self.check()?;
        self.proto.unsubscribe(sub);
        Ok(())
    }

    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError> {
        self.check()?;
        Ok(self.polls.pop_front().flatten())
    }

    fn resubscribe(&mut self) -> Result<(), XPlaneError> {
        self.check()?;
        self.proto.resubscribe();
        Ok(())
    }

    fn stats(&self) -> Stats {
        self.proto.stats()
    }
}

impl DatarefSink for FakeXPlane {
    fn write<T: Value>(
        &mut self,
        dataref: &Dataref<T, Writable>,
        value: T,
    ) -> Result<(), XPlaneError> {
        self.check()?;
        self.writes
            .push((dataref.path().into_owned(), value.to_wire()));
        Ok(())
    }

    fn command(&mut self, command: &Command) -> Result<(), XPlaneError> {
        self.check()?;
        self.commands.push(command.path().to_owned());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::io::ErrorKind;
    use std::time::Instant;

    use super::*;
    use crate::commands;
    use crate::datarefs::{engine, overrides, position};

    fn hz() -> Rate {
        Rate::hz(10).unwrap()
    }

    #[test]
    fn is_send_for_consumers_that_move_bridges_across_threads() {
        fn assert_send<T: Send>() {}
        assert_send::<FakeXPlane>();
    }

    #[test]
    fn poll_returns_queued_frames_then_silence() {
        let mut fake = FakeXPlane::new();
        let phi = fake.subscribe(&position::PHI, hz()).unwrap();
        let t = Instant::now();
        fake.push_frame(Frame::empty(t).with(phi, 1.0));
        fake.push_silence();
        fake.push_frame(Frame::empty(t).with(phi, 2.0));

        let polled: Vec<_> = (0..4)
            .map(|_| fake.poll().unwrap().and_then(|f| f.get(phi)))
            .collect();
        assert_eq!(polled, [Some(1.0), None, Some(2.0), None]);
    }

    #[test]
    fn records_writes_and_commands_in_order() {
        let mut fake = FakeXPlane::new();
        let engine0 = engine::THRO_USE.at(0).unwrap();
        fake.write(&overrides::THROTTLES, true).unwrap();
        fake.write(&engine0, 0.5).unwrap();
        fake.command(&commands::PAUSE_TOGGLE).unwrap();
        fake.write(&overrides::THROTTLES, false).unwrap();
        fake.command(&commands::RESET_FLIGHT).unwrap();

        let throttles = "sim/operation/override/override_throttles";
        let expected = [
            (throttles, 1.0),
            ("sim/flightmodel/engine/ENGN_thro_use[0]", 0.5),
            (throttles, 0.0),
        ];
        let writes: Vec<_> = fake
            .writes()
            .iter()
            .map(|(path, value)| (path.as_str(), *value))
            .collect();
        assert_eq!(writes, expected);
        assert_eq!(
            fake.commands(),
            ["sim/operation/pause_toggle", "sim/operation/reset_flight"]
        );
    }

    #[test]
    fn last_write_reads_typed_value_per_path() {
        let mut fake = FakeXPlane::new();
        let (engine0, engine1) = (
            engine::THRO_USE.at(0).unwrap(),
            engine::THRO_USE.at(1).unwrap(),
        );
        fake.write(&engine0, 0.25).unwrap();
        fake.write(&engine1, 0.75).unwrap();
        fake.write(&overrides::THROTTLES, true).unwrap();
        fake.write(&engine0, 0.5).unwrap();
        assert_eq!(fake.last_write(&engine0), Some(0.5));
        assert_eq!(fake.last_write(&engine1), Some(0.75));
        assert_eq!(fake.last_write(&overrides::THROTTLES), Some(true));
        assert_eq!(fake.last_write(&overrides::JOYSTICK), None);
    }

    #[test]
    fn failure_blocks_every_call_until_cleared() {
        let mut fake = FakeXPlane::new();
        let phi = fake.subscribe(&position::PHI, hz()).unwrap();
        fake.push_frame(Frame::empty(Instant::now()).with(phi, 1.0));
        fake.fail_with(Some(ErrorKind::ConnectionReset));

        let is_reset = |r: Result<(), XPlaneError>| matches!(r, Err(XPlaneError::Io(e)) if e.kind() == ErrorKind::ConnectionReset);
        assert!(is_reset(fake.subscribe(&position::THETA, hz()).map(drop)));
        assert!(is_reset(fake.unsubscribe(phi)));
        assert!(is_reset(fake.poll().map(drop)));
        assert!(is_reset(fake.resubscribe()));
        assert!(is_reset(fake.write(&overrides::THROTTLES, true)));
        assert!(is_reset(fake.command(&commands::PAUSE_TOGGLE)));

        fake.fail_with(None);
        assert!(fake.subscriptions().is_active(phi));
        assert_eq!(fake.subscriptions().get(&position::THETA), None);
        assert!(fake.writes().is_empty());
        assert!(fake.commands().is_empty());
        assert_eq!(fake.stats().resubscribes, 0);
        assert_eq!(fake.poll().unwrap().and_then(|f| f.get(phi)), Some(1.0));
    }

    #[test]
    fn resubscribe_counts_in_stats_and_keeps_handles() {
        let mut fake = FakeXPlane::new();
        let phi = fake.subscribe(&position::PHI, hz()).unwrap();
        fake.resubscribe().unwrap();
        fake.resubscribe().unwrap();
        assert_eq!(fake.stats().resubscribes, 2);
        assert!(fake.subscriptions().is_active(phi));
        assert_eq!(fake.subscribe(&position::PHI, hz()).unwrap(), phi);

        fake.unsubscribe(phi).unwrap();
        fake.unsubscribe(phi).unwrap();
        assert!(!fake.subscriptions().is_active(phi));
    }

    /// A decorator in the shape of stable-flight's `RecordingBridge`: counts
    /// polls and writes, forwards everything.
    struct Recording<B> {
        inner: B,
        polls: usize,
        writes: usize,
    }

    impl<B: DatarefSource> DatarefSource for Recording<B> {
        fn subscribe<T: Value, A>(
            &mut self,
            dataref: &Dataref<T, A>,
            rate: Rate,
        ) -> Result<Sub<T>, XPlaneError> {
            self.inner.subscribe(dataref, rate)
        }

        fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Result<(), XPlaneError> {
            self.inner.unsubscribe(sub)
        }

        fn poll(&mut self) -> Result<Option<Frame>, XPlaneError> {
            self.polls += 1;
            self.inner.poll()
        }

        fn resubscribe(&mut self) -> Result<(), XPlaneError> {
            self.inner.resubscribe()
        }

        fn stats(&self) -> Stats {
            self.inner.stats()
        }
    }

    impl<B: DatarefSink> DatarefSink for Recording<B> {
        fn write<T: Value>(
            &mut self,
            dataref: &Dataref<T, Writable>,
            value: T,
        ) -> Result<(), XPlaneError> {
            self.writes += 1;
            self.inner.write(dataref, value)
        }

        fn command(&mut self, command: &Command) -> Result<(), XPlaneError> {
            self.inner.command(command)
        }
    }

    /// Stands in for a consumer that needs both halves.
    fn tick(
        bridge: &mut (impl DatarefSource + DatarefSink),
        phi: Sub<f32>,
    ) -> Result<Option<f32>, XPlaneError> {
        bridge.write(&overrides::THROTTLES, true)?;
        Ok(bridge.poll()?.and_then(|f| f.get(phi)))
    }

    #[test]
    fn wrapper_composes() {
        let mut bridge = Recording {
            inner: FakeXPlane::new(),
            polls: 0,
            writes: 0,
        };
        let phi = bridge.subscribe(&position::PHI, hz()).unwrap();
        bridge
            .inner
            .push_frame(Frame::empty(Instant::now()).with(phi, 3.0));

        assert_eq!(tick(&mut bridge, phi).unwrap(), Some(3.0));
        assert_eq!(tick(&mut bridge, phi).unwrap(), None);
        assert_eq!((bridge.polls, bridge.writes), (2, 2));
        assert_eq!(bridge.inner.last_write(&overrides::THROTTLES), Some(true));
    }
}
