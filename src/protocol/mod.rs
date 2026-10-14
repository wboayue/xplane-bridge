//! The sans-I/O protocol core: calls become datagrams, received bytes become frames.

use std::time::Instant;

use log::debug;

use crate::codec::{self, Datagram};
use crate::frame::FrameBuilder;
use crate::{Command, Dataref, Frame, Rate, Sub, Subscriptions, Value, Writable, XPlaneError};

/// The whole X-Plane UDP protocol, without a socket.
///
/// Calls return the [`Datagram`]s to send to X-Plane, in order; received
/// datagrams go in through [`handle`](Self::handle), and [`finish`](Self::finish)
/// returns what they carried. A driver only moves bytes between this and its socket.
///
/// ```
/// use std::time::Instant;
/// use xplane_bridge::{Protocol, Rate, datarefs};
///
/// let mut proto = Protocol::new();
/// let rate = Rate::hz(100).expect("nonzero");
/// let (phi, rref) = proto.subscribe(&datarefs::position::PHI, rate)?;
/// assert!(rref.as_bytes().starts_with(b"RREF\0"));
///
/// // X-Plane's reply: `RREF,` then `(index, value)` records. `phi` is index 0.
/// let reply = [b"RREF,".as_slice(), &0_i32.to_le_bytes(), &12.5_f32.to_le_bytes()].concat();
/// proto.handle(&reply, Instant::now());
/// assert_eq!(proto.finish().and_then(|frame| frame.get(phi)), Some(12.5));
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
#[derive(Debug)]
pub struct Protocol {
    subscriptions: Subscriptions,
    frames: FrameBuilder,
    stats: Stats,
}

impl Protocol {
    /// A protocol with no subscriptions.
    pub fn new() -> Self {
        Self {
            subscriptions: Subscriptions::new(),
            frames: FrameBuilder::new(),
            stats: Stats::default(),
        }
    }

    /// Subscribes `dataref` at `rate`: the handle and the `RREF` to send.
    /// Subscribing a dataref again returns its existing handle at the new rate.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::TooManySubscriptions`] if the index space is exhausted.
    ///
    /// ```
    /// use xplane_bridge::{Protocol, Rate, datarefs};
    ///
    /// let mut proto = Protocol::new();
    /// let (phi, _) = proto.subscribe(&datarefs::position::PHI, Rate::hz(10).expect("nonzero"))?;
    /// let (again, _) = proto.subscribe(&datarefs::position::PHI, Rate::hz(50).expect("nonzero"))?;
    /// assert_eq!(phi, again);
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn subscribe<T: Value, A>(
        &mut self,
        dataref: &Dataref<T, A>,
        rate: Rate,
    ) -> Result<(Sub<T>, Datagram), XPlaneError> {
        let sub = self.subscriptions.insert(dataref, rate)?;
        let datagram = codec::subscribe(rate, sub.index(), dataref.wire_path());
        Ok((sub, datagram))
    }

    /// The `RREF` with `freq = 0` that stops `sub`, or `None` if `sub` is not active.
    ///
    /// ```
    /// use xplane_bridge::{Protocol, Rate, datarefs};
    ///
    /// let mut proto = Protocol::new();
    /// let (phi, _) = proto.subscribe(&datarefs::position::PHI, Rate::hz(10).expect("nonzero"))?;
    /// assert!(proto.unsubscribe(phi).is_some());
    /// assert!(proto.unsubscribe(phi).is_none());
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Option<Datagram> {
        let path = self.subscriptions.remove(sub)?;
        Some(codec::unsubscribe(sub.index(), path))
    }

    /// The `DREF` that writes `value` to `dataref`.
    ///
    /// ```
    /// use xplane_bridge::{Protocol, datarefs};
    ///
    /// let dref = Protocol::new().write(&datarefs::overrides::THROTTLES, true);
    /// assert!(dref.as_bytes().starts_with(b"DREF\0"));
    /// ```
    pub fn write<T: Value>(&self, dataref: &Dataref<T, Writable>, value: T) -> Datagram {
        codec::dref(value.to_wire(), dataref.wire_path())
    }

    /// The `CMND` that triggers `command`.
    ///
    /// ```
    /// use xplane_bridge::{Protocol, commands};
    ///
    /// let cmnd = Protocol::new().command(&commands::PAUSE_TOGGLE);
    /// assert_eq!(cmnd.as_bytes(), b"CMND\0sim/operation/pause_toggle\0");
    /// ```
    pub fn command(&self, command: &Command) -> Datagram {
        codec::cmnd(command.path())
    }

    /// One `freq = 0` `RREF` per active subscription, which it removes; for shutdown.
    ///
    /// ```
    /// use xplane_bridge::{Protocol, Rate, datarefs};
    ///
    /// let mut proto = Protocol::new();
    /// let (phi, _) = proto.subscribe(&datarefs::position::PHI, Rate::hz(10).expect("nonzero"))?;
    /// assert_eq!(proto.close().len(), 1);
    /// assert!(!proto.subscriptions().is_active(phi));
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn close(&mut self) -> Vec<Datagram> {
        self.subscriptions
            .drain()
            .map(|(index, path)| codec::unsubscribe(index, path))
            .collect()
    }

    /// The `RREF` for every active subscription, at its index and current rate,
    /// so a restarted X-Plane streams again and handles stay valid. Counts in
    /// [`Stats::resubscribes`], even with no subscriptions.
    ///
    /// ```
    /// use xplane_bridge::{Protocol, Rate, datarefs};
    ///
    /// let mut proto = Protocol::new();
    /// proto.subscribe(&datarefs::position::PHI, Rate::hz(10).expect("nonzero"))?;
    /// assert_eq!(proto.resubscribe().len(), 1);
    /// assert_eq!(proto.stats().resubscribes, 1);
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn resubscribe(&mut self) -> Vec<Datagram> {
        self.stats.resubscribes += 1;
        self.subscriptions
            .iter()
            .map(|(index, path, rate)| codec::subscribe(rate, index, path))
            .collect()
    }

    /// Folds one received datagram, which arrived at `at`, into the pending frame.
    ///
    /// Anything that is not an `RREF,` reply counts in [`Stats::malformed`];
    /// records for an inactive index count in [`Stats::dropped_records`]. Both are
    /// skipped, never errors.
    ///
    /// ```
    /// use std::time::Instant;
    /// use xplane_bridge::Protocol;
    ///
    /// let mut proto = Protocol::new();
    /// proto.handle(b"garbage", Instant::now());
    /// assert_eq!(proto.stats().malformed, 1);
    /// assert!(proto.finish().is_none());
    /// ```
    pub fn handle(&mut self, bytes: &[u8], at: Instant) {
        let records = match codec::decode_rref(bytes) {
            Ok(records) => records,
            Err(err) => {
                debug!("skipping {}-byte datagram: {err}", bytes.len());
                self.stats.malformed += 1;
                return;
            }
        };
        self.stats.packets += 1;
        let Self {
            subscriptions,
            frames,
            stats,
        } = self;
        // Counts as `push` consumes: it reads every record, so none go uncounted.
        let kept = records.filter(|&(index, _)| {
            let active = subscriptions.contains(index);
            if !active {
                stats.dropped_records += 1;
            }
            active
        });
        frames.push(kept, at);
    }

    /// Everything [`handle`](Self::handle)d since the last call, merged, newest
    /// value per subscription; `None` if no record was kept.
    ///
    /// ```
    /// use xplane_bridge::Protocol;
    ///
    /// assert!(Protocol::new().finish().is_none());
    /// ```
    pub fn finish(&mut self) -> Option<Frame> {
        self.frames.finish()
    }

    /// Records that the transport dropped an outgoing datagram (its send buffer
    /// was full), in [`Stats::dropped_sends`].
    ///
    /// ```
    /// use xplane_bridge::Protocol;
    ///
    /// let mut proto = Protocol::new();
    /// proto.dropped_send();
    /// assert_eq!(proto.stats().dropped_sends, 1);
    /// ```
    pub fn dropped_send(&mut self) {
        self.stats.dropped_sends += 1;
    }

    /// Counters since [`new`](Self::new).
    ///
    /// ```
    /// use xplane_bridge::{Protocol, Stats};
    ///
    /// assert_eq!(Protocol::new().stats(), Stats::default());
    /// ```
    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// The active subscriptions.
    ///
    /// ```
    /// use xplane_bridge::{Protocol, Rate, datarefs};
    ///
    /// let mut proto = Protocol::new();
    /// let (phi, _) = proto.subscribe(&datarefs::position::PHI, Rate::hz(10).expect("nonzero"))?;
    /// assert_eq!(proto.subscriptions().get(&datarefs::position::PHI), Some(phi));
    /// # Ok::<(), xplane_bridge::XPlaneError>(())
    /// ```
    pub fn subscriptions(&self) -> &Subscriptions {
        &self.subscriptions
    }
}

impl Default for Protocol {
    fn default() -> Self {
        Self::new()
    }
}

/// Counters since [`Protocol::new`]. Received datagrams are `packets + malformed`.
///
/// ```
/// use xplane_bridge::Stats;
///
/// let stats = Stats::default();
/// assert_eq!(stats.packets + stats.malformed, 0);
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Well-formed `RREF,` replies, kept records or not; shows X-Plane is sending.
    pub packets: u64,
    /// Received datagrams that were not `RREF,` replies, skipped.
    pub malformed: u64,
    /// Records skipped because their index was not an active subscription.
    pub dropped_records: u64,
    /// Calls to [`Protocol::resubscribe`].
    pub resubscribes: u64,
    /// Outgoing datagrams the transport dropped, recorded through
    /// [`Protocol::dropped_send`]. A dropped `RREF` stays missing until
    /// [`Protocol::resubscribe`]; a dropped `DREF` until the next write.
    pub dropped_sends: u64,
}

#[cfg(test)]
mod tests;
