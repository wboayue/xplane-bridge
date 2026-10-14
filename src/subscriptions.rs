//! Subscription handles and index bookkeeping.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

use crate::{Dataref, Rate, Value, XPlaneError};

/// Handle for one active subscription of value type `T`; reads go through it.
///
/// Valid only against frames from the `Protocol` that issued it: every
/// `Protocol` allocates the same indices, so a handle from another one reads the
/// wrong values without error.
///
/// ```
/// use xplane_bridge::{Protocol, Rate, Sub, datarefs};
///
/// let mut proto = Protocol::new();
/// let (paused, _): (Sub<bool>, _) =
///     proto.subscribe(&datarefs::time::PAUSED, Rate::hz(1).expect("nonzero"))?;
/// assert!(proto.subscriptions().is_active(paused));
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
pub struct Sub<T: Value> {
    index: i32,
    _t: PhantomData<fn() -> T>,
}

impl<T: Value> Sub<T> {
    const fn new(index: i32) -> Self {
        Self {
            index,
            _t: PhantomData,
        }
    }

    /// The `RREF` index.
    pub(crate) const fn index(self) -> i32 {
        self.index
    }
}

impl<T: Value> Clone for Sub<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T: Value> Copy for Sub<T> {}

impl<T: Value> fmt::Debug for Sub<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Sub").field(&self.index).finish()
    }
}

impl<T: Value> PartialEq for Sub<T> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

impl<T: Value> Eq for Sub<T> {}

impl<T: Value> Hash for Sub<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.index.hash(state);
    }
}

/// Active subscriptions: which wire path is under which index, at which rate.
///
/// Keyed by wire path, so an array element and the same `path[i]` built by hand
/// share one index, as do one path under two value types. Indices are never
/// reused, so a late packet for a removed subscription is never read as a new one.
/// Read it through [`Protocol::subscriptions`](crate::Protocol::subscriptions).
///
/// ```
/// use xplane_bridge::{Protocol, Rate, datarefs};
///
/// let mut proto = Protocol::new();
/// let (phi, _) = proto.subscribe(&datarefs::position::PHI, Rate::hz(10).expect("nonzero"))?;
/// assert_eq!(proto.subscriptions().get(&datarefs::position::PHI), Some(phi));
/// proto.unsubscribe(phi);
/// assert!(!proto.subscriptions().is_active(phi));
/// # Ok::<(), xplane_bridge::XPlaneError>(())
/// ```
#[derive(Debug)]
pub struct Subscriptions {
    // In index order, so `iter` and `drain` are deterministic. Path lookups scan:
    // they happen only on subscribe, over a few dozen entries.
    by_index: BTreeMap<i32, (Cow<'static, str>, Rate)>,
    next: i32,
}

impl Subscriptions {
    pub(crate) fn new() -> Self {
        Self {
            by_index: BTreeMap::new(),
            next: 0,
        }
    }

    /// The existing handle for `dataref`'s wire path, its rate updated, or a new one.
    pub(crate) fn insert<T: Value, A>(
        &mut self,
        dataref: &Dataref<T, A>,
        rate: Rate,
    ) -> Result<Sub<T>, XPlaneError> {
        let key = dataref.key();
        if let Some((&index, entry)) = self.by_index.iter_mut().find(|(_, (path, _))| *path == key)
        {
            entry.1 = rate;
            return Ok(Sub::new(index));
        }
        let index = self.next;
        // Leaves `i32::MAX` unused so `next` never overflows; the space is still
        // far beyond any real subscription count.
        self.next = index
            .checked_add(1)
            .ok_or(XPlaneError::TooManySubscriptions)?;
        self.by_index.insert(index, (key, rate));
        Ok(Sub::new(index))
    }

    /// Removes `sub`, returning its wire path, or `None` if it is not active.
    pub(crate) fn remove<T: Value>(&mut self, sub: Sub<T>) -> Option<Cow<'static, str>> {
        self.by_index.remove(&sub.index).map(|(path, _)| path)
    }

    /// Removes every subscription, in index order.
    pub(crate) fn drain(&mut self) -> impl Iterator<Item = (i32, Cow<'static, str>)> + use<> {
        std::mem::take(&mut self.by_index)
            .into_iter()
            .map(|(index, (path, _))| (index, path))
    }

    /// Every subscription with its wire path and rate, in index order.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (i32, &str, Rate)> + '_ {
        self.by_index
            .iter()
            .map(|(&index, (path, rate))| (index, path.as_ref(), *rate))
    }

    /// Whether a received record's `index` belongs to an active subscription.
    pub(crate) fn contains(&self, index: i32) -> bool {
        self.by_index.contains_key(&index)
    }

    /// Whether `sub` is subscribed.
    pub fn is_active<T: Value>(&self, sub: Sub<T>) -> bool {
        self.contains(sub.index)
    }

    /// The handle for `dataref`'s wire path, or `None` if it is not subscribed.
    pub fn get<T: Value, A>(&self, dataref: &Dataref<T, A>) -> Option<Sub<T>> {
        let key = dataref.key();
        self.by_index
            .iter()
            .find(|(_, (path, _))| *path == key)
            .map(|(&index, _)| Sub::new(index))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Writable;
    use crate::datarefs::{engine, overrides, position};

    fn hz(hz: u16) -> Rate {
        Rate::hz(hz).unwrap()
    }

    #[test]
    fn reuses_handle_and_updates_rate_on_resubscribe() {
        let mut subs = Subscriptions::new();
        let first = subs.insert(&position::PHI, hz(10)).unwrap();
        let again = subs.insert(&position::PHI, hz(50)).unwrap();
        assert_eq!(first, again);
        let listed: Vec<_> = subs.iter().collect();
        assert_eq!(listed, [(0, "sim/flightmodel/position/phi", hz(50))]);
    }

    #[test]
    fn array_element_and_hand_built_path_share_index() {
        let by_hand =
            Dataref::<f32, Writable>::new("sim/flightmodel/engine/ENGN_thro_use[0]").unwrap();
        let mut subs = Subscriptions::new();
        let element = subs
            .insert(&engine::THRO_USE.at(0).unwrap(), hz(10))
            .unwrap();
        assert_eq!(subs.insert(&by_hand, hz(10)).unwrap(), element);
        assert_eq!(subs.iter().count(), 1);
    }

    #[test]
    fn one_path_under_two_value_types_shares_index() {
        let as_bool = Dataref::<bool>::new("sim/flightmodel/position/phi").unwrap();
        let mut subs = Subscriptions::new();
        let float = subs.insert(&position::PHI, hz(10)).unwrap();
        let boolean = subs.insert(&as_bool, hz(10)).unwrap();
        assert_eq!(float.index(), boolean.index());
        assert_eq!(subs.iter().count(), 1);
    }

    #[test]
    fn never_reuses_index_after_remove() {
        let mut subs = Subscriptions::new();
        let phi = subs.insert(&position::PHI, hz(10)).unwrap();
        assert_eq!(
            subs.remove(phi).as_deref(),
            Some("sim/flightmodel/position/phi")
        );
        let again = subs.insert(&position::PHI, hz(10)).unwrap();
        assert_ne!(phi, again);
        assert!(!subs.is_active(phi));
        assert!(subs.is_active(again));
    }

    #[test]
    fn remove_is_none_when_inactive() {
        let mut subs = Subscriptions::new();
        let phi = subs.insert(&position::PHI, hz(10)).unwrap();
        assert!(subs.remove(phi).is_some());
        assert_eq!(subs.remove(phi), None);
        assert_eq!(subs.remove(Sub::<f32>::new(99)), None);
    }

    #[test]
    fn get_is_active_and_contains_track_membership() {
        let mut subs = Subscriptions::new();
        assert_eq!(subs.get(&position::PHI), None);
        let phi = subs.insert(&position::PHI, hz(10)).unwrap();
        assert_eq!(subs.get(&position::PHI), Some(phi));
        assert!(subs.is_active(phi));
        assert!(subs.contains(phi.index()));
        subs.remove(phi);
        assert_eq!(subs.get(&position::PHI), None);
        assert!(!subs.is_active(phi));
        assert!(!subs.contains(phi.index()));
    }

    #[test]
    fn iter_lists_index_path_rate_in_order() {
        let mut subs = Subscriptions::new();
        subs.insert(&position::PHI, hz(10)).unwrap();
        subs.insert(&overrides::THROTTLES, hz(1)).unwrap();
        subs.insert(&engine::THRO_USE.at(3).unwrap(), hz(20))
            .unwrap();
        let listed: Vec<_> = subs.iter().collect();
        assert_eq!(
            listed,
            [
                (0, "sim/flightmodel/position/phi", hz(10)),
                (1, "sim/operation/override/override_throttles", hz(1)),
                (2, "sim/flightmodel/engine/ENGN_thro_use[3]", hz(20)),
            ]
        );
    }

    #[test]
    fn drain_returns_all_in_order_and_empties() {
        let mut subs = Subscriptions::new();
        let phi = subs.insert(&position::PHI, hz(10)).unwrap();
        subs.insert(&engine::THRO_USE.at(0).unwrap(), hz(10))
            .unwrap();
        let drained: Vec<_> = subs.drain().collect();
        assert_eq!(
            drained,
            [
                (0, Cow::Borrowed("sim/flightmodel/position/phi")),
                (1, Cow::Borrowed("sim/flightmodel/engine/ENGN_thro_use[0]")),
            ]
        );
        assert_eq!(subs.iter().count(), 0);
        assert_eq!(subs.get(&position::PHI), None);
        assert!(!subs.is_active(phi));
        // Draining does not restart allocation.
        assert_eq!(subs.insert(&position::PHI, hz(10)).unwrap().index(), 2);
    }

    #[test]
    fn errs_when_index_space_exhausted() {
        let mut subs = Subscriptions::new();
        subs.next = i32::MAX;
        let err = subs.insert(&position::PHI, hz(10)).unwrap_err();
        assert!(matches!(err, XPlaneError::TooManySubscriptions));
        assert_eq!(subs.iter().count(), 0);
    }
}
