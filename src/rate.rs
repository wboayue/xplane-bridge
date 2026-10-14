use std::num::NonZeroU16;

/// Requested `RREF` rate, in Hz.
///
/// Nonzero: `freq = 0` means unsubscribe on the wire. X-Plane sends at most once
/// per frame, on the first frame at least `1/hz` after the last send, so the
/// delivered rate rounds down to whole frames; for every frame, ask for at least
/// twice the frame rate.
///
/// ```
/// use xplane_bridge::Rate;
///
/// assert!(Rate::hz(100).is_some());
/// assert!(Rate::hz(0).is_none());
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate(NonZeroU16);

impl Rate {
    /// `hz` per second, or `None` if `hz` is 0.
    ///
    /// ```
    /// use xplane_bridge::Rate;
    ///
    /// const FAST: Rate = Rate::hz(100).expect("nonzero");
    /// # let _ = FAST;
    /// ```
    pub const fn hz(hz: u16) -> Option<Self> {
        match NonZeroU16::new(hz) {
            Some(hz) => Some(Self(hz)),
            None => None,
        }
    }

    pub(crate) const fn get(self) -> u16 {
        self.0.get()
    }
}

/// A nonzero rate, for tests.
#[cfg(test)]
pub(crate) fn hz(hz: u16) -> Rate {
    Rate::hz(hz).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_zero() {
        assert_eq!(Rate::hz(0), None);
        assert!(Rate::hz(1).is_some());
        assert!(Rate::hz(u16::MAX).is_some());
    }
}
