use std::num::NonZeroU16;

/// Requested `RREF` rate, in Hz.
///
/// Nonzero: `freq = 0` means unsubscribe on the wire. X-Plane caps the delivered
/// rate at its frame rate whatever is asked.
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
