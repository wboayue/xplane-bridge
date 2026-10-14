/// A value type a dataref can hold: `f32`, `i32`, or `bool`.
///
/// The wire carries every value as `f32`; this converts. Sealed.
///
/// ```
/// use xplane_bridge::Value;
///
/// assert_eq!(i32::from_wire(2.6), 3);
/// assert_eq!(true.to_wire(), 1.0);
/// ```
pub trait Value: Copy + sealed::Sealed {
    /// The value as sent in a `DREF`.
    fn to_wire(self) -> f32;
    /// The value from an `RREF,` record.
    fn from_wire(raw: f32) -> Self;
}

mod sealed {
    pub trait Sealed {}
    impl Sealed for f32 {}
    impl Sealed for i32 {}
    impl Sealed for bool {}
}

impl Value for f32 {
    fn to_wire(self) -> f32 {
        self
    }

    fn from_wire(raw: f32) -> Self {
        raw
    }
}

/// Exact up to ±2^24. Reading rounds half away from zero, saturates out of
/// range, and maps NaN to 0.
impl Value for i32 {
    fn to_wire(self) -> f32 {
        self as f32
    }

    fn from_wire(raw: f32) -> Self {
        raw.round() as i32
    }
}

/// Writes `1.0`/`0.0`. Reading is `raw != 0.0`: `-0.0` is `false`, NaN is `true`.
impl Value for bool {
    fn to_wire(self) -> f32 {
        if self { 1.0 } else { 0.0 }
    }

    fn from_wire(raw: f32) -> Self {
        raw != 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn f32_passes_through() {
        for v in [0.0, -0.0, -12.5, 3.4e38] {
            assert_relative_eq!(f32::from_wire(v.to_wire()), v);
        }
    }

    #[test]
    fn i32_round_trips_and_rounds() {
        for v in [0, 3, -3, 1 << 24, -(1 << 24)] {
            assert_eq!(i32::from_wire(v.to_wire()), v, "value {v}");
        }
        let reads = [
            (2.4, 2),
            (2.5, 3),
            (-2.5, -3),
            (f32::NAN, 0),
            (1e10, i32::MAX),
            (-1e10, i32::MIN),
        ];
        for (raw, expected) in reads {
            assert_eq!(i32::from_wire(raw), expected, "raw {raw}");
        }
    }

    #[test]
    fn bool_writes_one_or_zero_and_reads_nonzero() {
        assert_relative_eq!(true.to_wire(), 1.0);
        assert_relative_eq!(false.to_wire(), 0.0);
        let reads = [
            (0.0, false),
            (-0.0, false),
            (1.0, true),
            (0.5, true),
            (-1.0, true),
            (f32::NAN, true),
        ];
        for (raw, expected) in reads {
            assert_eq!(bool::from_wire(raw), expected, "raw {raw}");
        }
    }
}
