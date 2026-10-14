use std::collections::HashSet;

use super::engine::THRO_USE;
use super::*;
use crate::PathError;

#[test]
fn formats_wire_path() {
    let scalar = Dataref::<f32, Writable>::new("sim/flightmodel/position/phi").unwrap();
    let cases = [
        (scalar, "sim/flightmodel/position/phi"),
        (
            THRO_USE.at(0).unwrap(),
            "sim/flightmodel/engine/ENGN_thro_use[0]",
        ),
        (
            THRO_USE.at(3).unwrap(),
            "sim/flightmodel/engine/ENGN_thro_use[3]",
        ),
        (
            THRO_USE.at(15).unwrap(),
            "sim/flightmodel/engine/ENGN_thro_use[15]",
        ),
    ];
    for (dataref, expected) in cases {
        assert_eq!(dataref.path(), expected);
    }
}

#[test]
fn at_is_none_past_the_end() {
    assert!(THRO_USE.at(15).is_some());
    assert!(THRO_USE.at(16).is_none());
    assert!(THRO_USE.at(usize::MAX).is_none());
}

#[test]
fn iter_yields_every_element_in_order() {
    let paths: Vec<_> = THRO_USE.iter().map(|d| d.path().into_owned()).collect();
    let expected: Vec<_> = (0..16)
        .map(|i| format!("sim/flightmodel/engine/ENGN_thro_use[{i}]"))
        .collect();
    assert_eq!(paths, expected);
}

#[test]
fn elements_are_distinct_handles() {
    let elements: HashSet<_> = THRO_USE.iter().collect();
    assert_eq!(elements.len(), 16);
    assert_eq!(THRO_USE.at(2), THRO_USE.at(2));
    assert_ne!(THRO_USE.at(2), THRO_USE.at(3));
}

#[test]
fn static_constructor_matches_runtime_constructor() {
    const PHI: Dataref<f32, Writable> = Dataref::from_static("sim/flightmodel/position/phi");
    let runtime = Dataref::<f32, Writable>::new("sim/flightmodel/position/phi").unwrap();
    assert_eq!(PHI, runtime);
}

#[test]
fn new_rejects_invalid_path() {
    let err = Dataref::<f32>::new("").unwrap_err();
    assert!(matches!(err, XPlaneError::InvalidPath(PathError::Empty)));
}

#[test]
fn iterator_outlives_the_constant() {
    // A const is a temporary at each use; the iterator must not borrow it.
    let elements = THRO_USE.iter();
    assert_eq!(elements.count(), 16);
}
