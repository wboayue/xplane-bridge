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

#[test]
fn clone_equals_original() {
    let element = THRO_USE.at(3).unwrap();
    assert_eq!(element.clone(), element);
    assert_eq!(super::position::PHI.clone(), super::position::PHI);
    assert_eq!(THRO_USE.clone(), THRO_USE);
}

#[test]
fn debug_shows_path_and_index() {
    let cases = [
        (
            format!("{:?}", super::position::PHI),
            r#"Dataref { path: "sim/flightmodel/position/phi", index: None }"#,
        ),
        (
            format!("{:?}", THRO_USE.at(3).unwrap()),
            r#"Dataref { path: "sim/flightmodel/engine/ENGN_thro_use", index: Some(3) }"#,
        ),
        (
            format!("{THRO_USE:?}"),
            r#"DatarefArray { base: "sim/flightmodel/engine/ENGN_thro_use", len: 16 }"#,
        ),
    ];
    for (debug, expected) in cases {
        assert_eq!(debug, expected);
    }
}

#[test]
fn arrays_compare_and_hash_by_base() {
    const BY_HAND: DatarefArray<f32, Writable, 16> =
        DatarefArray::from_static("sim/flightmodel/engine/ENGN_thro_use");
    const OTHER: DatarefArray<f32, Writable, 16> =
        DatarefArray::from_static("sim/flightmodel/engine/ENGN_prop");
    assert_eq!(BY_HAND, THRO_USE);
    assert_ne!(OTHER, THRO_USE);
    let arrays: HashSet<_> = [BY_HAND, THRO_USE, OTHER].into_iter().collect();
    assert_eq!(arrays.len(), 2);
}
