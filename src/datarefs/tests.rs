use std::collections::HashSet;

use super::engine::THRO_USE;
use super::*;
use crate::PathError;
use crate::xplane_files;

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

/// X-Plane types a value type reads correctly: `double` arrives as `f32`, and
/// `bool` reads an `int` boolean.
trait Listed {
    const TYPES: &'static [&'static str];
}

impl Listed for f32 {
    const TYPES: &'static [&'static str] = &["float", "double"];
}

impl Listed for i32 {
    const TYPES: &'static [&'static str] = &["int"];
}

impl Listed for bool {
    const TYPES: &'static [&'static str] = &["int"];
}

trait Access {
    const WRITABLE: bool;
}

impl Access for ReadOnly {
    const WRITABLE: bool = false;
}

impl Access for Writable {
    const WRITABLE: bool = true;
}

/// A constant as `DataRefs.txt` should list it, read from its type.
struct Entry {
    path: String,
    types: Vec<String>,
    writable: bool,
    wire_paths: Vec<String>,
}

fn scalar<T: Value + Listed, A: Access>(dataref: &Dataref<T, A>) -> Entry {
    Entry {
        path: dataref.path().into_owned(),
        types: T::TYPES.iter().map(|t| t.to_string()).collect(),
        writable: A::WRITABLE,
        wire_paths: vec![dataref.path().into_owned()],
    }
}

fn array<T: Value + Listed, A: Access, const N: usize>(array: &DatarefArray<T, A, N>) -> Entry {
    Entry {
        path: array.base.as_str().to_owned(),
        types: T::TYPES.iter().map(|t| format!("{t}[{N}]")).collect(),
        writable: A::WRITABLE,
        wire_paths: array.iter().map(|d| d.path().into_owned()).collect(),
    }
}

/// Every dataref constant. Add new constants here.
fn constants() -> Vec<Entry> {
    use super::{joystick, overrides, position, time};
    vec![
        scalar(&position::PHI),
        scalar(&position::THETA),
        scalar(&position::PSI),
        scalar(&position::P),
        scalar(&position::Q),
        scalar(&position::R),
        scalar(&position::LOCAL_X),
        scalar(&position::LOCAL_Y),
        scalar(&position::LOCAL_Z),
        scalar(&position::LOCAL_VX),
        scalar(&position::LOCAL_VY),
        scalar(&position::LOCAL_VZ),
        scalar(&position::LOCAL_AX),
        scalar(&position::LOCAL_AY),
        scalar(&position::LOCAL_AZ),
        scalar(&position::LATITUDE),
        scalar(&position::LONGITUDE),
        scalar(&position::ELEVATION),
        scalar(&position::LAT_REF),
        scalar(&position::LON_REF),
        array(&position::QUATERNION),
        array(&THRO_USE),
        scalar(&joystick::YOKE_ROLL_RATIO),
        scalar(&joystick::YOKE_PITCH_RATIO),
        scalar(&joystick::YOKE_HEADING_RATIO),
        scalar(&overrides::THROTTLES),
        scalar(&overrides::JOYSTICK),
        scalar(&overrides::CONTROL_SURFACES),
        scalar(&time::PAUSED),
        scalar(&time::TOTAL_RUNNING_TIME_SEC),
    ]
}

#[test]
fn every_constant_passes_path_rules() {
    for entry in constants() {
        for path in entry.wire_paths {
            assert!(Path::new(path.as_str()).is_ok(), "path {path}");
        }
    }
}

#[test]
fn constants_are_distinct() {
    let paths: HashSet<_> = constants().into_iter().map(|e| e.path).collect();
    assert_eq!(paths.len(), constants().len());
}

#[test]
#[ignore = "needs an X-Plane 12 install at $XPLANE_DIR"]
fn constants_match_datarefs_txt() {
    let listed = xplane_files::datarefs();
    for entry in constants() {
        let Some(found) = listed.get(&entry.path) else {
            panic!("{} is not in DataRefs.txt", entry.path);
        };
        assert!(
            entry.types.contains(&found.ty),
            "{}: listed as {}, constant reads {:?}",
            entry.path,
            found.ty,
            entry.types
        );
        assert_eq!(entry.writable, found.writable, "{}: writable", entry.path);
    }
}
