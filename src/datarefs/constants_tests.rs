//! Checks every dataref constant: path rules, and against `DataRefs.txt`.

use std::collections::HashSet;

use super::*;
use crate::xplane_files;

/// X-Plane types a value type reads correctly: `double` arrives as `f32`, and
/// `bool` reads an `int` boolean.
trait XPlaneType {
    const TYPES: &'static [&'static str];
}

impl XPlaneType for f32 {
    const TYPES: &'static [&'static str] = &["float", "double"];
}

impl XPlaneType for i32 {
    const TYPES: &'static [&'static str] = &["int"];
}

impl XPlaneType for bool {
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

fn scalar<T: Value + XPlaneType, A: Access>(dataref: &Dataref<T, A>) -> Entry {
    Entry {
        path: dataref.path().into_owned(),
        types: T::TYPES.iter().map(|t| t.to_string()).collect(),
        writable: A::WRITABLE,
        wire_paths: vec![dataref.path().into_owned()],
    }
}

fn array<T: Value + XPlaneType, A: Access, const N: usize>(array: &DatarefArray<T, A, N>) -> Entry {
    Entry {
        path: array.base.as_str().to_owned(),
        types: T::TYPES.iter().map(|t| format!("{t}[{N}]")).collect(),
        writable: A::WRITABLE,
        wire_paths: array.iter().map(|d| d.path().into_owned()).collect(),
    }
}

/// Every dataref constant. Add new constants here.
fn constants() -> Vec<Entry> {
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
        array(&engine::THRO_USE),
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

/// Source of every module with constants, to catch one missing from [`constants`].
const SOURCES: [&str; 5] = [
    include_str!("engine.rs"),
    include_str!("joystick.rs"),
    include_str!("overrides.rs"),
    include_str!("position.rs"),
    include_str!("time.rs"),
];

#[test]
fn table_lists_every_constant() {
    let defined = SOURCES
        .iter()
        .flat_map(|src| src.lines())
        .filter(|line| line.starts_with("pub const "))
        .count();
    assert_eq!(constants().len(), defined);
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
