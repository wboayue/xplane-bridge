//! Reads X-Plane's own lists of datarefs and commands, for the `#[ignore]`d
//! tests that check constants against them.

use std::collections::HashMap;
use std::path::PathBuf;

/// `Resources/plugins/<name>` under `$XPLANE_DIR`.
fn read(name: &str) -> String {
    let dir =
        std::env::var_os("XPLANE_DIR").expect("set XPLANE_DIR to the X-Plane 12 install directory");
    let path = PathBuf::from(dir).join("Resources/plugins").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

/// One `DataRefs.txt` line: type (`float`, `int[8]`, ...) and whether writable.
#[derive(Debug, PartialEq, Eq)]
pub(crate) struct Listed {
    pub(crate) ty: String,
    pub(crate) writable: bool,
}

/// `DataRefs.txt` by path. Lines are tab-separated: path, type, `y`/`n`, units, description.
pub(crate) fn datarefs() -> HashMap<String, Listed> {
    read("DataRefs.txt")
        .lines()
        .filter_map(|line| {
            let mut fields = line.split('\t');
            let (path, ty, writable) = (fields.next()?, fields.next()?, fields.next()?);
            let listed = Listed {
                ty: ty.to_owned(),
                writable: writable == "y",
            };
            Some((path.to_owned(), listed))
        })
        .collect()
}

/// Command paths in `Commands.txt`: the first word of each line.
pub(crate) fn commands() -> Vec<String> {
    read("Commands.txt")
        .lines()
        .filter_map(|line| line.split_whitespace().next())
        .map(str::to_owned)
        .collect()
}
