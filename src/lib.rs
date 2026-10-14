#![doc = include_str!("../README.md")]

#[cfg_attr(not(test), expect(dead_code, reason = "used by Protocol (#12)"))]
mod codec;
pub mod commands;
pub mod datarefs;
mod error;
mod path;
mod rate;

pub use codec::Datagram;
pub use commands::Command;
pub use datarefs::{Dataref, DatarefArray, ReadOnly, Value, Writable};
pub use error::{PathError, XPlaneError};
pub use rate::Rate;
