#![doc = include_str!("../README.md")]

pub mod commands;
pub mod datarefs;
mod error;
mod path;
mod rate;

pub use commands::Command;
pub use datarefs::{Dataref, DatarefArray, ReadOnly, Value, Writable};
pub use error::{PathError, XPlaneError};
pub use rate::Rate;
