#![doc = include_str!("../README.md")]

#[cfg_attr(not(test), expect(dead_code, reason = "used by Protocol (#12)"))]
mod codec;
pub mod commands;
pub mod datarefs;
mod error;
mod frame;
mod path;
mod rate;
mod subscriptions;
#[cfg(test)]
mod xplane_files;

pub use codec::Datagram;
pub use commands::Command;
pub use datarefs::{Dataref, DatarefArray, ReadOnly, Value, Writable};
pub use error::{PathError, XPlaneError};
pub use frame::{Frame, Latest};
pub use rate::Rate;
pub use subscriptions::{Sub, Subscriptions};
