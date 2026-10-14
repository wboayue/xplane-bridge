#![doc = include_str!("../README.md")]

mod bridge;
mod codec;
pub mod commands;
pub mod datarefs;
mod error;
mod frame;
mod path;
mod protocol;
mod rate;
mod subscriptions;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
#[cfg(test)]
mod xplane_files;

pub use bridge::{DatarefSink, DatarefSource, XPlaneUdpBridge};
pub use codec::Datagram;
pub use commands::Command;
pub use datarefs::{Dataref, DatarefArray, ReadOnly, Value, Writable};
pub use error::{PathError, XPlaneError};
pub use frame::{Frame, Latest};
pub use protocol::{Protocol, Stats};
pub use rate::Rate;
pub use subscriptions::{Sub, Subscriptions};
