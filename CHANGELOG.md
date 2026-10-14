# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `XPlaneError` and `PathError` (#6).
- `Value`, `Dataref`, `DatarefArray`, `ReadOnly`, `Writable`, `Command`, and `Rate` (#7).
- `Datagram`: an encoded outgoing packet (#8).
- Dataref constants in `datarefs::{position, engine, joystick, overrides, time}`; `commands::PAUSE_TOGGLE` and `RESET_FLIGHT` (#9).
- `Sub` handles and `Subscriptions` (#10).
- `Frame` and `Latest` (#11).
- `Protocol` and `Stats`: the sans-I/O protocol core (#12).
- `DatarefSource` and `DatarefSink` traits; `testing::FakeXPlane` behind feature `testing` (#13).
- `XPlaneUdpBridge`: the `std::net::UdpSocket` driver (#14).
