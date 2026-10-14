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
- `probe` example and `#[ignore]`d tests against a running X-Plane (#15).
- `datarefs::time::FRAMERATE_PERIOD`; `probe` prints frame rate and the local origin (#16).
- `datarefs::wing`: control surface deflection constants (#35).
- `XPlaneUdpBridge::connect` takes any `ToSocketAddrs`, so hostnames such as `oya.local:49000` work, as do `probe --host` and `XPLANE_ADDR` (#34).
- `probe` resubscribes while any subscription has no value, since X-Plane can drop `RREF`s sent in a burst (#34).
- `Stats::dropped_sends` and `Protocol::dropped_send`: `XPlaneUdpBridge` drops and counts a send that finds the send buffer full instead of failing with `WouldBlock` (#34).

### Fixed

- `XPlaneUdpBridge` sets a 64 KiB send buffer, so subscribing to many datarefs over Wi-Fi on macOS no longer fails with `WouldBlock` (#34).
