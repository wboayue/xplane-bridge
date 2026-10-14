# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-14

Initial release.

### Added

- `Protocol` and `Stats`: a sans-I/O core for X-Plane's UDP dataref protocol (`RREF`, `DREF`, `CMND`); calls return `Datagram`s to send, received bytes go in through `handle`.
- `XPlaneUdpBridge`: a sync, non-blocking `std::net::UdpSocket` driver; `connect` takes any `ToSocketAddrs`.
- `DatarefSource` and `DatarefSink` traits; `testing::FakeXPlane` behind feature `testing`.
- Typed datarefs and handles: `Dataref`, `DatarefArray`, `Value`, `ReadOnly`, `Writable`, `Command`, `Rate`, and `Sub`; writing a read-only dataref or a wrong-typed value does not compile.
- `Frame` (one drain of state, newest value per dataref) and `Latest` (last known value per subscription).
- Dataref constants in `datarefs::{position, engine, joystick, overrides, time, wing}`; commands `PAUSE_TOGGLE` and `RESET_FLIGHT`.
- `XPlaneError` and `PathError`; malformed packets are counted in `Stats` and skipped, never surfaced.
- `probe` example for a manual check against a running X-Plane.

[Unreleased]: https://github.com/wboayue/xplane-bridge/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/wboayue/xplane-bridge/releases/tag/v0.1.0
