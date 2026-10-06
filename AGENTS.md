# AGENTS.md

Guidance for AI assistants and humans working in `xplane-bridge`. `design.md` has the rationale and protocol details; `README.md` is the user-facing doc.

## What this is

A small sans-I/O Rust library (sync driver by default, async behind `rt-tokio`) that talks X-Plane's UDP dataref protocol (`RREF`/`DREF`/`CMND`) so an external flight controller can read simulator state and drive the aircraft. Sibling of [realflight-bridge](https://github.com/wboayue/realflight-bridge); consumed by the `stable-flight-sitl-xplane` backend in [stable-flight](https://github.com/wboayue/stable-flight), but independent of both.

Status: pre-release. The crate skeleton may not exist yet; `design.md` is the spec until it does.

## Commands

```bash
cargo build
cargo test                          # codec + loopback bridge tests, no simulator needed
cargo test -- --ignored             # tests that need a running X-Plane
cargo run --example probe -- --host 127.0.0.1:49000   # spike/manual check against X-Plane
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo doc --no-deps                 # keep rustdoc warning-free
```

Rust edition 2024, MSRV 1.85.

## Layout

```
src/lib.rs        re-exports, crate docs
src/error.rs             XPlaneError (thiserror)
src/codec/               pure packet encode/decode, EncodeError/DecodeError + tests.rs
src/datarefs/            Dataref type and constants grouped by X-Plane prefix
src/commands.rs          Command type and constants
src/subscriptions.rs     Sub handles, Subscriptions (pure)
src/frame.rs             Frame, FrameBuilder, Latest (pure)
src/protocol/            Protocol, Datagram, Stats: sans-I/O core, calls → datagrams, bytes → frames + tests.rs
src/bridge/              DatarefSource/DatarefSink traits, udp.rs (sync driver), tests.rs,
                         tokio.rs (async traits + driver, feature = "rt-tokio")
examples/probe.rs
```

## Rules

- **Sans-I/O.** All protocol logic lives in `Protocol`, which never touches a socket: calls return `Datagram`s to send, `handle` takes received bytes. A driver (`bridge/udp.rs`, `bridge/tokio.rs`) is only the loop that moves datagrams to and from its socket. New behavior goes in `Protocol`, never in a driver; if the same code appears in both drivers, it belongs in `Protocol`.
- **Sync by default, `std` only.** The default build is `std::net::UdpSocket` with non-blocking reads: no tokio, no async traits. Async lives behind `rt-tokio` (`bridge/tokio.rs`) and is added only when a real async consumer exists; see `design.md` "Drivers".
- **Raw simulator units.** Degrees, deg/s, meters in X-Plane's OpenGL local frame (x east, y up, z south). Do not convert to NED/FRD, radians, or `uom` types here; that is the consumer's job.
- **Never panic on simulator input.** The codec returns `DecodeError` for a bad or truncated packet; `poll` skips it (logged at `debug!`, counted in `Stats`) and keeps draining. `DecodeError` never escapes the bridge. No `.unwrap()`/`.expect()` in `src/`; use `?` or a typed error. `examples/` and tests may `expect`.
- **Never block the caller.** `poll` drains until `WouldBlock` and returns; `write` is one `send_to`. A control loop at 250 Hz must be able to call both every tick.
- **Only drivers touch sockets.** `Subscriptions`, `FrameBuilder`, and `Protocol` are pure and unit-tested without I/O; driver tests are a few loopback smoke tests.
- **Codec is pure.** Encode/decode take and return byte slices or arrays; no sockets, no state. Packet layouts are in `design.md`; byte widths (400 for `RREF` paths, 500 for `DREF`) are named constants.
- **Datarefs and commands are constants, not strings in call sites.** Add datarefs to `src/datarefs/` under the module matching their X-Plane path prefix, typed from `DataRefs.txt` (`Dataref<T, A>` or `DatarefArray<T, A, N>` with the right value type and `Writable` only if writable), with a doc comment giving units and sign; commands go in `src/commands.rs`. Verify the path against `Resources/plugins/DataRefs.txt` before adding it.
- **Dependencies stay minimal.** `thiserror`, `log`. Dev-dependencies for tests (e.g. `approx`) are fine. Anything else needs a reason in the PR.
- Logging: `log` crate; `info!` for lifecycle (bind, subscribe, drop), `debug!`/`trace!` inside the poll/write path. Nothing printed per tick at `info`.

## Testing

- Table-driven byte-level tests for every packet type in `src/codec/tests.rs`, including boundary paths (exactly max, one over), empty and truncated replies.
- `Subscriptions`, `FrameBuilder`, and `Latest` get plain unit tests, no sockets.
- `Protocol` carries the bulk of behavior tests in `src/protocol/tests.rs`: bytes in, `Datagram`s out, asserted byte-for-byte. No sockets.
- Socket behavior is tested against a loopback `UdpSocket` acting as a fake X-Plane in `src/bridge/tests.rs`; the fake replies from a different port than it listens on, as X-Plane does. Keep these to socket concerns (drain, `ConnectionReset`, drop); the `rt-tokio` driver gets the same handful under `#[tokio::test]`. No mocking framework; hand-written fakes built from `Protocol` and `Frame::empty(t).with(sub, value)`.
- Anything that needs the real simulator is `#[ignore]` with a one-line reason.
- Name tests after the behavior they protect (`merges_drained_packets`), not the function.
- Floats: `approx::assert_relative_eq!`.

## Docs and releases

- Public items have rustdoc with an example. Keep `README.md` usage in sync with the real API in the same PR.
- `CHANGELOG.md` ([Keep a Changelog](https://keepachangelog.com/en/1.1.0/)): every user-facing change adds a bullet under `[Unreleased]`; mark breaking changes **Breaking**.
- Update `design.md` when a **verify** item is confirmed or an open question is answered; don't leave it describing a protocol detail the code contradicts.
- `Cargo.toml` metadata mirrors realflight-bridge (license MIT, keywords, categories, `include` list). Check `cargo package --list` before a release.

## Commits and PRs

- Concise imperative subjects; reference issues as `(#n)`.
- No AI attribution lines or footers in commits or PRs.
- American English spelling in prose, docs, comments, and commit messages.
- Before a PR: `cargo fmt`, `cargo clippy --all-targets -- -D warnings`, `cargo test`, `cargo doc --no-deps` all clean.
