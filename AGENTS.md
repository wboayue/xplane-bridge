# AGENTS.md

Guidance for AI assistants and humans working in `xplane-bridge`. `design.md` has the rationale and protocol details; `README.md` is the user-facing doc.

## What this is

A small synchronous Rust library that talks X-Plane's UDP dataref protocol (`RREF`/`DREF`/`CMND`) so an external flight controller can read simulator state and drive the aircraft. Sibling of [realflight-bridge](https://github.com/wboayue/realflight-bridge); consumed by the `stable-flight-sitl-xplane` backend in [stable-flight](https://github.com/wboayue/stable-flight), but independent of both.

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
src/error.rs      XPlaneError (thiserror)
src/codec/        pure packet encode/decode + tests.rs
src/datarefs/     dataref path constants grouped by X-Plane prefix
src/bridge/       XPlaneBridge trait, Frame, udp.rs implementation, tests.rs
examples/probe.rs
```

## Rules

- **Sync and `std` only.** `std::net::UdpSocket`, non-blocking reads. No tokio, no async traits. If async is ever wanted, it goes behind a feature like realflight-bridge's `rt-tokio`, never in the default build.
- **Raw simulator units.** Degrees, deg/s, meters in X-Plane's OpenGL local frame (x east, y up, z south). Do not convert to NED/FRD, radians, or `uom` types here; that is the consumer's job.
- **Never panic on simulator input.** The codec returns `XPlaneError::Malformed` for a bad or truncated packet; `poll` skips it (logged at `debug!`, counted) and keeps draining. No `.unwrap()`/`.expect()` in `src/`; use `?` or a typed error. `examples/` and tests may `expect`.
- **Never block the caller.** `poll` drains until `WouldBlock` and returns; `write` is one `send_to`. A control loop at 250 Hz must be able to call both every tick.
- **Codec is pure.** Encode/decode take and return byte slices or arrays; no sockets, no state. Packet layouts are in `design.md`; byte widths (400 for `RREF` paths, 500 for `DREF`) are named constants.
- **Datarefs are constants, not strings in call sites.** Add new ones to `src/datarefs/` under the module matching their X-Plane path prefix, with a doc comment giving units and sign. Verify the path against `Resources/plugins/DataRefs.txt` before adding it.
- **Dependencies stay minimal.** `thiserror`, `log`. Dev-dependencies for tests (e.g. `approx`) are fine. Anything else needs a reason in the PR.
- Logging: `log` crate; `info!` for lifecycle (bind, subscribe, drop), `debug!`/`trace!` inside the poll/write path. Nothing printed per tick at `info`.

## Testing

- Table-driven byte-level tests for every packet type in `src/codec/tests.rs`, including boundary paths (exactly max, one over), empty and truncated replies.
- Bridge behavior is tested against a loopback `UdpSocket` acting as a fake X-Plane in `src/bridge/tests.rs`. No mocking framework; hand-written fakes.
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
