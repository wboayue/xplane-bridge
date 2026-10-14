# AGENTS.md

Guidance for AI assistants and humans working in `xplane-bridge`. `design.md` has the rationale and protocol details; `README.md` is the user-facing doc.

## What this is

A small sans-I/O, sync Rust library that talks X-Plane's UDP dataref protocol (`RREF`/`DREF`/`CMND`) so an external flight controller can read simulator state and drive the aircraft. Sibling of [realflight-bridge](https://github.com/wboayue/realflight-bridge); consumed by the `stable-flight-sitl-xplane` backend in [stable-flight](https://github.com/wboayue/stable-flight), but independent of both.

Status: pre-release. The crate skeleton may not exist yet; `design.md` is the spec until it does.

Design against the consumer, not speculatively. Before adding API, check stable-flight's open issues for the X-Plane backend (#351 backend, #352 mission demo, #89) and the ones that touch host timing or state (#79 sensor synthesis, #302/#309 stalls and reconnects): `gh issue list -R wboayue/stable-flight`. Its `StateSource`/`Actuator` traits and `RealTimeRunner` are sync, which is why this crate is.

## Commands

```bash
cargo build
cargo test                          # codec + loopback bridge tests, no simulator needed
cargo test -- --ignored             # tests that need a running X-Plane
cargo run --example probe -- --host 127.0.0.1:49000   # spike/manual check against X-Plane
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo doc --no-deps                 # keep rustdoc warning-free

# check a dataref path, type, writability, units (X-Plane 12 via Steam on macOS)
awk -F'\t' '$1 == "sim/flightmodel/position/P"' \
  ~/Library/Application\ Support/Steam/steamapps/common/X-Plane\ 12/Resources/plugins/DataRefs.txt
```

Reading `DataRefs.txt` needs no running simulator; only behavior (rates, reloads, write order) does. Use the X-Plane 12 file, not a neighboring X-Plane 11 install.

Rust edition 2024, MSRV 1.85.

## Layout

```
src/lib.rs               re-exports; crate docs are README.md (doctested)
src/error.rs             XPlaneError, PathError (thiserror)
src/path.rs              Path: the one place path rules live
src/codec/               pure packet encode/decode, Datagram, DecodeError + tests.rs
src/datarefs/            Value, Dataref, DatarefArray, constants grouped by X-Plane prefix
src/commands.rs          Command type and constants
src/subscriptions.rs     Sub handles, Subscriptions (pure)
src/frame.rs             Frame, FrameBuilder, Latest (pure)
src/protocol/            Protocol, Stats: sans-I/O core, calls → datagrams, bytes → frames + tests.rs
src/bridge/              DatarefSource/DatarefSink traits, udp.rs (driver), tests.rs
src/testing.rs           FakeXPlane (feature `testing`)
examples/probe.rs
```

## Rules

- **Sans-I/O.** All protocol logic lives in `Protocol`, which never touches a socket: calls return `Datagram`s to send, `handle` takes received bytes. The driver (`bridge/udp.rs`) is only the loop that moves datagrams to and from its socket. New behavior goes in `Protocol`, never in the driver; a second transport must be able to reuse it unchanged.
- **Sync, `std` only.** `std::net::UdpSocket` with non-blocking reads: no tokio, no async traits, no async feature. The consumer (stable-flight's `StateSource`/`Actuator` under `RealTimeRunner`) is sync; an async user drives the public `Protocol` from their own socket. See `design.md` "Non-goals" and "Why sans-I/O".
- **Report liveness, don't decide it.** The bridge exposes facts (`Frame::received_at`, `Stats`, `resubscribe`, pause and sim-time datarefs); timeouts, reconnect policy, and releasing overrides on shutdown belong to the consumer. See `design.md` "Liveness".
- **Raw simulator units.** Degrees, deg/s, meters in X-Plane's OpenGL local frame (x east, y up, z south). Do not convert to NED/FRD, radians, or `uom` types here; that is the consumer's job.
- **Never panic on simulator input.** The codec returns `DecodeError` for a bad or truncated packet; `poll` skips it (logged at `debug!`, counted in `Stats`) and keeps draining. `DecodeError` never escapes the bridge. No `.unwrap()`/`.expect()` in `src/`; use `?` or a typed error. `examples/` and tests may `expect`.
- **Never block the caller.** `poll` drains until `WouldBlock` and returns; `write` is one `send_to`. A control loop at 250 Hz must be able to call both every tick.
- **Only drivers touch sockets.** `Subscriptions`, `FrameBuilder`, and `Protocol` are pure and unit-tested without I/O; driver tests are a few loopback smoke tests.
- **Codec is pure.** Encode/decode take and return byte slices or arrays; no sockets, no state. Packet layouts are in `design.md`; byte widths (400 for `RREF` paths, 500 for `DREF`) are named constants.
- **Datarefs and commands are constants, not strings in call sites.** Add datarefs to `src/datarefs/` under the module matching their X-Plane path prefix, typed from `DataRefs.txt` (`Dataref<T, A>` or `DatarefArray<T, A, N>` with the right value type and `Writable` only if writable), with a doc comment giving units and sign; commands go in `src/commands.rs`. Verify the path against `Resources/plugins/DataRefs.txt` before adding it.
- **Dependencies stay minimal.** `thiserror`, `log`. Dev-dependencies for tests (e.g. `approx`) are fine. Anything else needs a reason in the PR.
- Logging: `log` crate; `info!` for lifecycle (bind, subscribe, drop), `debug!`/`trace!` inside the poll/write path. Nothing printed per tick at `info`.

## Style

Code:
- **One job per part.** Each type and module has one responsibility, describable without "and". The table in `design.md` "Architecture" is the map: new code fits a row or adds one.
- **Compose, don't extend.** New behavior is a new small part, wired in `Protocol` (the only place parts meet) or layered as a wrapper over `DatarefSource`/`DatarefSink`. No mode flags or optional fields bolted onto existing types.
- **Depend on traits, keep them small.** Consumers and tests take `impl DatarefSource`/`impl DatarefSink`, never `XPlaneUdpBridge`. Anything implementing them must also work wrapped (`impl<B: DatarefSource> DatarefSource for Wrapper<B>`).
- **One source of truth.** Each rule, constant, and layout lives once: path rules in `Path`, byte widths in `codec`, wire facts in `design.md` "Protocol". A second copy is a signal to extract; a third similar test becomes a table.
- **Types over checks.** Encode invariants in types (`Writable`, `Rate`, `Sub<T>`) and validate once at construction, not at each use.
- **Smallest visibility.** `pub(crate)` by default; `pub` only for what a consumer or a second transport needs.
- **No speculative API.** No parameter, generic, or feature without a consumer asking for it (see "Design against the consumer").

Docs:
- **Say it once, where it belongs.** `README.md`: usage. `design.md`: rationale and wire facts. Rustdoc: the item's contract. `AGENTS.md`: rules. Elsewhere, link instead of restating.
- **Concise.** Short sentences in present tense stating what is true; no filler, hedging, or history (that belongs in commits).
- **Rustdoc adds what the signature doesn't.** One summary line, then only units, sign, frame, errors, and edge cases; never a restatement of the types.
- **Comments say why, not what.** No comment that restates the code.

## Testing

- Table-driven byte-level tests for every packet type in `src/codec/tests.rs`, including boundary paths (exactly max, one over), empty and truncated replies.
- `Subscriptions`, `FrameBuilder`, and `Latest` get plain unit tests, no sockets.
- `Protocol` carries the bulk of behavior tests in `src/protocol/tests.rs`: bytes in, `Datagram`s out, asserted byte-for-byte. No sockets.
- Socket behavior is tested against a loopback `UdpSocket` acting as a fake X-Plane in `src/bridge/tests.rs`; the fake replies from a different port than it listens on, as X-Plane does. Keep these to socket concerns (drain, `ConnectionReset`, drop). No mocking framework; use `testing::FakeXPlane` (built on `Protocol` and `Frame::empty(t).with(sub, value)`) rather than writing another fake.
- Anything that needs the real simulator is `#[ignore]` with a one-line reason.
- Name tests after the behavior they protect (`merges_drained_packets`), not the function.
- Floats: `approx::assert_relative_eq!`.

## Docs and releases

- Public items have rustdoc with an example. `README.md` is the crate docs, so its usage compiles as a doctest; change it in the same PR as the API.
- `CHANGELOG.md` ([Keep a Changelog](https://keepachangelog.com/en/1.1.0/)): every user-facing change adds a bullet under `[Unreleased]`; mark breaking changes **Breaking**.
- Update `design.md` when a **verify** item is confirmed or an open question is answered; don't leave it describing a protocol detail the code contradicts.
- `Cargo.toml` metadata mirrors realflight-bridge (license MIT, keywords, categories, `include` list). Check `cargo package --list` before a release.

## Commits and PRs

- Concise imperative subjects; reference issues as `(#n)`. Issues in stable-flight are `wboayue/stable-flight#n`; a bare `#n` links to this repo.
- Pass PR bodies with `gh pr create --body-file`; inline heredocs break on apostrophes.
- No AI attribution lines or footers in commits or PRs.
- American English spelling in prose, docs, comments, and commit messages.
- Before a PR: `cargo fmt`, `cargo clippy --all-targets -- -D warnings`, `cargo test`, `cargo doc --no-deps` all clean.
