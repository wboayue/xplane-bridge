# xplane-bridge design

Initial design, written before the first line of code. This document states what is true; how it was established lives in [Spike findings](#spike-findings). Items marked **verify** are still assumptions; when one is confirmed, update the fact where it is stated and add the evidence to the findings.

## Goals

- Give an external flight controller a thin link to X-Plane over its built-in UDP protocol: read datarefs every simulator frame, write datarefs every control tick.
- Run on macOS, Windows, and Linux with only `std` networking in the default build. No plugin inside X-Plane; no async runtime unless the `rt-tokio` feature is enabled.
- Implement the protocol once, sans-I/O, so the sync driver and the optional async driver are thin shells over the same logic ([Drivers](#drivers)).
- Be testable without a simulator: the protocol core is pure, and consumers depend on small traits they can fake.
- Stay simulator-honest: report values in X-Plane's own units and frames. Conversion to a controller's conventions is the caller's job (for Stable Flight, the `stable-flight-sitl-xplane` backend).

## Non-goals

- Lockstep or deterministic stepping. X-Plane does not expose it over UDP; the simulator runs free and the bridge samples it.
- Sensor emulation (IMU noise, GPS, baro). The bridge returns ground truth; sensor models belong upstream.
- Plugin-side code, the X-Plane 12.1 REST/WebSocket API, multiplayer, or the legacy `DATA` output rows. They can be added behind the same traits later if a need appears.
- Frame/unit conversion, `uom` types. Possibly a feature later, as in realflight-bridge; not in v1.
- An async driver in v1. The design supports one ([Drivers](#drivers)), but it is only added once a real async consumer exists (a telemetry or GCS tool, not the flight backend).

## Protocol

This section is the single source for wire-level facts; later sections refer to it.

**Transport.** X-Plane listens on UDP `49000` (configurable in **Settings → Network**). It replies to the source address and port of the socket that sent the request, so one socket serves both directions. Replies come *from* port `49001`, not `49000`. Cross-host behavior: **verify**.

**Encoding.** Integers and floats are little-endian. Strings are null-padded to a fixed width.

| Packet | Direction | Layout | Notes |
|---|---|---|---|
| `RREF` | → X-Plane | `"RREF\0"`, `i32 freq`, `i32 index`, `char[400] path` (413 bytes) | Subscribe `path` at `freq` per second under caller-chosen `index`. `freq = 0` unsubscribes. The same path under different indices is sent once per index. |
| `RREF,` | ← X-Plane | `"RREF,"`, then repeated `{ i32 index, f32 value }` | Holds at most 183 records (1469 bytes). Up to 183 subscriptions arrive in one packet per frame; larger sets spill into extra packets, and not every record may arrive every frame. Rate is capped at the simulator frame rate whatever `freq` asks for. |
| `DREF` | → X-Plane | `"DREF\0"`, `f32 value`, `char[500] path` (509 bytes) | Write one value. Array elements address as `path[i]`. Fire-and-forget; no ack. **verify** float conversion for int datarefs (e.g. `override_throttles`) and that writes apply in arrival order within a frame. |
| `CMND` | → X-Plane | `"CMND\0"`, null-terminated `path` | Trigger a command, e.g. `sim/operation/pause_toggle`. |
| `BECN` | ← multicast `239.255.1.1:49707` | beacon | Discovery. Out of scope for v1; the caller passes an address. |

Datarefs are addressed by string path (`sim/flightmodel/position/phi`). The authoritative list ships with X-Plane as `Resources/plugins/DataRefs.txt`. Writable datarefs the controller needs only take effect after the matching `sim/operation/override/*` flag is set to `1`.

## Architecture

The bridge is sans-I/O: every protocol decision lives in `Protocol`, which never touches a socket, and a *driver* is the smallest loop that moves `Protocol`'s datagrams to and from one. Each part has one job, depends only on the parts above it, and is tested on its own. The file tree is in `AGENTS.md`.

| Part | Module | Job | I/O |
|---|---|---|---|
| `codec` | `codec/` | Encode `RREF`/`DREF`/`CMND`; decode `RREF,` into `(index, value)` records | none |
| `Dataref`, `Command` | `datarefs/`, `commands.rs` | Name a path; constants | none |
| `Subscriptions` | `subscriptions.rs` | Allocate indices; track active `Sub` ↔ `Dataref` | none |
| `Frame`, `FrameBuilder`, `Latest` | `frame.rs` | Merge records from one drain; optionally hold values across frames | none |
| `Protocol` | `protocol/` | Compose the above: calls → `Datagram`s, received bytes → `Frame`s, `Stats` | none |
| traits, `XPlaneUdpBridge` | `bridge/mod.rs`, `bridge/udp.rs` | Sync driver: `std::net::UdpSocket` around a `Protocol` | socket |
| async traits, `tokio::XPlaneUdpBridge` | `bridge/tokio.rs` | Async driver (`rt-tokio`): `tokio::net::UdpSocket` around the same `Protocol` | socket |

`codec` knows bytes, `Subscriptions` knows indices, `FrameBuilder` knows merging; none knows the others. `Protocol` is the only part that wires them together. Consumers depend on the traits `DatarefSource`/`DatarefSink` (sync) or `AsyncDatarefSource`/`AsyncDatarefSink` (`rt-tokio`), not on a driver type.

### Names: `Dataref`, `Command`

```rust
/// A dataref path plus optional array index. Constants are `const`
/// (`Cow::Borrowed`); `Dataref::new(impl Into<String>)` builds one at runtime.
/// `at(i)` returns a copy addressing element `i`.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Dataref { path: Cow<'static, str>, index: Option<u16> }

/// A command path. Same shape and rules as `Dataref`, no index.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Command { path: Cow<'static, str> }
```

Call sites use constants or values built once at startup, never string literals per tick.

### Handles: `Sub`, `Subscriptions`

```rust
/// Handle for one active subscription. Cheap to copy; reads go through it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Sub(i32);

/// Index bookkeeping only: no encoding, no rates, no datagrams.
impl Subscriptions {
    /// The existing handle for `dataref`, or a newly allocated one.
    pub(crate) fn insert(&mut self, dataref: &Dataref) -> Result<Sub, XPlaneError>;
    /// Remove `sub`, returning its dataref, or `None` if not active.
    pub(crate) fn remove(&mut self, sub: Sub) -> Option<Dataref>;
    /// Remove every active subscription (shutdown).
    pub(crate) fn drain(&mut self) -> impl Iterator<Item = (Sub, Dataref)> + '_;

    pub fn is_active(&self, sub: Sub) -> bool;
    pub fn get(&self, dataref: &Dataref) -> Option<Sub>;
}
```

Handles replace path lookup on the read path: `frame.get(sub)` is a map lookup on an `i32`, with no string hashing per tick and no ambiguity between `THRO_USE.at(0)` and `Dataref::new("…/ENGN_thro_use[0]")`. Indices are allocated monotonically and not reused after unsubscribe, so a late packet for an old index is never misread as a new subscription. Mutation is crate-private: handles are only ever allocated by `Protocol`, in the same call that encodes the `RREF`, so a handle and its datagram cannot drift apart. Fakes get handles from a `Protocol`, as the real bridge does; the read-only half is public through `Protocol::subscriptions()`.

### Core: `Protocol`, `Datagram`

```rust
/// One outgoing UDP payload. Sized for the largest packet (`DREF`, 509 bytes); no heap.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Datagram { buf: [u8; 509], len: usize }
impl Datagram { pub fn as_bytes(&self) -> &[u8]; }

/// Sans-I/O protocol state. Owns `Subscriptions`, a `FrameBuilder`, and `Stats`.
/// Every method is synchronous. `write`, `command`, and `handle` do not
/// allocate; `finish` allocates only the `Frame` it returns.
pub struct Protocol { .. }

impl Protocol {
    pub fn new() -> Self;

    // Calls → datagrams. The driver sends what comes back, in order.
    /// Re-subscribing a dataref returns its existing handle at the new rate.
    pub fn subscribe(&mut self, dataref: &Dataref, rate_hz: i32) -> Result<(Sub, Datagram), XPlaneError>;
    /// The `freq = 0` datagram, or `None` if `sub` is not active.
    pub fn unsubscribe(&mut self, sub: Sub) -> Option<Datagram>;
    pub fn write(&self, dataref: &Dataref, value: f32) -> Result<Datagram, XPlaneError>;
    pub fn command(&self, command: &Command) -> Result<Datagram, XPlaneError>;
    /// `freq = 0` for every active subscription; for shutdown.
    pub fn close(&mut self) -> Vec<Datagram>;

    // Received bytes → frames. The driver calls `handle` once per datagram
    // it drained and `finish` once per drain.
    /// Decode one datagram and fold it into the pending frame. Non-`RREF,`
    /// datagrams are ignored; malformed ones are counted and skipped. Never errs.
    pub fn handle(&mut self, bytes: &[u8], at: Instant);
    /// The merged frame since the last `finish`, or `None`.
    pub fn finish(&mut self) -> Option<Frame>;

    pub fn stats(&self) -> Stats;
    pub fn subscriptions(&self) -> &Subscriptions;
}

/// Counters since `Protocol::new`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats { pub packets: u64, pub malformed: u64, pub dropped_records: u64 }
```

Each call maps to exactly one outgoing datagram (`close` to one per subscription), so methods *return* the datagram instead of queueing it; there is no outbox and no callback. `Protocol` is public: it is the whole protocol for anyone bringing their own socket (`socket2`, `mio`, an embedded stack), and it is what the tests exercise.

`Protocol` is the only place the pure parts meet. `subscribe` checks the rate (negative is `InvalidRate`), asks `Subscriptions` for a handle, and encodes the `RREF`; `unsubscribe` and `close` encode `freq = 0` for what `Subscriptions` removed. `handle` decodes with `codec`, keeps records whose index `Subscriptions` reports active, counts the rest in `Stats::dropped_records`, and pushes the kept ones into `FrameBuilder`.

### Values: `Frame`, `FrameBuilder`, `Latest`

```rust
/// Values that arrived in one drain, newest per `Sub`. `received_at` is the
/// arrival time of the last datagram merged in.
pub struct Frame { values: HashMap<Sub, f32>, received_at: Instant }

impl Frame {
    pub fn get(&self, sub: Sub) -> Option<f32>;
    pub fn received_at(&self) -> Instant;
    /// For fakes and tests.
    pub fn from_values(received_at: Instant, values: impl IntoIterator<Item = (Sub, f32)>) -> Self;
}

impl FrameBuilder {
    /// Fold one packet's records in; later records overwrite earlier ones.
    /// Knows nothing of `Subscriptions`; `Protocol` filters before pushing.
    pub fn push(&mut self, records: impl Iterator<Item = (Sub, f32)>, at: Instant);
    /// The merged frame, or `None` if nothing was pushed. Resets the builder.
    pub fn finish(&mut self) -> Option<Frame>;
}

impl Latest {
    pub fn apply(&mut self, frame: &Frame);
    /// Last known value and when it arrived.
    pub fn get(&self, sub: Sub) -> Option<(f32, Instant)>;
}
```

A `Frame` reports only what arrived; a `Sub` absent from it (rate below poll rate, or a split reply that skipped it) reads `None`. Callers who want held values compose `Latest` on top instead of each writing their own cache. Merging every packet in a drain, rather than keeping the newest, is what keeps slower subscriptions and split replies intact.

### Traits

```rust
pub trait DatarefSource {
    /// Subscribe each dataref at `rate_hz`; handles are returned in input order.
    fn subscribe(&mut self, datarefs: &[Dataref], rate_hz: i32) -> Result<Vec<Sub>, XPlaneError>;
    /// Stop streaming. Unknown or already-removed handles are ignored.
    fn unsubscribe(&mut self, subs: &[Sub]) -> Result<(), XPlaneError>;
    /// Drain everything that arrived; `None` if nothing did. Never blocks.
    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError>;
    fn stats(&self) -> Stats;
}

pub trait DatarefSink {
    /// One `DREF`. Returns once the datagram is handed to the OS.
    fn write(&mut self, dataref: &Dataref, value: f32) -> Result<(), XPlaneError>;
    /// One `CMND`.
    fn command(&mut self, command: &Command) -> Result<(), XPlaneError>;
}
```

An actuator-only component takes `impl DatarefSink`; a state estimator takes `impl DatarefSource`. `XPlaneUdpBridge` implements both. A fake source is a `Protocol` plus a queue of `Frame`s.

The async twins (`rt-tokio`) have the same methods as `async fn`, plus one the sync side cannot offer:

```rust
pub trait AsyncDatarefSource {
    async fn subscribe(&mut self, datarefs: &[Dataref], rate_hz: i32) -> Result<Vec<Sub>, XPlaneError>;
    async fn unsubscribe(&mut self, subs: &[Sub]) -> Result<(), XPlaneError>;
    /// Drain what has arrived without waiting; same contract as the sync `poll`.
    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError>;
    /// Wait for the next frame. Cancel-safe: dropping the future loses nothing.
    async fn next_frame(&mut self) -> Result<Frame, XPlaneError>;
    fn stats(&self) -> Stats;
}

pub trait AsyncDatarefSink {
    async fn write(&mut self, dataref: &Dataref, value: f32) -> Result<(), XPlaneError>;
    async fn command(&mut self, command: &Command) -> Result<(), XPlaneError>;
}
```

`async fn` in traits (edition 2024) is used as is; the traits are for static dispatch, and nothing in the crate needs `dyn`.

### Drivers

A driver is the smallest loop that moves `Datagram`s between a `Protocol` and a socket. The sync and async drivers differ only in the socket type and the `.await`s; neither contains protocol logic.

```rust
// bridge/udp.rs (default build)
pub struct XPlaneUdpBridge { proto: Protocol, sock: std::net::UdpSocket, addr: SocketAddr }

impl DatarefSink for XPlaneUdpBridge {
    fn write(&mut self, d: &Dataref, v: f32) -> Result<(), XPlaneError> {
        let dg = self.proto.write(d, v)?;
        self.sock.send_to(dg.as_bytes(), self.addr)?;
        Ok(())
    }
}

impl DatarefSource for XPlaneUdpBridge {
    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError> {
        let mut buf = [0u8; 1500];
        loop {
            match self.sock.recv_from(&mut buf) {
                Ok((n, _)) => self.proto.handle(&buf[..n], Instant::now()),
                Err(e) if is_no_data(&e) => return Ok(self.proto.finish()),
                Err(e) => return Err(e.into()),
            }
        }
    }
}

// bridge/tokio.rs (feature = "rt-tokio"): identical shape over tokio::net::UdpSocket.
// `poll` uses `try_recv_from`; `next_frame` loops `recv_from(..).await` + `handle`
// until `finish()` is `Some`.
```

Socket rules, shared by both drivers:

- `connect(addr: SocketAddr) -> Result<Self>` binds an ephemeral local port, sets the socket non-blocking, and stores `addr`. No handshake exists, so "connected" only means the socket is ready. The socket is never `connect()`ed, and replies are accepted from any port on `addr`'s IP, since they come from `49001` ([Protocol](#protocol)).
- Every other call is one `Protocol` call plus a `send_to` per returned datagram. Eight motor writes per tick are eight datagrams, which is fine on loopback.
- `poll` errs only on socket errors; malformed datagrams are `Protocol`'s concern (counted, logged at `debug!`, skipped).
- `is_no_data` (in `bridge/mod.rs`, used by both drivers) treats `WouldBlock` and `ConnectionReset` as "drained". On Windows `ConnectionReset` follows a `send_to` that hit a closed port (X-Plane not running yet). **verify** on Windows; the alternative, disabling `SIO_UDP_CONNRESET`, needs a non-`std` call.
- `Drop` sends `Protocol::close()` so X-Plane stops streaming to a dead port. Send errors are logged and ignored; `Drop` never panics. The async driver cannot `.await` in `Drop`, so it uses `try_send_to`, best effort.

#### Why sans-I/O

This is the split `rustls` (`read_tls`/`write_tls`) and `quinn-proto`/`quinn` use. Compared with realflight-bridge's `async_impl.rs` files, which duplicate each bridge's logic for the async runtime:

- Protocol logic exists once and has one test suite; the drivers are too small to diverge.
- `Protocol` tests need no sockets; driver tests are a few loopback smoke tests each.
- A third transport (the 12.1 WebSocket API, a pcap replayer, an in-process fake X-Plane for CI) is another driver, not another fork.

Rejected alternatives: a `maybe-async`-style macro emitting both from one source (hides control flow, still mixes I/O with logic); an async-only core with `block_on` in a sync shim (tokio in the default build, a runtime inside a 250 Hz loop); sync-only with `spawn_blocking` for async users (works as a stopgap, costs a thread and a channel per bridge).

### Errors

Codec errors are separate from bridge errors, so the public bridge error lists only what a bridge call can return.

```rust
// codec
pub enum EncodeError { PathTooLong { len: usize, max: usize } }   // > 400 (RREF) or 500 (DREF) bytes
pub struct DecodeError { reason: &'static str }                   // bad header or truncated record

// bridge
pub enum XPlaneError {
    Io(std::io::Error),            // socket bind/send/recv
    PathTooLong { len, max },      // From<EncodeError>
    InvalidRate(i32),              // negative subscription rate
    TooManySubscriptions,          // index space exhausted (i32, so practically never)
}
```

`DecodeError` never leaves the bridge: simulator input never panics the library and never fails `poll`.

## Timing

- State rate is capped at X-Plane's frame rate ([Protocol](#protocol)), typically 60–120 Hz. A 250 Hz controller gets `None` from `poll` on most ticks; `Latest` or the caller holds values in between. The bridge does not interpolate.
- State age is one frame plus loopback latency; `Frame::received_at` lets callers measure it.
- The flight model runs several physics steps per frame (**Settings → General → Flight models per frame**); writes land at frame boundaries, so actuator latency is also up to one frame.
- The simulator pauses when it loses focus unless that setting is disabled, and `poll` then returns `None` indefinitely. The bridge does not detect this; callers that care should watch `received_at`.

## Testing

- `codec/tests.rs`: table-driven byte-level cases for every packet type, including boundary paths (400/500 bytes, one over), empty replies, truncated records, a 183-record reply.
- `subscriptions.rs`, `frame.rs`: unit tests for index allocation, handle reuse on re-subscribe, no index reuse after unsubscribe, merging across packets, `Latest` hold behavior. No sockets.
- `protocol/tests.rs`: the bulk of the behavior tests. Feed `Protocol` byte vectors and assert the returned `Datagram`s byte-for-byte: subscribe/unsubscribe/write/command round trips, malformed input counted and skipped, records for inactive indices dropped and counted, split replies merged by `finish`, `close` emitting one `freq = 0` per active sub. No sockets.
- `bridge/tests.rs`: a loopback `UdpSocket` as fake X-Plane, replying from a different port than it listens on. Only socket behavior: draining until `WouldBlock`, `ConnectionReset`, datagrams actually sent, unsubscribe on drop. The `rt-tokio` driver gets the same handful under `#[tokio::test]`.
- `examples/probe.rs` doubles as the manual test: run against X-Plane with a multirotor loaded, confirm attitude signs and that `override_throttles` + `ENGN_thro_use` spin the motors. Anything `#[ignore]`d needs the simulator and says so.

## Relationship to Stable Flight

`stable-flight-sitl-xplane` will hold an `XPlaneUdpBridge` in a `Vehicle`: `actuate` uses it as a `DatarefSink` for motor and surface datarefs; `estimate_state` uses it as a `DatarefSource`, applies frames to a `Latest`, and converts from degrees and the OpenGL local frame (x east, y up, z south) to the core's NED/FRD contract. Because there is no lockstep, it runs under the existing `RealTimeRunner`. That crate depends on this one and on `stable-flight`; this crate depends on neither.

## Open questions

1. When an async consumer appears: whether `rt-tokio` should also offer a `Stream<Item = Frame>` adapter over `next_frame`, and whether the two trait families should be unified behind one trait with an associated future type instead of twins.
2. Exact override set for a multirotor: `override_throttles` alone, or also `override_joystick` for the models px4xplane uses.
3. `ENGN_thro_use` or `ENGN_thro` as the write target under override (px4xplane source is the reference).
4. Whether `freq` below the frame rate is honored exactly.
5. The **verify** items under [Protocol](#protocol) (cross-host replies, int datarefs, write order) and [Drivers](#drivers) (`ConnectionReset` on Windows).

## Spike findings

X-Plane 12, same host, 2026-10-05. Python probe sending `RREF` from an ephemeral port.

- **Transport.** Replies arrived at the ephemeral source port, sent from `127.0.0.1:49001`.
- **Header.** Every reply began `RREF,`.
- **Rate.** 6 subscriptions at `freq = 100`: 54 packets/s, consistent with a frame-rate ceiling (frame rate not measured).
- **Packet cap.** Subscriptions at `freq = 100` (20 distinct paths, then `phi` repeated under distinct indices), 2 s each:

  | Subs | Packet sizes (bytes) | Records/packet | Packets/s |
  |---|---|---|---|
  | 20 | 165 | 20 | 70 |
  | 50 / 100 / 150 | 405 / 805 / 1205 | all | 59–69 |
  | 183 | 1469 | 183 | 64 |
  | 184 | 1469 + 13 | 183 + 1 | 133 |
  | 200 | 1469 + 141 | 183 + 17 | 129 |
  | 400 | 1469 + 277 | 183 + 34 | 197 |

  All indices were seen in every run; no malformed packets. At 400, full packets outnumbered overflow packets about 2:1 (265 vs 132) and the two packet shapes carry only 217 records, yet all 400 indices appeared, so X-Plane rotates which records it sends. Not investigated further; the bridge targets ~20 subscriptions.
