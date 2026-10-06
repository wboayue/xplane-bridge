# xplane-bridge design

Initial design, written before the first line of code. This document states what is true; how it was established lives in [Spike findings](#spike-findings). Items marked **verify** are still assumptions; when one is confirmed, update the fact where it is stated and add the evidence to the findings.

## Goals

- Give an external flight controller a thin, synchronous link to X-Plane over its built-in UDP protocol: read datarefs every simulator frame, write datarefs every control tick.
- Run on macOS, Windows, and Linux with only `std` networking. No plugin inside X-Plane, no async runtime.
- Be testable without a simulator: the codec and bookkeeping are pure, and consumers depend on small traits they can fake.
- Stay simulator-honest: report values in X-Plane's own units and frames. Conversion to a controller's conventions is the caller's job (for Stable Flight, the `stable-flight-sitl-xplane` backend).

## Non-goals

- Lockstep or deterministic stepping. X-Plane does not expose it over UDP; the simulator runs free and the bridge samples it.
- Sensor emulation (IMU noise, GPS, baro). The bridge returns ground truth; sensor models belong upstream.
- Plugin-side code, the X-Plane 12.1 REST/WebSocket API, multiplayer, or the legacy `DATA` output rows. They can be added behind the same traits later if a need appears.
- Frame/unit conversion, `uom` types. Possibly a feature later, as in realflight-bridge; not in v1.

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

The bridge is a thin I/O shell around pure parts. Each part has one job and is tested on its own:

| Part | Job | I/O |
|---|---|---|
| `codec` | Encode `RREF`/`DREF`/`CMND`; decode `RREF,` into records | none |
| `Subscriptions` | Assign subscription indices, map `Dataref` ↔ `Sub`, produce the `RREF` packets to send | none |
| `FrameBuilder` | Merge decoded records from one drain into a `Frame` | none |
| `Latest` | Optional cache: last known value per `Sub` across frames | none |
| `XPlaneUdpBridge` | Own the socket; send what `Subscriptions` produces; drain, decode, feed `FrameBuilder` | socket |

Consumers depend on two traits, `DatarefSource` (read) and `DatarefSink` (write), not on the UDP type.

### Crate layout

```
src/
  lib.rs            pub use of the items below; crate docs
  error.rs          XPlaneError (thiserror)
  codec/            pure encode/decode; EncodeError, DecodeError; tests.rs with byte-table cases
  datarefs/         Dataref type and constants grouped by X-Plane path prefix (position, engine, joystick, overrides)
  commands.rs       Command type and constants (operation/pause_toggle, ...)
  subscriptions.rs  Sub, Subscriptions
  frame.rs          Frame, FrameBuilder, Latest
  bridge/
    mod.rs          DatarefSource, DatarefSink, Stats
    udp.rs          XPlaneUdpBridge
    tests.rs        socket behavior against a loopback UdpSocket standing in for X-Plane
examples/
  probe.rs          subscribe attitude, print at 1 Hz, spin motors; the spike tool
```

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

impl Subscriptions {
    /// Returns the handle and the `RREF` packet to send. Re-subscribing a
    /// dataref returns its existing handle with the new rate.
    pub fn subscribe(&mut self, dataref: &Dataref, rate_hz: i32) -> Result<(Sub, RrefPacket), XPlaneError>;
    /// Returns the `freq = 0` packet, or `None` if `sub` is not active.
    pub fn unsubscribe(&mut self, sub: Sub) -> Option<RrefPacket>;
    pub fn is_active(&self, sub: Sub) -> bool;
    pub fn get(&self, dataref: &Dataref) -> Option<Sub>;
    /// `freq = 0` packets for every active subscription (shutdown).
    pub fn unsubscribe_all(&mut self) -> Vec<RrefPacket>;
}
```

Handles replace path lookup on the read path: `frame.get(sub)` is a map lookup on an `i32`, with no string hashing per tick and no ambiguity between `THRO_USE.at(0)` and `Dataref::new("…/ENGN_thro_use[0]")`. `Subscriptions` is public so fakes assign handles the same way the real bridge does. Negative `rate_hz` is `InvalidRate`. Indices are allocated monotonically and not reused after unsubscribe, so a late packet for an old index is never misread as a new subscription.

### Values: `Frame`, `FrameBuilder`, `Latest`

```rust
/// Values that arrived in one `poll`, newest per `Sub`.
pub struct Frame { values: HashMap<Sub, f32>, received_at: Instant }

impl Frame {
    pub fn get(&self, sub: Sub) -> Option<f32>;
    pub fn received_at(&self) -> Instant;
    /// For fakes and tests.
    pub fn from_values(received_at: Instant, values: impl IntoIterator<Item = (Sub, f32)>) -> Self;
}

impl FrameBuilder {
    /// Fold one decoded packet in; later records overwrite earlier ones.
    /// Records whose index is not active in `subs` are dropped and counted.
    pub fn push(&mut self, subs: &Subscriptions, records: impl Iterator<Item = (i32, f32)>, at: Instant);
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

/// Counters since connect.
pub struct Stats { pub packets: u64, pub malformed: u64, pub dropped_records: u64 }
```

An actuator-only component takes `impl DatarefSink`; a state estimator takes `impl DatarefSource`. `XPlaneUdpBridge` implements both. A fake source is `Subscriptions` plus a queue of `Frame`s.

### `XPlaneUdpBridge`

Only socket concerns live here; everything else is delegated.

- `connect(addr: SocketAddr) -> Result<Self>` binds an ephemeral local port, sets the socket non-blocking, and stores `addr`. No handshake exists, so "connected" only means the socket is ready. The socket is never `connect()`ed, and replies are accepted from any port on `addr`'s IP, since they come from `49001` ([Protocol](#protocol)).
- `subscribe`/`unsubscribe` ask `Subscriptions` for packets and `send_to` them.
- `poll` loops `recv_from` until `WouldBlock`, decodes each datagram, pushes it into `FrameBuilder`, then returns `finish()`. Non-`RREF,` datagrams are ignored.
- A malformed datagram is skipped: logged at `debug!`, counted in `Stats::malformed`, and draining continues. `poll` errs only on socket errors.
- `ConnectionReset` from `recv_from` is treated as "no data" on all platforms. On Windows it follows a `send_to` that hit a closed port (X-Plane not running yet). **verify** on Windows; the alternative, disabling `SIO_UDP_CONNRESET`, needs a non-`std` call.
- `write`/`command` encode and `send_to` one datagram each. Eight motor writes per tick are eight datagrams, which is fine on loopback.
- `Drop` sends `unsubscribe_all()` so X-Plane stops streaming to a dead port. Send errors are logged and ignored; `Drop` never panics.

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
- `subscriptions.rs`, `frame.rs`: unit tests for index allocation, handle reuse on re-subscribe, no index reuse after unsubscribe, merging across packets, dropping records for inactive indices, `Latest` hold behavior. No sockets.
- `bridge/tests.rs`: a loopback `UdpSocket` as fake X-Plane, replying from a different port than it listens on. Covers draining until `WouldBlock`, skipping malformed datagrams, `ConnectionReset`, unsubscribe on drop.
- `examples/probe.rs` doubles as the manual test: run against X-Plane with a multirotor loaded, confirm attitude signs and that `override_throttles` + `ENGN_thro_use` spin the motors. Anything `#[ignore]`d needs the simulator and says so.

## Relationship to Stable Flight

`stable-flight-sitl-xplane` will hold an `XPlaneUdpBridge` in a `Vehicle`: `actuate` uses it as a `DatarefSink` for motor and surface datarefs; `estimate_state` uses it as a `DatarefSource`, applies frames to a `Latest`, and converts from degrees and the OpenGL local frame (x east, y up, z south) to the core's NED/FRD contract. Because there is no lockstep, it runs under the existing `RealTimeRunner`. That crate depends on this one and on `stable-flight`; this crate depends on neither.

## Open questions

1. Exact override set for a multirotor: `override_throttles` alone, or also `override_joystick` for the models px4xplane uses.
2. `ENGN_thro_use` or `ENGN_thro` as the write target under override (px4xplane source is the reference).
3. Whether `freq` below the frame rate is honored exactly.
4. Cross-host reply behavior (port and source).
5. The **verify** items under [Protocol](#protocol) and [`XPlaneUdpBridge`](#xplaneudpbridge).

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
