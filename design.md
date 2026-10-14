# xplane-bridge design

Initial design, written before the first line of code. This document states what is true; how it was established lives in [Spike findings](#spike-findings). Items marked **verify** are still assumptions; when one is confirmed, update the fact where it is stated and add the evidence to the findings.

## Goals

- Give an external flight controller a thin link to X-Plane over its built-in UDP protocol: read datarefs every simulator frame, write datarefs every control tick.
- Run on macOS, Windows, and Linux with only `std` networking. No plugin inside X-Plane, no async runtime.
- Implement the protocol once, sans-I/O, so the driver is a thin shell and another transport is another shell, not a fork ([Driver](#driver)).
- Be testable without a simulator: the protocol core is pure, and consumers depend on small traits they can fake.
- Stay simulator-honest: report values in X-Plane's own units and frames. Conversion to a controller's conventions is the caller's job (for Stable Flight, the `stable-flight-sitl-xplane` backend).

## Non-goals

- Lockstep or deterministic stepping. X-Plane does not expose it over UDP; the simulator runs free and the bridge samples it.
- Sensor emulation (IMU noise, GPS, baro). The bridge returns ground truth; sensor models belong upstream.
- Plugin-side code, the X-Plane 12.1 REST/WebSocket API, multiplayer, or the legacy `DATA` output rows. They can be added behind the same traits later if a need appears.
- Frame/unit conversion, `uom` types. Possibly a feature later, as in realflight-bridge; not in v1.
- An async driver. The consumer is sync: Stable Flight's `StateSource`/`Actuator` traits are plain `fn`s, and `RealTimeRunner` paces a thread with `thread::sleep`. Its open issues for this backend (stable-flight #351, #352, #89) keep that shape. An async user drives the public `Protocol` from their own socket ([Why sans-I/O](#why-sans-io)).

## Protocol

This section is the single source for wire-level facts; later sections refer to it.

**Transport.** X-Plane listens on UDP `49000` (configurable in **Settings → Network**). It replies to the source address and port of the socket that sent the request, so one socket serves both directions. Replies come *from* port `49001`, not `49000`. Cross-host behavior: **verify**.

**Encoding.** Integers and floats are little-endian. Strings are null-padded to a fixed width.

| Packet | Direction | Layout | Notes |
|---|---|---|---|
| `RREF` | → X-Plane | `"RREF\0"`, `i32 freq`, `i32 index`, `char[400] path` (413 bytes) | Subscribe `path` at `freq` per second under caller-chosen `index`. `freq = 0` unsubscribes. The same path under different indices is sent once per index. Whether a 400-byte path with no terminator is accepted is untested, so the bridge caps paths at 399. |
| `RREF,` | ← X-Plane | `"RREF,"`, then repeated `{ i32 index, f32 value }` | Holds at most 183 records (1469 bytes). Up to 183 subscriptions arrive in one packet per frame; larger sets spill into extra packets, and not every record may arrive every frame. Rate is capped at the simulator frame rate whatever `freq` asks for. |
| `DREF` | → X-Plane | `"DREF\0"`, `f32 value`, `char[500] path` (509 bytes) | Write one value. Array elements address as `path[i]`. Fire-and-forget; no ack. **verify** float conversion for int datarefs (e.g. `override_throttles`) and that writes apply in arrival order within a frame. |
| `CMND` | → X-Plane | `"CMND\0"`, null-terminated `path` | Trigger a command, e.g. `sim/operation/pause_toggle`. |
| `BECN` | ← multicast `239.255.1.1:49707` | beacon | Discovery. Out of scope for v1; the caller passes an address. |

Datarefs are addressed by string path (`sim/flightmodel/position/phi`). The authoritative list ships with X-Plane as `Resources/plugins/DataRefs.txt`. Writable datarefs the controller needs only take effect after the matching `sim/operation/override/*` flag is set to `1`.

## Architecture

The bridge is sans-I/O: every protocol decision lives in `Protocol`, which never touches a socket, and a *driver* is the smallest loop that moves `Protocol`'s datagrams to and from one. Each part has one job, depends only on the parts above it, and is tested on its own. The file tree is in `AGENTS.md`.

| Part | Module | Job | I/O |
|---|---|---|---|
| `Path` | `path.rs` | Validate a path once (crate-private) | none |
| `codec` | `codec/` | Encode `RREF`/`DREF`/`CMND`; decode `RREF,` into `(index, value)` records | none |
| `Dataref`, `Command` | `datarefs/`, `commands.rs` | Name a path; constants | none |
| `Rate` | `rate.rs` | Nonzero `RREF` rate | none |
| `Subscriptions` | `subscriptions.rs` | Allocate indices; track active `Sub` ↔ `Dataref` | none |
| `Frame`, `FrameBuilder`, `Latest` | `frame.rs` | Merge records from one drain; optionally hold values across frames | none |
| `Protocol` | `protocol/` | Compose the above: calls → `Datagram`s, received bytes → `Frame`s, `Stats` | none |
| traits, `XPlaneUdpBridge` | `bridge/mod.rs`, `bridge/udp.rs` | Driver: `std::net::UdpSocket` around a `Protocol` | socket |
| `FakeXPlane` | `testing.rs` (feature `testing`) | Scripted `DatarefSource`/`DatarefSink` over a `Protocol`, for consumers' tests | none |

`codec` knows bytes, `Subscriptions` knows indices, `FrameBuilder` knows merging; none knows the others. `Protocol` is the only part that wires them together. Consumers depend on the traits `DatarefSource`/`DatarefSink`, not on the driver type.

### Names: `Dataref`, `Command`, `Rate`

Datarefs carry their value type and access in the type, so reading returns the right Rust type and writing a read-only dataref does not compile.

```rust
/// Values a dataref can hold. Sealed. The wire is always `f32`; `Value` converts.
pub trait Value: Copy + sealed::Sealed {
    fn to_wire(self) -> f32;
    fn from_wire(raw: f32) -> Self;
}
// f32: identity. i32: rounds half away from zero, saturates, NaN → 0 (exact up to 2^24).
// bool: `raw != 0.0` (NaN → true), writes 0.0/1.0.

/// Access markers.
pub enum ReadOnly {}
pub enum Writable {}

/// A validated path, shared by `Dataref` and `Command` so the rules live once.
/// Non-empty ASCII, no NUL, at most 399 bytes including any `[i]` suffix:
/// it fits an `RREF` (the tighter limit) with a terminator to spare.
struct Path(Cow<'static, str>);   // crate-private

/// A dataref path of value type `T`, scalar or one array element.
/// Paths are validated once, at construction: a `Dataref` always fits a packet.
// Clone, Debug, PartialEq, Eq, Hash implemented by hand: derives would bound `T`/`A`.
pub struct Dataref<T: Value, A = ReadOnly> { path: Path, index: Option<u16>, _t: PhantomData<fn() -> (T, A)> }

impl<T: Value, A> Dataref<T, A> {
    /// Runtime path for anything without a constant. `InvalidPath` if it breaks a `Path` rule.
    pub fn new(path: impl Into<String>) -> Result<Self, XPlaneError>;
    /// The wire path: `path`, or `path[i]` for an array element.
    pub fn path(&self) -> Cow<'_, str>;
}

/// A fixed-length array dataref (`ENGN_thro_use[16]`). Only arrays have `at`.
pub struct DatarefArray<T: Value, A, const N: usize> { .. }

impl<T: Value, A, const N: usize> DatarefArray<T, A, N> {
    /// Element `i`, or `None` if `i >= N`.
    pub fn at(&self, i: usize) -> Option<Dataref<T, A>>;
    /// All `N` elements in order.
    pub fn iter(&self) -> impl Iterator<Item = Dataref<T, A>> + '_;
}

/// A command path, validated by the same `Path` rules.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Command { path: Path }

/// Requested `RREF` rate. Nonzero: `freq = 0` means unsubscribe on the wire,
/// so it is only ever sent by `unsubscribe`/`close`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate(NonZeroU16);
impl Rate { pub const fn hz(hz: u16) -> Option<Self>; }
```

Constants are built by crate-private `const fn from_static` constructors that check the `Path` rules. Rust evaluates every `const` item, so an invalid constant fails the build and needs no runtime check. `DatarefArray::from_static` also checks that the last element's path, `base[N-1]`, fits, and that `1 <= N <= 65536` (indices are `u16`). Examples:

```rust
pub const PHI: Dataref<f32, Writable> = ..;                    // position, deg
pub const LATITUDE: Dataref<f32> = ..;                         // position, deg (double on the sim side)
pub const THRO_USE: DatarefArray<f32, Writable, 16> = ..;       // engine, 0–1
pub const THROTTLES: Dataref<bool, Writable> = ..;              // overrides
pub const PAUSED: Dataref<bool> = ..;                          // time
```

Access mirrors the writable column of `DataRefs.txt` exactly, with no judgment calls: much of the flight state (`phi`, `P`, `local_x`, `q`) is writable there, and writing it repositions the aircraft, which is a legitimate way to set up a test. The type stops writes the simulator would ignore, not writes that are unwise.

Call sites use constants or values built once at startup, never string literals per tick. Doubles such as `latitude` are `Dataref<f32>`, because `RREF` sends `f32`. At 47° latitude one `f32` step is about 0.4 m, too coarse for position hold or differentiating velocity, so position for control comes from local `x`/`y`/`z` (small values, sub-millimeter steps near the origin); lat/lon are for display and for anchoring the origin ([Relationship to Stable Flight](#relationship-to-stable-flight)). Doc comments on `f64` datarefs state the precision. The type and access of a new constant come from `DataRefs.txt`.

### Handles: `Sub`, `Subscriptions`

```rust
/// Handle for one active subscription of value type `T`. Cheap to copy; reads go through it.
// Copy, Debug, PartialEq, Eq, Hash by hand, as for `Dataref`.
pub struct Sub<T: Value> { index: i32, _t: PhantomData<fn() -> T> }

/// Index bookkeeping only: what is subscribed, under which index, at which rate.
/// Keyed by wire path (`path[i]` for elements, formatted once here). No encoding, no datagrams.
impl Subscriptions {
    /// The existing handle for `dataref`'s wire path (its rate updated), or a newly allocated one.
    pub(crate) fn insert<T: Value, A>(&mut self, dataref: &Dataref<T, A>, rate: Rate) -> Result<Sub<T>, XPlaneError>;
    /// Remove `sub`, returning its wire path, or `None` if not active.
    pub(crate) fn remove<T: Value>(&mut self, sub: Sub<T>) -> Option<Cow<'static, str>>;
    /// Remove every active subscription (shutdown).
    pub(crate) fn drain(&mut self) -> impl Iterator<Item = (i32, Cow<'static, str>)> + '_;
    /// Every active subscription with its wire path, for `resubscribe`.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (i32, &str, Rate)> + '_;

    pub fn is_active<T: Value>(&self, sub: Sub<T>) -> bool;
    pub fn get<T: Value, A>(&self, dataref: &Dataref<T, A>) -> Option<Sub<T>>;
}
```

Handles replace path lookup on the read path: `frame.get(sub)` is a map lookup on an `i32`, with no string hashing per tick. Keying by wire path means `THRO_USE.at(0)` and `Dataref::new("…/ENGN_thro_use[0]")` share one index, and so do the same path under two value types: both read the same raw `f32`, each converted by its own `Value`. The handle carries the dataref's value type, so `frame.get(sub)` returns `Option<T>` and a `bool` override cannot be read as a throttle. Indices are allocated monotonically and not reused after unsubscribe, so a late packet for an old index is never misread as a new subscription. Mutation is crate-private: handles are only ever allocated by `Protocol`, in the same call that encodes the `RREF`, so a handle and its datagram cannot drift apart. Fakes get handles from a `Protocol`, as the real bridge does; the read-only half is public through `Protocol::subscriptions()`.

A handle is only meaningful against frames from the `Protocol` that issued it. Indices start at the same value in every `Protocol`, so a `Sub` from one bridge reads another bridge's frames without error. Tagging handles with an instance id would catch this, but no consumer runs two simulators, so v1 documents it instead.

### Core: `Protocol`, `Datagram`

```rust
/// One outgoing UDP payload. Sized for the largest packet (`DREF`, 509 bytes); no heap.
// PartialEq, Eq by hand over `as_bytes()`, so bytes past `len` never matter.
#[derive(Clone, Copy, Debug)]
pub struct Datagram { buf: [u8; 509], len: usize }
impl Datagram { pub fn as_bytes(&self) -> &[u8]; }

/// Sans-I/O protocol state. Owns `Subscriptions`, a `FrameBuilder`, and `Stats`.
/// Every method is synchronous. `write` and `command` do not allocate; `handle`
/// allocates only while the pending frame grows past its high-water mark;
/// `finish` allocates only the `Frame` it returns.
#[derive(Default)]
pub struct Protocol { .. }

impl Protocol {
    pub fn new() -> Self;

    // Calls → datagrams. The driver sends what comes back, in order.
    /// Re-subscribing a dataref returns its existing handle at the new rate.
    pub fn subscribe<T: Value, A>(&mut self, dataref: &Dataref<T, A>, rate: Rate) -> Result<(Sub<T>, Datagram), XPlaneError>;
    /// The `freq = 0` datagram, or `None` if `sub` is not active.
    pub fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Option<Datagram>;
    /// Infallible: the path was validated when the `Dataref` was built.
    pub fn write<T: Value>(&self, dataref: &Dataref<T, Writable>, value: T) -> Datagram;
    pub fn command(&self, command: &Command) -> Datagram;
    /// `freq = 0` for every active subscription; for shutdown.
    pub fn close(&mut self) -> Vec<Datagram>;
    /// The `RREF` for every active subscription, same indices and rates, so a
    /// restarted X-Plane streams again and existing handles stay valid. Counts
    /// in `Stats::resubscribes`.
    pub fn resubscribe(&mut self) -> Vec<Datagram>;

    // Received bytes → frames. The driver calls `handle` once per datagram
    // it drained and `finish` once per drain.
    /// Decode one datagram and fold it into the pending frame. Anything that
    /// does not decode as `RREF,` (wrong header included) counts in
    /// `Stats::malformed` and is skipped. Never errs.
    pub fn handle(&mut self, bytes: &[u8], at: Instant);
    /// The merged frame since the last `finish`, or `None`.
    pub fn finish(&mut self) -> Option<Frame>;

    pub fn stats(&self) -> Stats;
    pub fn subscriptions(&self) -> &Subscriptions;
}

/// Counters since `Protocol::new`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats { pub packets: u64, pub malformed: u64, pub dropped_records: u64, pub resubscribes: u64 }
```

Each call maps to exactly one outgoing datagram (`close` and `resubscribe` to one per subscription), so methods *return* the datagram instead of queueing it; there is no outbox and no callback. `Protocol` is public: it is the whole protocol for anyone bringing their own socket (`socket2`, `mio`, an embedded stack), and it is what the tests exercise.

`Protocol` is the only place the pure parts meet. `subscribe` asks `Subscriptions` for a handle and encodes the `RREF`; `unsubscribe` and `close` encode `freq = 0` for what `Subscriptions` removed; `resubscribe` re-encodes what `Subscriptions` holds, so `Subscriptions` also keeps each subscription's `Rate`. `handle` decodes with `codec`, keeps records whose index `Subscriptions` reports active, counts the rest in `Stats::dropped_records`, and pushes the kept ones into `FrameBuilder`.

`write` and `command` take `&self` and read no state: they are `codec` calls routed through `Protocol` so a transport has one entry point. Because the sink half needs no `Protocol` state, a driver could later split into a source half and a sink half that each own a clone of the socket. Nothing needs that today: the backend's `Vehicle` owns one bridge and uses both halves from one thread.

### Values: `Frame`, `FrameBuilder`, `Latest`

```rust
/// Values that arrived in one drain, newest per subscription. `received_at` is
/// the arrival time of the last datagram merged in.
pub struct Frame { values: HashMap<i32, f32>, received_at: Instant }

impl Frame {
    /// The value for `sub`, converted to its type, or `None` if it did not arrive.
    pub fn get<T: Value>(&self, sub: Sub<T>) -> Option<T>;
    pub fn received_at(&self) -> Instant;
    /// For fakes and tests: `Frame::empty(t).with(phi, 1.5).with(armed, true)`.
    pub fn empty(received_at: Instant) -> Self;
    pub fn with<T: Value>(self, sub: Sub<T>, value: T) -> Self;
}

/// Crate-private; `Protocol` is its only user.
impl FrameBuilder {
    /// Fold one packet's records in; later records overwrite earlier ones.
    /// Knows nothing of `Subscriptions`; `Protocol` filters before pushing.
    pub(crate) fn push(&mut self, records: impl Iterator<Item = (i32, f32)>, at: Instant);
    /// The merged frame, or `None` if nothing was pushed. Resets the builder.
    pub(crate) fn finish(&mut self) -> Option<Frame>;
}

impl Latest {
    pub fn apply(&mut self, frame: &Frame);
    /// Last known value and when it arrived.
    pub fn get<T: Value>(&self, sub: Sub<T>) -> Option<(T, Instant)>;
}
```

Storage stays raw `f32` keyed by index, as it came off the wire; conversion happens in `get`, where the handle supplies the type. A `Frame` reports only what arrived; a `Sub` absent from it (rate below poll rate, or a split reply that skipped it) reads `None`. Callers who want held values compose `Latest` on top instead of each writing their own cache. Merging every packet in a drain, rather than keeping the newest, is what keeps slower subscriptions and split replies intact.

### Traits

```rust
pub trait DatarefSource {
    /// Subscribe one dataref; one call per dataref keeps each handle's type.
    fn subscribe<T: Value, A>(&mut self, dataref: &Dataref<T, A>, rate: Rate) -> Result<Sub<T>, XPlaneError>;
    /// Stop streaming. An unknown or already-removed handle is ignored.
    fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Result<(), XPlaneError>;
    /// Drain everything that arrived; `None` if nothing did. Never blocks.
    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError>;
    /// Resend every active `RREF` ([Liveness](#liveness)). Handles stay valid.
    fn resubscribe(&mut self) -> Result<(), XPlaneError>;
    fn stats(&self) -> Stats;
}

pub trait DatarefSink {
    /// One `DREF`. Only `Writable` datarefs compile. Errs only on I/O.
    fn write<T: Value>(&mut self, dataref: &Dataref<T, Writable>, value: T) -> Result<(), XPlaneError>;
    /// One `CMND`.
    fn command(&mut self, command: &Command) -> Result<(), XPlaneError>;
}
```

An actuator-only component takes `impl DatarefSink`; a state estimator takes `impl DatarefSource`. `XPlaneUdpBridge` implements both. A fake source is a `Protocol` plus a queue of `Frame`s.

The split follows who uses what, not one trait per method. `DatarefSource` keeps subscription management, `poll`, and `stats` together because the one component that reads state (the backend's `StateSource`) subscribes at startup, polls every tick, and resubscribes and reports stats on silence. Splitting it would make every fake implement two traits for one role.

The traits are for static dispatch: generic methods already rule out `dyn`, and nothing in the crate needs it.

### Driver

A driver is the smallest loop that moves `Datagram`s between a `Protocol` and a socket. It contains no protocol logic.

```rust
// bridge/udp.rs
pub struct XPlaneUdpBridge { proto: Protocol, sock: std::net::UdpSocket, addr: SocketAddr }

impl DatarefSink for XPlaneUdpBridge {
    fn write<T: Value>(&mut self, d: &Dataref<T, Writable>, v: T) -> Result<(), XPlaneError> {
        let dg = self.proto.write(d, v);
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
```

Socket rules:

- `connect(addr: SocketAddr) -> Result<Self>` binds an ephemeral port on the unspecified address of `addr`'s family (`0.0.0.0` or `[::]`, so cross-host works), sets the socket non-blocking, and stores `addr`. No handshake exists, so "connected" only means the socket is ready. The socket is never `connect()`ed, because replies come from `49001` ([Protocol](#protocol)) and a connected socket would drop them. Replies are not filtered by source: anything that is not a valid `RREF,` for an active index is counted and dropped by `Protocol`, which never sees addresses.
- Every other call is one `Protocol` call plus a `send_to` per returned datagram. Eight motor writes per tick are eight datagrams, which is fine on loopback.
- `poll` errs only on socket errors; malformed datagrams are `Protocol`'s concern (counted, logged at `debug!`, skipped).
- `is_no_data` (in `bridge/mod.rs`) treats `WouldBlock` and `ConnectionReset` as "drained". On Windows `ConnectionReset` follows a `send_to` that hit a closed port (X-Plane not running yet). **verify** on Windows; the alternative, disabling `SIO_UDP_CONNRESET`, needs a non-`std` call.
- `Drop` sends `Protocol::close()` so X-Plane stops streaming to a dead port. Send errors are logged and ignored; `Drop` never panics. `Drop` does not release overrides: it does not know which the caller set ([Relationship to Stable Flight](#relationship-to-stable-flight)).

#### Why sans-I/O

This is the split `rustls` (`read_tls`/`write_tls`) and `quinn-proto`/`quinn` use. With one driver it still pays for itself:

- `Protocol` tests need no sockets and assert datagrams byte-for-byte; driver tests are a few loopback smoke tests.
- Another transport for the same packets (a pcap replayer, an in-process fake X-Plane for CI, a tokio socket) is another small driver over the public `Protocol`, not a fork. The 12.1 WebSocket API is a different protocol (JSON) and would need its own core, not just a driver. realflight-bridge's `async_impl.rs` files, which duplicate each bridge's logic for the async runtime, are the fork this avoids.

Rejected alternatives: shipping an async driver behind a feature now (no consumer; Stable Flight's host stack is sync, see [Non-goals](#non-goals)); a `maybe-async`-style macro emitting sync and async from one source (hides control flow, mixes I/O with logic); an async-only core with `block_on` in a sync shim (a runtime inside a 250 Hz loop).

### Liveness

UDP has no connection, so the bridge cannot tell an X-Plane that has not started yet, one that restarted, and one that is stalled apart; each reads as `poll` returning `None`. It reports facts and leaves the policy to the caller:

- **Silence.** `Frame::received_at` (or `Latest`) gives the age of the newest state. The caller picks a timeout.
- **Restart, or late start.** A restarted X-Plane has forgotten every `RREF`, and one that was not running when the bridge subscribed never got them. After a silence, the caller calls `resubscribe` (repeating it while silent is harmless: same indices, same rates). `Stats::resubscribes` counts them, so a consumer can report reconnects (stable-flight #309). Whether subscriptions also survive an aircraft or scenery reload: **verify**.
- **Pause.** `sim/time/paused` (`int` boolean, read-only, so `Dataref<bool>`) marks a paused simulator. This only helps if X-Plane keeps streaming `RREF` while paused, including the pause on focus loss: **verify**. If it does, a paused sim streams frozen values with `paused` set; if not, a pause reads as silence and `paused` is never seen. Either way, a pause is a host stall to the controller (stable-flight #302): nothing in it is flight data.
- **Simulator time.** `sim/time/total_running_time_sec` (`float`, seconds) stamps state in simulator time, independent of arrival jitter (stable-flight #89, #81). It is `f32`: 1 ms steps hold for about 4.6 hours of running time.

### Errors

Codec errors are separate from bridge errors, so the public bridge error lists only what a bridge call can return.

```rust
// codec: encoding cannot fail, because paths are validated on construction, so there is no EncodeError
pub enum DecodeError { BadHeader, Truncated { len: usize } }   // `len` not 5 + 8k

// bridge
pub enum XPlaneError {
    Io(std::io::Error),             // socket bind/send/recv
    InvalidPath(PathError),         // Dataref::new / Command::new only
    TooManySubscriptions,           // index space exhausted (i32, so practically never)
}
pub enum PathError { Empty, TooLong { len: usize, max: usize }, NotAscii, ContainsNul }
```

Invalid rates and unwritable datarefs are type errors, not runtime errors, so `XPlaneError` has no variant for them. On the per-tick path only `Io` is possible.

`DecodeError` never leaves the bridge: simulator input never panics the library and never fails `poll`.

## Timing

- State rate is capped at X-Plane's frame rate ([Protocol](#protocol)), typically 60–120 Hz. A 250 Hz controller gets `None` from `poll` on most ticks; `Latest` or the caller holds values in between. The bridge does not interpolate.
- State age is one frame plus loopback latency; `Frame::received_at` lets callers measure it.
- The flight model runs several physics steps per frame (**Settings → General → Flight models per frame**); writes land at frame boundaries, so actuator latency is also up to one frame.
- The simulator may pause when it loses focus; see [Liveness](#liveness) for what `poll` then reports.
- Frame-rate state quantizes anything measured from it. At 60 fps a period or delay is known to about 17 ms: for a relay-tuner oscillation (stable-flight #310, Tu 40–70 ms) that is a quarter to a third of a period. Raising the frame rate, not the subscription rate, is the fix.

## Testing

- `codec/tests.rs`: table-driven byte-level cases for every packet type, including boundary paths (exactly 399 bytes, null-terminated in the 400-byte field), empty replies, truncated records, a 183-record reply.
- `datarefs/`, `frame.rs`, `subscriptions.rs`: unit tests for `Path` rules (empty, 399 accepted and 400 rejected, non-ASCII, NUL), `Value` round trips (`i32` rounding, `bool` from nonzero), `DatarefArray::at` bounds, index allocation, handle reuse on re-subscribe (including `at(0)` vs the same path built by hand), no index reuse after unsubscribe, merging across packets, `Latest` hold behavior. No sockets.
- `protocol/tests.rs`: the bulk of the behavior tests. Feed `Protocol` byte vectors and assert the returned `Datagram`s byte-for-byte: subscribe/unsubscribe/write/command round trips, malformed input counted and skipped, records for inactive indices dropped and counted, split replies merged by `finish`, `close` emitting one `freq = 0` per active sub, `resubscribe` re-emitting each active `RREF` at its current rate and none for removed subs. No sockets.
- `bridge/tests.rs`: a loopback `UdpSocket` as fake X-Plane, replying from a different port than it listens on. Only socket behavior: draining until `WouldBlock`, `ConnectionReset`, datagrams actually sent, unsubscribe on drop.
- `examples/probe.rs` doubles as the manual test: run against X-Plane with a multirotor loaded, confirm attitude signs and that `override_throttles` + `ENGN_thro_use` spin the motors. Anything `#[ignore]`d needs the simulator and says so.

## Relationship to Stable Flight

`stable-flight-sitl-xplane` will hold an `XPlaneUdpBridge` in a `Vehicle`: `actuate` uses it as a `DatarefSink` for motor and surface datarefs; `state` (`StateSource`) uses it as a `DatarefSource`, applies frames to a `Latest`, and converts from degrees and the OpenGL local frame (x east, y up, z south) to the core's NED/FRD contract. Because there is no lockstep, it runs under the existing `RealTimeRunner`. That crate depends on this one and on `stable-flight`; this crate depends on neither.

What the backend's issues (stable-flight #351, #352, #79, #302, #309) need from this crate, and where the line falls:

- **Position.** Control uses local `x`/`y`/`z` ([Names](#names-dataref-command-rate)). The local origin moves when scenery reloads; the backend detects it from `sim/flightmodel/position/lat_ref`/`lon_ref` (latitude and longitude of the local origin) changing, or a jump in local position, and re-anchors its `GeodeticOrigin` from lat/lon. The mission demo's `--origin` comes from the same lat/lon.
- **Sensor synthesis.** #79 synthesizes IMU from truth, so the dataref constants cover body rates (`sim/flightmodel/position/P`/`Q`/`R`, deg/s) and world acceleration (`local_ax`/`local_ay`/`local_az`, m/s², OpenGL frame).
- **Host faults.** The backend maps [Liveness](#liveness) to the core: silence past its timeout triggers `resubscribe` and counts as a reconnect; `paused` and a silence both count as a stall for the autotune.
- **Shutdown.** Handing control back means writing every override the backend set back to `0`, then dropping the bridge. The bridge's `Drop` only unsubscribes.

## Open questions

1. Exact override set for a multirotor: `override_throttles` alone, or also `override_joystick` for the models px4xplane uses.
2. Which per-surface deflection datarefs to write under `override_control_surfaces` for the plane and tailsitter models (stable-flight #333, #348, #349). `DataRefs.txt` names `sim/flightmodel/controls/lail1def` as an example.
3. Whether `freq` below the frame rate is honored exactly.
4. The **verify** items, stated where they apply: [Protocol](#protocol) (cross-host replies, int datarefs, write order), [Driver](#driver) (`ConnectionReset` on Windows), [Liveness](#liveness) (subscriptions across a reload, streaming while paused).

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

X-Plane 12 `DataRefs.txt` (header dated 2026-06-24), 2026-10-13:

- **Paths.** `sim/time/paused` (`int` boolean, read-only), `sim/time/total_running_time_sec` (`float`), `sim/flightmodel/position/lat_ref`/`lon_ref` (`float`, the origin of local coordinates), `local_ax/ay/az` (`float`, m/s², OpenGL frame), `P`/`Q`/`R` (`float`, deg/s), `local_x` (`double`), `latitude` (`double`), and `sim/operation/override/override_throttles`, `override_joystick`, `override_control_surfaces` (`int` boolean, writable) all exist.
- **Throttle target.** `override_throttles` reads "Use ENGN_thro_use to control them", so `ENGN_thro_use` is the write target. It is `float[16]`.
- **Access.** `phi`, `P`, `local_x`, `local_ax`, `q` (`float[4]`), and `total_running_time_sec` are writable; `latitude` and `sim/time/paused` are not.
