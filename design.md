# xplane-bridge design

Initial design, written before the first line of code. Items marked **verify** are assumptions to confirm in the protocol spike against a running X-Plane 12; update this document when they are.

## Goals

- Give an external flight controller a thin, synchronous link to X-Plane over its built-in UDP protocol: read datarefs every simulator frame, write datarefs every control tick.
- Run on macOS, Windows, and Linux with only `std` networking. No plugin inside X-Plane, no async runtime.
- Be testable without a simulator: the wire codec is pure, and the bridge is a trait that callers can mock.
- Stay simulator-honest: report values in X-Plane's own units and frames. Conversion to a controller's conventions is the caller's job (for Stable Flight, the `stable-flight-sitl-xplane` backend).

## Non-goals

- Lockstep or deterministic stepping. X-Plane does not expose it over UDP; the simulator runs free and the bridge samples it.
- Sensor emulation (IMU noise, GPS, baro). The bridge returns ground truth; sensor models belong upstream.
- Plugin-side code, the X-Plane 12.1 REST/WebSocket API, multiplayer, or the legacy `DATA` output rows. They can be added behind the same trait later if a need appears.
- Frame/unit conversion, `uom` types. Possibly a feature later, as in realflight-bridge; not in v1.

## Protocol summary

X-Plane listens on UDP `49000` (configurable in **Settings → Network**) and replies to the source address and port of whichever socket sent the request, so one socket serves both directions. Replies come *from* port `49001`, not `49000` (confirmed, X-Plane 12, same host), so the bridge must never `connect()` its socket or filter replies by the address it sends to. All integers and floats are little-endian; the codec uses `to_le_bytes`/`from_le_bytes` explicitly, never native order. Strings are null-padded to a fixed width.

| Packet | Direction | Layout | Notes |
|---|---|---|---|
| `RREF` | → X-Plane | `"RREF\0"`, `i32 freq`, `i32 index`, `char[400] path` (413 bytes) | Subscribe `path` at `freq` per second under caller-chosen `index`. `freq = 0` unsubscribes. |
| `RREF,` | ← X-Plane | `"RREF"` + one byte, then repeated `{ i32 index, f32 value }` | One packet per frame holding every due subscription. Sent at most once per rendered frame regardless of `freq`. X-Plane 12 sends `RREF,` (confirmed); the decoder also accepts `\0` as the fifth byte for safety. Six subscriptions arrive in one 53-byte packet (confirmed). **verify** the packet size cap and whether a set of ~20 splits across packets. |
| `DREF` | → X-Plane | `"DREF\0"`, `f32 value`, `char[500] path` (509 bytes) | Write one value. Array elements address as `path[i]`. Fire-and-forget; no ack. **verify** that X-Plane converts the float for int datarefs (e.g. `override_throttles`). |
| `CMND` | → X-Plane | `"CMND\0"`, null-terminated `path` | Trigger a command, e.g. `sim/operation/pause_toggle`. |
| `BECN` | ← multicast `239.255.1.1:49707` | beacon | Discovery. Out of scope for v1; the caller passes an address. |

Datarefs are addressed by string path (`sim/flightmodel/position/phi`). The authoritative list ships with X-Plane as `Resources/plugins/DataRefs.txt`. Writable datarefs the controller needs only take effect after the matching `sim/operation/override/*` flag is set to `1`.

## Crate layout

```
src/
  lib.rs          pub use of the items below; crate docs
  error.rs        XPlaneError (thiserror)
  codec/          pure encode/decode of `RREF`, `RREF,` reply, `DREF`, `CMND`; tests.rs with byte-table cases
  datarefs/       typed dataref constants grouped by X-Plane path prefix (position, engine, joystick, overrides)
  bridge/
    mod.rs        XPlaneBridge trait, Frame (decoded RREF reply)
    udp.rs        XPlaneUdpBridge: std::net::UdpSocket implementation
    tests.rs      bridge logic against a loopback UdpSocket standing in for X-Plane
examples/
  probe.rs        subscribe attitude, print at 1 Hz, spin motors; the spike tool
```

### Core types

```rust
/// A dataref path plus optional array index. Constants use `Cow::Borrowed`
/// (`const fn`); `Dataref::new(impl Into<String>)` builds one at runtime for
/// paths without a constant. `at(i)` returns a copy addressing element `i`.
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct Dataref { path: Cow<'static, str>, index: Option<u16> }

/// Merged values from every `RREF,` packet drained by one `poll`.
pub struct Frame {
    values: HashMap<i32, f32>,      // subscription index → latest value
    table: Arc<IndexTable>,         // snapshot of the bridge's index → Dataref map
    received_at: Instant,           // arrival time of the newest packet
}

pub trait XPlaneBridge {
    /// Subscribe `datarefs` at `rate_hz`; the bridge assigns indices and remembers them.
    fn subscribe(&mut self, datarefs: &[Dataref], rate_hz: i32) -> Result<(), XPlaneError>;
    /// Drain the socket; merge every packet into one frame, or `None` if nothing arrived. Never blocks.
    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError>;
    /// Write one value (`DREF`). Returns once the datagram is handed to the OS.
    fn write(&mut self, dataref: Dataref, value: f32) -> Result<(), XPlaneError>;
    /// Trigger a command (`CMND`).
    fn command(&mut self, path: &str) -> Result<(), XPlaneError>;
}
```

`Frame::get(&Dataref) -> Option<f32>` resolves through `table`, so callers use dataref constants, not integers. The bridge holds the table as `Arc<IndexTable>` and `subscribe` updates it with `Arc::make_mut`, so a `Frame` owns its snapshot and borrows nothing from the bridge. Datarefs absent from every packet drained in this poll (rate lower than poll rate) read as `None`; the caller holds its own last-known values.

### `XPlaneUdpBridge`

- `connect(addr: SocketAddr) -> Result<Self>` binds an ephemeral local port, sets the socket non-blocking, and stores `addr`. No handshake exists, so "connected" only means the socket is ready. X-Plane replies to the ephemeral port (confirmed, same host). **verify** the same holds when the client is on another host.
- `subscribe` sends one `RREF` per dataref and records `index → Dataref`. Re-subscribing the same dataref reuses its index. `rate_hz` is `i32` to match the wire; negative values are `InvalidRate`.
- `poll` loops `recv_from` until `WouldBlock`, decoding each `RREF,` packet and merging it into one frame: later packets overwrite earlier values per index. Merging, rather than keeping only the newest packet, keeps values from slower subscriptions and from a subscription set split across packets. Packets that aren't `RREF,` are ignored. Replies are accepted from any port on X-Plane's IP, since they arrive from `49001` rather than the `49000` the bridge sends to.
- A malformed packet is skipped: logged at `debug!`, counted (`XPlaneUdpBridge::malformed_count()`), and draining continues. `poll` errs only on socket errors, so a caller using `?` is never stopped by bad simulator input.
- On Windows, `recv_from` on a UDP socket reports `ConnectionReset` after a `send_to` reached a closed port (X-Plane not running yet). `poll` treats `ConnectionReset` as "no data" on all platforms. **verify** on Windows; the alternative is disabling `SIO_UDP_CONNRESET`, which needs a non-`std` call.
- `write` encodes `DREF` and `send_to`s it. One datagram per value; a per-tick batch of eight motor writes is eight datagrams, which is fine on loopback. **verify** that X-Plane applies writes in arrival order within a frame.
- `Drop` sends `freq = 0` for every subscription so X-Plane stops streaming to a dead port. Send errors are logged and ignored; `Drop` never panics.

### Errors

```rust
pub enum XPlaneError {
    Io(std::io::Error),            // socket bind/send/recv
    PathTooLong { path, max },     // > 400 (RREF) or 500 (DREF) bytes
    Malformed { reason: &'static str }, // codec decode failure; poll skips the packet and never returns this
    InvalidRate(i32),              // negative subscription rate
    TooManySubscriptions,          // index space exhausted (i32, so practically never)
}
```

Simulator input never panics the library and never fails `poll`: a malformed packet is skipped and counted.

## Timing

- Rate ceiling is X-Plane's frame rate (60–120 Hz typical). A 250 Hz controller sees `None` from `poll` on most ticks and must reuse its last frame. The bridge does not interpolate.
- State age is one frame plus loopback latency; `Frame::received_at` lets callers measure it.
- The flight model runs several physics steps per frame (**Settings → General → Flight models per frame**); writes land at frame boundaries, so actuator latency is also up to one frame.
- The simulator pauses when it loses focus unless that setting is disabled, and `poll` then returns `None` indefinitely. The bridge does not detect this; callers that care should watch `received_at`.

## Testing

- `codec/tests.rs`: table-driven byte-level cases for every packet type, including boundary paths (400/500 bytes, one over), empty replies, truncated records.
- `bridge/tests.rs`: bind a second `UdpSocket` on loopback as a fake X-Plane, drive `subscribe`/`poll`/`write` through it, assert the datagrams it receives and the frames the bridge returns. Covers merging drained packets, skipping malformed packets, `WouldBlock` and `ConnectionReset` handling, and unsubscribe on drop.
- `examples/probe.rs` doubles as the manual test: run against X-Plane with a multirotor loaded, confirm attitude signs and that `override_throttles` + `ENGN_thro_use` spin the motors. Anything `#[ignore]`d needs the simulator and says so.

## Relationship to Stable Flight

`stable-flight-sitl-xplane` will wrap `XPlaneUdpBridge` in a `Vehicle`: `actuate` writes motor and surface datarefs, `estimate_state` polls and converts the last frame from degrees and the OpenGL local frame (x east, y up, z south) to the core's NED/FRD contract. Because there is no lockstep, it runs under the existing `RealTimeRunner`. That crate depends on this one and on `stable-flight`; this crate depends on neither.

## Open questions (for the spike)

1. Max `RREF,` packet size and splitting behavior with ~20 subscriptions.
2. Does `freq` above the frame rate change anything, or is it purely a ceiling? Partly answered: `freq = 100` gave ~54 packets/s, consistent with a frame-rate ceiling (frame rate not measured); not yet checked whether `freq` below the frame rate is honored exactly.
3. Exact override set needed for a multirotor: `override_throttles` alone, or also `override_joystick` for the models px4xplane uses.
4. Whether `ENGN_thro_use` or `ENGN_thro` is the right write target under override (px4xplane source is the reference).
5. Cross-host behavior: does X-Plane reply to the request's source port, or to a fixed port? Same host: replies go to the request's source port and come from `49001`. Cross-host still open.
