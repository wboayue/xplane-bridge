# xplane-bridge

*A Rust library to interface external flight controllers with the X-Plane simulator.*

[![CI](https://github.com/wboayue/xplane-bridge/actions/workflows/ci.yaml/badge.svg)](https://github.com/wboayue/xplane-bridge/actions/workflows/ci.yaml)
[![License:MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![crates.io](https://img.shields.io/crates/v/xplane-bridge.svg)](https://crates.io/crates/xplane-bridge)
[![Documentation](https://img.shields.io/docsrs/xplane-bridge)](https://docs.rs/xplane-bridge/latest/xplane_bridge/index.html)

## Overview

[X-Plane](https://www.x-plane.com/) is a flight simulator with a blade-element aerodynamic model that runs natively on macOS (Apple Silicon), Windows, and Linux. It exposes its state and controls as *datarefs* that any process can read and write over UDP, with no plugin required.

**xplane-bridge** wraps that UDP protocol so an external flight controller can:

* Subscribe to datarefs (`RREF`) and receive them every simulator frame: attitude, body rates, position, velocity, acceleration.
* Write datarefs (`DREF`) to drive the aircraft: per-engine throttle, stick deflections, control surface deflections, and the override flags that hand control to the bridge.
* Send commands (`CMND`) such as pause or reset.

It is the X-Plane counterpart of [realflight-bridge](https://github.com/wboayue/realflight-bridge) and is used by the [Stable Flight](https://github.com/wboayue/stable-flight) `stable-flight-sitl-xplane` backend, but it has no dependency on either.

## Design

* **Sans-I/O.** The whole protocol lives in `Protocol`, which never touches a socket: calls return `Datagram`s to send, received bytes go in through `handle`. `XPlaneUdpBridge` is a thin driver that moves those datagrams over a `std::net::UdpSocket`. Bring your own socket (`socket2`, `mio`, an embedded stack) by driving `Protocol` yourself.
* **Sync, `std` only.** No async runtime; reads are non-blocking so a fixed-rate control loop never stalls on the simulator. For async, drive `Protocol` from a `tokio::net::UdpSocket` yourself.
* **Raw simulator units.** Values are returned as X-Plane reports them (degrees, degrees per second, OpenGL local frame: x east, y up, z south). Frame and unit conversion belong to the caller.
* **Small traits.** Consumers depend on `DatarefSource` (subscribe, poll) and `DatarefSink` (write, command), not on the driver; controllers can be tested against `testing::FakeXPlane` without a simulator ([Testing without a simulator](#testing-without-a-simulator)).
* **Typed datarefs and handles.** Each dataref constant carries its value type (`f32`, `i32`, `bool`) and whether it is writable. `subscribe` returns a typed `Sub<T>`, so `frame.get(sub)` returns `Option<T>` with no string lookup; writing a read-only dataref or a wrong-typed value does not compile.
* **Typed errors.** Fallible calls return `Result<_, XPlaneError>` (`thiserror`); the library never panics on simulator input, and a malformed packet is counted in `stats()` and skipped, never surfaced as an error.

## Prerequisites

* [X-Plane 12](https://www.x-plane.com/) (X-Plane 11 uses the same UDP protocol but is untested).
* X-Plane's UDP port, shown under **Settings → Network** (default `49000`). `RREF`/`DREF` need no data-output rows enabled.
* A multirotor or VTOL model if you are flying one; X-Plane ships no multirotors. The models used by [px4xplane](https://github.com/alireza787b/px4xplane) (Ehang 184, QuadTailsitter, Alia 250) are a good start.

## Requirements

* Rust 1.85 or later (2024 edition)

## Install

```bash
cargo add xplane-bridge
```

## Usage

```rust,no_run
use std::error::Error;
use std::thread;
use std::time::Duration;

use xplane_bridge::{DatarefSink, DatarefSource, Latest, Rate, XPlaneUdpBridge, datarefs};

fn main() -> Result<(), Box<dyn Error>> {
    // X-Plane on this machine, default UDP port.
    let mut bridge = XPlaneUdpBridge::connect("127.0.0.1:49000")?;

    // Ask X-Plane to stream roll and roll rate every simulator frame: it sends
    // at most once per frame, so ask for well above the frame rate. Each handle
    // is typed: `Sub<f32>` here, because both are `f32` datarefs.
    let rate = Rate::hz(250).expect("nonzero rate");
    let phi = bridge.subscribe(&datarefs::position::PHI, rate)?;
    let p = bridge.subscribe(&datarefs::position::P, rate)?;

    // Take control of the engines from X-Plane's own throttle logic.
    // `THROTTLES` is `Dataref<bool, Writable>`, so the value is a `bool`.
    bridge.write(&datarefs::overrides::THROTTLES, true)?;

    // Per-engine throttle datarefs, built once, not per tick.
    // `THRO_USE` is a 16-element writable `f32` array.
    let throttles: Vec<_> = datarefs::engine::THRO_USE.iter().take(4).collect();

    // Holds the last known value of each subscription between frames.
    let mut latest = Latest::default();
    // Fly for 10 seconds.
    for _ in 0..2500 {
        // Drain and merge whatever has arrived; `None` means nothing new since the last poll.
        if let Some(frame) = bridge.poll()? {
            latest.apply(&frame);
        }

        // `latest.get(phi)` is `Option<(f32, Instant)>`.
        if let (Some((roll_deg, _)), Some((roll_rate_dps, _))) = (latest.get(phi), latest.get(p)) {
            // ... run the controller on `roll_deg` and `roll_rate_dps` ...
        }

        // Fire-and-forget per-engine throttle, 0.0 to 1.0.
        for throttle in &throttles {
            bridge.write(throttle, 0.55)?;
        }

        // 250 Hz control loop.
        thread::sleep(Duration::from_millis(4));
    }

    // Hand the engines back to X-Plane. Dropping the bridge only stops the
    // streams; it does not know which overrides you set.
    bridge.write(&datarefs::overrides::THROTTLES, false)?;
    Ok(())
}
```

### Bringing your own socket

`XPlaneUdpBridge` is a few dozen lines over `Protocol`; any other transport is the same loop. Send each returned `Datagram` to X-Plane, feed each received datagram to `handle`, and call `finish` once per drain. In this sketch, `my_socket` stands for your transport:

```rust
# use std::error::Error;
# use std::io;
# use std::net::SocketAddr;
use std::time::Instant;

use xplane_bridge::{Protocol, Rate, datarefs};

# struct MySocket;
# impl MySocket {
#     fn send_to(&self, bytes: &[u8], _: SocketAddr) -> io::Result<usize> { Ok(bytes.len()) }
#     fn try_recv(&mut self) -> Option<&[u8]> { None }
# }
# fn main() -> Result<(), Box<dyn Error>> {
# let mut my_socket = MySocket;
# let xplane_addr: SocketAddr = "127.0.0.1:49000".parse()?;
let mut proto = Protocol::new();

// Calls return the datagram to send. Only `subscribe` can fail (index space exhausted).
// Use `send_to`, not a socket connected to X-Plane: replies come from port 49001,
// not 49000, and a connected socket drops them.
let (phi, rref) = proto.subscribe(&datarefs::position::PHI, Rate::hz(100).expect("nonzero"))?;
my_socket.send_to(rref.as_bytes(), xplane_addr)?;
let dref = proto.write(&datarefs::overrides::THROTTLES, true);
my_socket.send_to(dref.as_bytes(), xplane_addr)?;

// Received bytes go in; anything malformed or for an unknown subscription is counted
// in `stats()` and dropped; frames come out.
while let Some(bytes) = my_socket.try_recv() {
    proto.handle(bytes, Instant::now());
}
if let Some(frame) = proto.finish() {
    let roll_deg: Option<f32> = frame.get(phi);
}

// On shutdown, tell X-Plane to stop streaming.
for dg in proto.close() {
    my_socket.send_to(dg.as_bytes(), xplane_addr)?;
}
# Ok(())
# }
```

### Testing without a simulator

With feature `testing`, `testing::FakeXPlane` implements both traits: queue frames for `poll`, then check what was written. Write code against `impl DatarefSource`/`impl DatarefSink` and it runs unchanged against the fake and the real bridge.

```toml
[dev-dependencies]
xplane-bridge = { version = "0.1", features = ["testing"] }
```

```rust
use std::io::ErrorKind;
use std::time::Instant;

use xplane_bridge::testing::FakeXPlane;
use xplane_bridge::{DatarefSink, DatarefSource, Frame, Rate, datarefs};

let mut fake = FakeXPlane::new();
let phi = fake.subscribe(&datarefs::position::PHI, Rate::hz(100).expect("nonzero"))?;

// One frame, then silence.
fake.push_frame(Frame::empty(Instant::now()).with(phi, 12.5));
assert_eq!(fake.poll()?.and_then(|frame| frame.get(phi)), Some(12.5));
assert_eq!(fake.poll()?, None);

// Writes and commands are recorded.
fake.write(&datarefs::overrides::THROTTLES, true)?;
assert_eq!(fake.last_write(&datarefs::overrides::THROTTLES), Some(true));

// Every call fails until cleared.
fake.fail_with(Some(ErrorKind::ConnectionReset));
assert!(fake.poll().is_err());
# Ok::<(), xplane_bridge::XPlaneError>(())
```

### Timing

X-Plane emits subscribed datarefs at most once per rendered frame, so the state rate is bounded by the simulator's frame rate (typically 60–120 Hz) regardless of the rate requested. Below that, it sends a dataref on the first frame at least `1/rate` after the last send, so delivery rounds down to whole frames: at 120 fps, 100 Hz arrives at 60 Hz. For every frame, request at least twice the frame rate. A controller running faster than that gets `None` from `poll` when nothing new has arrived; one running slower gets every packet since the last poll merged into one frame, newest value per dataref. A frame holds only what arrived, stamped with `received_at` (arrival of its last packet); `Latest` keeps the last known value of each subscription across frames. The simulator is not stepped by the bridge: there is no lockstep mode, so runs are not deterministic.

### Liveness

UDP has no connection, so the bridge reports facts and leaves policy to you:

* **Silence.** `received_at` on a frame (or the `Instant` from `Latest::get`) gives the age of the newest state; pick your own timeout.
* **Restart or late start.** X-Plane forgets every subscription when it restarts, and one that was not running when you subscribed never received them. After a silence, call `resubscribe()`: handles stay valid, and repeating it is harmless. `stats().resubscribes` counts the calls.
* **Lost subscriptions.** X-Plane can drop some `RREF`s when many arrive at once, so a subscription may never stream while the rest do. Until every handle has a value (`Latest::get` is `Some`), call `resubscribe()` now and then.
* **Pause.** X-Plane keeps streaming while paused, with frozen values; subscribe to `datarefs::time::PAUSED` to tell a pause from flight. Losing window focus does not pause X-Plane 12.
* **Shutdown.** Dropping the bridge unsubscribes. Write any overrides you set back to `false` first.

### Datarefs

Named constants for the datarefs a flight controller needs live in `datarefs`, one module per X-Plane path prefix. The table lists X-Plane path names; a constant is the path's last segment in upper case, minus a redundant prefix (`phi` → `PHI`, `ENGN_thro_use` → `THRO_USE`, `override_throttles` → `THROTTLES`). The one exception is the quaternion `q`, which is `QUATERNION` to avoid clashing with pitch rate `Q`. Each constant's type gives its value type and access as `DataRefs.txt` lists them (`Dataref<f32>` read-only, `Dataref<bool, Writable>`, `DatarefArray<f32, Writable, 16>`). Arrays are indexed with `at(i)` (`None` out of range) or iterated with `iter()`.

| Module | Dataref paths |
|---|---|
| `position` | `phi`, `theta`, `psi` (deg); `P`, `Q`, `R` (deg/s, body); `local_x/y/z` (m), `local_vx/vy/vz` (m/s), `local_ax/ay/az` (m/s²), all OpenGL local frame; `latitude`, `longitude` (deg), `elevation` (m MSL); `lat_ref`, `lon_ref` (deg, origin of the local frame); `q` (quaternion `[w, x, y, z]`, north-east-down to body) |
| `engine` | `ENGN_thro_use[16]` (0–1 per engine) |
| `joystick` | `yoke_roll_ratio`, `yoke_pitch_ratio`, `yoke_heading_ratio` (−1 to 1) |
| `overrides` | `override_throttles`, `override_joystick`, `override_control_surfaces` |
| `time` | `paused`; `total_running_time_sec` (s) |
| `wing` | `aileron1/2_deg[48]`, `elevator1/2_deg[48]`, `rudder1/2_deg[48]` (deg, per `.acf` wing) |

Any other dataref can be used by path with `Dataref::<f32>::new("sim/...")?` (or `Dataref::<i32, Writable>::new`, etc.), built once at startup; the path is checked there, so writes never fail on it. Commands work the same way: constants in `commands` (`PAUSE_TOGGLE`, `RESET_FLIGHT`), or `Command::new("sim/...")?`. The full list ships with X-Plane in `Resources/plugins/DataRefs.txt`.

## Testing

```bash
cargo test
```

The codec and `Protocol` are tested without sockets (bytes in, datagrams out); the UDP driver has a few loopback tests against a fake X-Plane. Tests that need a running X-Plane are `#[ignore]`d; run them with `cargo test -- --ignored` while X-Plane is up (at `XPLANE_ADDR`, default `127.0.0.1:49000`). Set `XPLANE_DIR` to the X-Plane 12 install directory to also check the constants against its `DataRefs.txt` and `Commands.txt`.

For a manual check against a running X-Plane, run `cargo run --example probe`; see [`examples/probe.rs`](https://github.com/wboayue/xplane-bridge/blob/main/examples/probe.rs) for its options.

## License

MIT, see [LICENSE](https://github.com/wboayue/xplane-bridge/blob/main/LICENSE).
