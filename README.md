# xplane-bridge

*A Rust library to interface external flight controllers with the X-Plane simulator.*

[![License:MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

> **Status:** early development, not yet published. The API below is the target shape and may change before the first release.

## Overview

[X-Plane](https://www.x-plane.com/) is a flight simulator with a blade-element aerodynamic model that runs natively on macOS (Apple Silicon), Windows, and Linux. It exposes its state and controls as *datarefs* that any process can read and write over UDP, with no plugin required.

**xplane-bridge** wraps that UDP protocol so an external flight controller can:

* Subscribe to datarefs (`RREF`) and receive them every simulator frame: attitude, body rates, position, velocity, acceleration.
* Write datarefs (`DREF`) to drive the aircraft: per-engine throttle, control surfaces, and the override flags that hand control to the bridge.
* Send commands (`CMND`) such as pause or reset.

It is the X-Plane counterpart of [realflight-bridge](https://github.com/wboayue/realflight-bridge) and is used by the [Stable Flight](https://github.com/wboayue/stable-flight) `stable-flight-sitl-xplane` backend, but it has no dependency on either.

## Design

* **Sans-I/O.** The whole protocol lives in `Protocol`, which never touches a socket: calls return `Datagram`s to send, received bytes go in through `handle`. `XPlaneUdpBridge` is a thin driver that moves those datagrams over a `std::net::UdpSocket`. Bring your own socket (`socket2`, `mio`, an embedded stack) by driving `Protocol` yourself.
* **Sync by default, `std` only.** The default build has no async runtime; reads are non-blocking so a fixed-rate control loop never stalls on the simulator. An async driver over `tokio::net::UdpSocket` (same `Protocol`) is planned behind the `rt-tokio` feature once there is a consumer for it.
* **Raw simulator units.** Values are returned as X-Plane reports them (degrees, degrees per second, OpenGL local frame: x east, y up, z south). Frame and unit conversion belong to the caller.
* **Small traits.** Consumers depend on `DatarefSource` (subscribe, poll) and `DatarefSink` (write, command), not on the driver; controllers can be tested against fakes without a simulator.
* **Handles, not lookups.** `subscribe` returns a `Sub` handle per dataref; reading a frame is `frame.get(sub)`.
* **Typed errors.** Fallible calls return `Result<_, XPlaneError>` (`thiserror`); the library never panics on simulator input, and a malformed packet is counted in `stats()` and skipped, never surfaced as an error.

## Prerequisites

* [X-Plane 12](https://www.x-plane.com/) (X-Plane 11 uses the same UDP protocol but is untested).
* X-Plane's UDP port, shown under **Settings → Network** (default `49000`). `RREF`/`DREF` need no data-output rows enabled.
* A multirotor or VTOL model if you are flying one; X-Plane ships no multirotors. The models used by [px4xplane](https://github.com/alireza787b/px4xplane) (Ehang 184, QuadTailsitter, Alia 250) are a good start.
* Disable **pause on loss of focus** in the settings if the controller runs on the same machine and you alt-tab away.

## Requirements

* Rust 1.85 or later (2024 edition)

## Install

Once published:

```bash
cargo add xplane-bridge
```

Until then, use a git dependency:

```toml
[dependencies]
xplane-bridge = { git = "https://github.com/wboayue/xplane-bridge" }
```

## Usage

```rust
use std::error::Error;
use std::net::SocketAddr;
use std::thread;
use std::time::Duration;

use xplane_bridge::{DatarefSink, DatarefSource, Latest, XPlaneUdpBridge, datarefs};

fn main() -> Result<(), Box<dyn Error>> {
    // X-Plane on this machine, default UDP port.
    let addr: SocketAddr = "127.0.0.1:49000".parse()?;
    let mut bridge = XPlaneUdpBridge::connect(addr)?;

    // Ask X-Plane to stream attitude and body rates at up to 100 Hz
    // (it sends at most once per simulator frame). Handles come back in order.
    let subs = bridge.subscribe(&[
        datarefs::position::PHI,
        datarefs::position::THETA,
        datarefs::position::PSI,
        datarefs::position::P,
        datarefs::position::Q,
        datarefs::position::R,
    ], 100)?;
    let phi = subs[0];

    // Take control of the engines from X-Plane's own throttle logic.
    bridge.write(&datarefs::overrides::THROTTLES, 1.0)?;

    // Build per-engine throttle datarefs once, not per tick.
    let throttles: Vec<_> = (0..4).map(|i| datarefs::engine::THRO_USE.at(i)).collect();

    // Holds the last known value of each subscription between frames.
    let mut latest = Latest::default();
    loop {
        // Drain and merge whatever has arrived; `None` means nothing new since the last poll.
        if let Some(frame) = bridge.poll()? {
            latest.apply(&frame);
        }

        if let Some((roll_deg, _received_at)) = latest.get(phi) {
            // ... run the controller on `roll_deg` ...
        }

        // Fire-and-forget per-engine throttle, 0.0 to 1.0.
        for throttle in &throttles {
            bridge.write(throttle, 0.55)?;
        }

        // 250 Hz control loop.
        thread::sleep(Duration::from_millis(4));
    }
}
```

### Bringing your own socket

`XPlaneUdpBridge` is a few dozen lines over `Protocol`; any other transport is the same loop. Send each returned `Datagram` to X-Plane, feed each received datagram to `handle`, and call `finish` once per drain:

```rust
use std::time::Instant;

use xplane_bridge::{Protocol, datarefs};

let mut proto = Protocol::new();

// Calls return the datagram to send.
let (phi, rref) = proto.subscribe(&datarefs::position::PHI, 100)?;
my_socket.send(rref.as_bytes())?;
let dref = proto.write(&datarefs::overrides::THROTTLES, 1.0)?;
my_socket.send(dref.as_bytes())?;

// Received bytes go in; records for unknown subscriptions are dropped and counted; frames come out.
while let Some(bytes) = my_socket.try_recv() {
    proto.handle(bytes, Instant::now());
}
if let Some(frame) = proto.finish() {
    let roll_deg = frame.get(phi);
}

// On shutdown, tell X-Plane to stop streaming.
for dg in proto.close() {
    my_socket.send(dg.as_bytes())?;
}
```

### Timing

X-Plane emits subscribed datarefs once per rendered frame, so the state rate is bounded by the simulator's frame rate (typically 60–120 Hz) regardless of the rate requested. A controller running faster than that gets `None` from `poll` when nothing new has arrived; one running slower gets every packet since the last poll merged into one frame, newest value per dataref. A frame holds only what arrived, stamped with `received_at` (arrival of its last packet); `Latest` keeps the last known value of each subscription across frames. The simulator is not stepped by the bridge: there is no lockstep mode, so runs are not deterministic.

### Datarefs

Named constants for the datarefs a flight controller needs live in `datarefs`, one module per X-Plane path prefix. The table lists X-Plane path names; constants are the upper-case form (`phi` → `PHI`, `ENGN_thro_use` → `THRO_USE`), except the quaternion `q`, which is `QUATERNION` to avoid clashing with pitch rate `Q`.

| Module | Dataref paths |
|---|---|
| `position` | `phi`, `theta`, `psi` (deg); `P`, `Q`, `R` (deg/s, body); `local_x/y/z` (m), `local_vx/vy/vz` (m/s), `local_ax/ay/az` (m/s²), all OpenGL local frame; `latitude`, `longitude` (deg), `elevation` (m MSL); `q` (quaternion) |
| `engine` | `ENGN_thro_use[8]` (0–1 per engine) |
| `joystick` | `yoke_roll_ratio`, `yoke_pitch_ratio`, `yoke_heading_ratio` (−1 to 1) |
| `overrides` | `override_throttles`, `override_joystick`, `override_control_surfaces` |

Any other dataref can be used by path with `Dataref::new("sim/...")`, built once at startup. Commands work the same way: constants in `commands`, or `Command::new("sim/...")`. The full list ships with X-Plane in `Resources/plugins/DataRefs.txt`.

## Testing

```bash
cargo test
```

The codec and `Protocol` are tested without sockets (bytes in, datagrams out); the UDP driver has a few loopback tests against a fake X-Plane. Tests that need a running X-Plane are `#[ignore]`d; run them with `cargo test -- --ignored` while X-Plane is up.

## License

MIT, see [LICENSE](LICENSE).
