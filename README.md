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

* **Synchronous, `std` only.** One `std::net::UdpSocket`, no async runtime. Reads are non-blocking so a fixed-rate control loop never stalls on the simulator.
* **Raw simulator units.** Values are returned as X-Plane reports them (degrees, degrees per second, OpenGL local frame: x east, y up, z south). Frame and unit conversion belong to the caller.
* **Trait-based.** `XPlaneBridge` is a trait; `XPlaneUdpBridge` is the real implementation, so controllers can be tested against a mock without a simulator.
* **Pure codec.** Packet encoding and decoding are plain functions over byte slices, unit-tested without a socket.
* **Typed errors.** Fallible calls return `Result<_, XPlaneError>` (`thiserror`); the library never panics on simulator input.

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

use xplane_bridge::{XPlaneBridge, XPlaneUdpBridge, datarefs};

fn main() -> Result<(), Box<dyn Error>> {
    // X-Plane on this machine, default UDP port.
    let addr: SocketAddr = "127.0.0.1:49000".parse()?;
    let mut bridge = XPlaneUdpBridge::connect(addr)?;

    // Ask X-Plane to stream attitude and body rates at up to 100 Hz
    // (it sends at most once per simulator frame).
    bridge.subscribe(&[
        datarefs::position::PHI,
        datarefs::position::THETA,
        datarefs::position::PSI,
        datarefs::position::P,
        datarefs::position::Q,
        datarefs::position::R,
    ], 100)?;

    // Take control of the engines from X-Plane's own throttle logic.
    bridge.write(datarefs::overrides::THROTTLES, 1.0)?;

    let mut roll_deg = 0.0;
    loop {
        // Drain and merge whatever has arrived; `None` means nothing new since the last poll.
        if let Some(frame) = bridge.poll()? {
            if let Some(phi) = frame.get(&datarefs::position::PHI) {
                roll_deg = phi;
            }
        }

        // ... run the controller on `roll_deg` ...

        // Fire-and-forget per-engine throttle, 0.0 to 1.0 (array element i).
        for engine in 0..4 {
            bridge.write(datarefs::engine::THRO_USE.at(engine), 0.55)?;
        }

        // 250 Hz control loop.
        thread::sleep(Duration::from_millis(4));
    }
}
```

### Timing

X-Plane emits subscribed datarefs once per rendered frame, so the state rate is bounded by the simulator's frame rate (typically 60–120 Hz) regardless of the rate requested. A controller running faster than that gets `None` from `poll` when nothing new has arrived; one running slower gets every packet since the last poll merged into one frame, newest value per dataref. The simulator is not stepped by the bridge: there is no lockstep mode, so runs are not deterministic.

### Datarefs

Named constants for the datarefs a flight controller needs live in `datarefs`, one module per X-Plane path prefix. The table lists X-Plane path names; constants are the upper-case form (`phi` → `PHI`, `ENGN_thro_use` → `THRO_USE`), except the quaternion `q`, which is `QUATERNION` to avoid clashing with pitch rate `Q`.

| Module | Dataref paths |
|---|---|
| `position` | `phi`, `theta`, `psi` (deg); `P`, `Q`, `R` (deg/s, body); `local_x/y/z` (m), `local_vx/vy/vz` (m/s), `local_ax/ay/az` (m/s²), all OpenGL local frame; `latitude`, `longitude` (deg), `elevation` (m MSL); `q` (quaternion) |
| `engine` | `ENGN_thro_use[8]` (0–1 per engine) |
| `joystick` | `yoke_roll_ratio`, `yoke_pitch_ratio`, `yoke_heading_ratio` (−1 to 1) |
| `overrides` | `override_throttles`, `override_joystick`, `override_control_surfaces` |

Any other dataref can be used by path with `Dataref::new("sim/...")`. The full list ships with X-Plane in `Resources/plugins/DataRefs.txt`.

## Testing

```bash
cargo test
```

Codec and bridge logic are covered without a simulator. Tests that need a running X-Plane are `#[ignore]`d; run them with `cargo test -- --ignored` while X-Plane is up.

## License

MIT, see [LICENSE](LICENSE).
