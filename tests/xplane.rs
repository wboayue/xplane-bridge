//! Against a running X-Plane at `$XPLANE_ADDR` (default `127.0.0.1:49000`):
//! `cargo test --test xplane -- --ignored`.

use std::error::Error;
use std::net::SocketAddr;
use std::thread;
use std::time::{Duration, Instant};

use xplane_bridge::datarefs::{overrides, position};
use xplane_bridge::{DatarefSink, DatarefSource, Frame, Rate, XPlaneError, XPlaneUdpBridge};

const TIMEOUT: Duration = Duration::from_secs(2);
const RATE: Rate = match Rate::hz(100) {
    Some(rate) => rate,
    None => panic!("zero rate"),
};

fn connect() -> Result<XPlaneUdpBridge, Box<dyn Error>> {
    let addr: SocketAddr = std::env::var("XPLANE_ADDR")
        .unwrap_or_else(|_| "127.0.0.1:49000".into())
        .parse()?;
    Ok(XPlaneUdpBridge::connect(addr)?)
}

/// Polls until a frame satisfies `pred`; `false` after [`TIMEOUT`].
fn poll_until(
    bridge: &mut impl DatarefSource,
    pred: impl Fn(&Frame) -> bool,
) -> Result<bool, XPlaneError> {
    let start = Instant::now();
    while start.elapsed() < TIMEOUT {
        if bridge.poll()?.is_some_and(|frame| pred(&frame)) {
            return Ok(true);
        }
        thread::sleep(Duration::from_millis(4));
    }
    Ok(false)
}

#[test]
#[ignore = "needs X-Plane running at $XPLANE_ADDR"]
fn streams_subscribed_datarefs() -> Result<(), Box<dyn Error>> {
    let mut bridge = connect()?;
    let phi = bridge.subscribe(&position::PHI, RATE)?;
    assert!(poll_until(&mut bridge, |frame| frame.get(phi).is_some())?);
    assert!(bridge.stats().packets > 0);
    Ok(())
}

#[test]
#[ignore = "needs X-Plane running at $XPLANE_ADDR"]
fn write_reads_back() -> Result<(), Box<dyn Error>> {
    let mut bridge = connect()?;
    let throttles = bridge.subscribe(&overrides::THROTTLES, RATE)?;
    // Both writes happen before any assert, so a failure never leaves the override set.
    let mut seen = Vec::new();
    for value in [true, false] {
        bridge.write(&overrides::THROTTLES, value)?;
        seen.push(poll_until(&mut bridge, |frame| {
            frame.get(throttles) == Some(value)
        })?);
    }
    assert_eq!(
        seen,
        [true, true],
        "override_throttles read back [true, false]"
    );
    Ok(())
}
