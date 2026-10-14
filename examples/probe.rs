//! Prints attitude, rates, local position, pause state, and sim time from a
//! running X-Plane at 10 Hz; the manual check against the simulator.
//!
//! ```text
//! cargo run --example probe -- [--host 127.0.0.1:49000] [--spin]
//! ```
//!
//! `--spin` overrides the throttles and holds every engine at [`SPIN_THROTTLE`];
//! Ctrl-C releases the override before exiting.

use std::error::Error;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use xplane_bridge::datarefs::{engine, overrides, position, time};
use xplane_bridge::{
    DatarefSink, DatarefSource, Latest, Rate, Stats, Sub, XPlaneError, XPlaneUdpBridge,
};

/// Throttle for `--spin`: enough to see the motors turn, not to lift off.
const SPIN_THROTTLE: f32 = 0.2;
const RATE: Rate = match Rate::hz(100) {
    Some(rate) => rate,
    None => panic!("zero rate"),
};
const TICK: Duration = Duration::from_millis(4);
/// Paces both the printed line and the `--spin` throttle refresh.
const REPORT_EVERY: Duration = Duration::from_millis(100);
/// Silence after which the probe resubscribes, for a late-started X-Plane.
const SILENCE: Duration = Duration::from_secs(1);

struct Args {
    host: SocketAddr,
    spin: bool,
}

fn parse_args() -> Result<Args, Box<dyn Error>> {
    let mut args = Args {
        host: "127.0.0.1:49000".parse()?,
        spin: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--host" => args.host = it.next().ok_or("--host needs an address")?.parse()?,
            "--spin" => args.spin = true,
            _ => {
                return Err(
                    format!("unknown argument {arg}; usage: probe [--host ADDR] [--spin]").into(),
                );
            }
        }
    }
    Ok(args)
}

struct Subs {
    phi: Sub<f32>,
    theta: Sub<f32>,
    psi: Sub<f32>,
    p: Sub<f32>,
    q: Sub<f32>,
    r: Sub<f32>,
    x: Sub<f32>,
    y: Sub<f32>,
    z: Sub<f32>,
    quaternion: Vec<Sub<f32>>,
    paused: Sub<bool>,
    sim_time: Sub<f32>,
}

impl Subs {
    fn new(bridge: &mut impl DatarefSource) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            phi: bridge.subscribe(&position::PHI, RATE)?,
            theta: bridge.subscribe(&position::THETA, RATE)?,
            psi: bridge.subscribe(&position::PSI, RATE)?,
            p: bridge.subscribe(&position::P, RATE)?,
            q: bridge.subscribe(&position::Q, RATE)?,
            r: bridge.subscribe(&position::R, RATE)?,
            x: bridge.subscribe(&position::LOCAL_X, RATE)?,
            y: bridge.subscribe(&position::LOCAL_Y, RATE)?,
            z: bridge.subscribe(&position::LOCAL_Z, RATE)?,
            quaternion: position::QUATERNION
                .iter()
                .map(|element| bridge.subscribe(&element, RATE))
                .collect::<Result<_, _>>()?,
            paused: bridge.subscribe(&time::PAUSED, RATE)?,
            sim_time: bridge.subscribe(&time::TOTAL_RUNNING_TIME_SEC, RATE)?,
        })
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = parse_args()?;
    let stop = Arc::new(AtomicBool::new(false));
    let handler_stop = Arc::clone(&stop);
    ctrlc::set_handler(move || handler_stop.store(true, Ordering::Relaxed))?;

    let mut bridge = XPlaneUdpBridge::connect(args.host)?;
    println!("probing X-Plane at {}; Ctrl-C to stop", args.host);

    if args.spin {
        bridge.write(&overrides::THROTTLES, true)?;
    }
    let result = run(&mut bridge, &args, &stop);
    // Release even after an error; the override outlives this process otherwise.
    let released = if args.spin {
        release(&mut bridge)
    } else {
        Ok(())
    };
    result?;
    Ok(released?)
}

/// Zeroes the throttles and hands them back to X-Plane. Both writes are
/// attempted; the override flag is the one that matters.
fn release(bridge: &mut impl DatarefSink) -> Result<(), XPlaneError> {
    let zeroed = set_throttles(bridge, 0.0);
    bridge.write(&overrides::THROTTLES, false)?;
    println!("throttle override released");
    zeroed
}

fn run(
    bridge: &mut (impl DatarefSource + DatarefSink),
    args: &Args,
    stop: &AtomicBool,
) -> Result<(), Box<dyn Error>> {
    let subs = Subs::new(bridge)?;

    let mut latest = Latest::default();
    let mut last_frame: Option<Instant> = None;
    let mut last_print = Instant::now();
    let mut last_resubscribe = Instant::now();
    while !stop.load(Ordering::Relaxed) {
        if let Some(frame) = bridge.poll()? {
            latest.apply(&frame);
            last_frame = Some(frame.received_at());
        }

        let now = Instant::now();
        let silent = last_frame.is_none_or(|t| now - t > SILENCE);
        if silent && now - last_resubscribe > SILENCE {
            bridge.resubscribe()?;
            last_resubscribe = now;
            println!("no data for {SILENCE:?}; resubscribed");
        }

        if now - last_print >= REPORT_EVERY {
            last_print = now;
            if args.spin {
                // Rewritten each print so a lost datagram heals.
                set_throttles(bridge, SPIN_THROTTLE)?;
            }
            if !silent {
                print_line(&latest, &subs, last_frame.map(|t| now - t), bridge.stats());
            }
        }
        thread::sleep(TICK);
    }
    Ok(())
}

/// Sets every engine's throttle, 0 to 1.
fn set_throttles(bridge: &mut impl DatarefSink, value: f32) -> Result<(), XPlaneError> {
    for throttle in engine::THRO_USE.iter() {
        bridge.write(&throttle, value)?;
    }
    Ok(())
}

fn print_line(latest: &Latest, subs: &Subs, age: Option<Duration>, stats: Stats) {
    let f = |sub: Sub<f32>| latest.get(sub).map_or(f32::NAN, |(v, _)| v);
    let q: Vec<String> = subs
        .quaternion
        .iter()
        .map(|&s| format!("{:+.3}", f(s)))
        .collect();
    println!(
        "t {:8.2}s paused {:?} | phi {:+7.2} theta {:+7.2} psi {:6.2} deg | \
         p {:+7.2} q {:+7.2} r {:+7.2} deg/s | x {:+9.2} y {:+8.2} z {:+9.2} m | \
         quat [{}] | age {:3}ms pkts {} bad {}",
        f(subs.sim_time),
        latest.get(subs.paused).map(|(v, _)| v),
        f(subs.phi),
        f(subs.theta),
        f(subs.psi),
        f(subs.p),
        f(subs.q),
        f(subs.r),
        f(subs.x),
        f(subs.y),
        f(subs.z),
        q.join(" "),
        age.map_or(0, |a| a.as_millis()),
        stats.packets,
        stats.malformed,
    );
}
