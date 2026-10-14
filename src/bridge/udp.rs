//! The `std::net::UdpSocket` driver: moves `Protocol`'s datagrams, nothing more.

use std::io::{self, ErrorKind};
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr, UdpSocket};
use std::time::Instant;

use log::{debug, info, warn};

use crate::{
    Command, Datagram, Dataref, DatarefSink, DatarefSource, Frame, Protocol, Rate, Stats, Sub,
    Value, Writable, XPlaneError,
};

/// Receive buffer size: the largest UDP payload, so no datagram is ever
/// truncated (on Windows truncation is a `recv_from` error, not a short read).
const RECV_LEN: usize = 65_535;

/// X-Plane over UDP: a [`Protocol`] driven through a non-blocking
/// `std::net::UdpSocket`.
///
/// Dropping it unsubscribes every active subscription. It does not release
/// overrides; write those back first.
///
/// ```no_run
/// use std::net::SocketAddr;
/// use xplane_bridge::{DatarefSource, Rate, XPlaneUdpBridge, datarefs};
///
/// let addr: SocketAddr = "127.0.0.1:49000".parse()?;
/// let mut bridge = XPlaneUdpBridge::connect(addr)?;
/// let phi = bridge.subscribe(&datarefs::position::PHI, Rate::hz(100).expect("nonzero"))?;
/// if let Some(frame) = bridge.poll()? {
///     let roll_deg: Option<f32> = frame.get(phi);
/// }
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug)]
pub struct XPlaneUdpBridge {
    proto: Protocol,
    sock: UdpSocket,
    addr: SocketAddr,
    buf: Box<[u8]>,
}

impl XPlaneUdpBridge {
    /// A bridge to X-Plane at `addr` (its UDP port, default 49000).
    ///
    /// Binds an ephemeral port on the unspecified address of `addr`'s family, so
    /// replies from another host arrive. The socket is never `connect()`ed:
    /// X-Plane replies from a different port (49001), which a connected socket
    /// would drop. No handshake exists, so success means only that the socket is
    /// ready.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] if the socket cannot be bound or made non-blocking.
    pub fn connect(addr: SocketAddr) -> Result<Self, XPlaneError> {
        let local: SocketAddr = match addr {
            SocketAddr::V4(_) => (Ipv4Addr::UNSPECIFIED, 0).into(),
            SocketAddr::V6(_) => (Ipv6Addr::UNSPECIFIED, 0).into(),
        };
        let sock = UdpSocket::bind(local)?;
        sock.set_nonblocking(true)?;
        info!("bound {} for X-Plane at {addr}", sock.local_addr()?);
        Ok(Self {
            proto: Protocol::new(),
            sock,
            addr,
            buf: vec![0; RECV_LEN].into_boxed_slice(),
        })
    }

    fn send(&self, datagram: &Datagram) -> io::Result<()> {
        self.sock.send_to(datagram.as_bytes(), self.addr)?;
        Ok(())
    }
}

impl DatarefSource for XPlaneUdpBridge {
    fn subscribe<T: Value, A>(
        &mut self,
        dataref: &Dataref<T, A>,
        rate: Rate,
    ) -> Result<Sub<T>, XPlaneError> {
        let (sub, datagram) = self.proto.subscribe(dataref, rate)?;
        self.send(&datagram)?;
        Ok(sub)
    }

    fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Result<(), XPlaneError> {
        if let Some(datagram) = self.proto.unsubscribe(sub) {
            self.send(&datagram)?;
        }
        Ok(())
    }

    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError> {
        loop {
            match self.sock.recv_from(&mut self.buf) {
                Ok((len, _)) => {
                    let bytes = self.buf.get(..len).unwrap_or_default();
                    self.proto.handle(bytes, Instant::now());
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return Ok(self.proto.finish()),
                // Windows reports an earlier `send_to` that hit a closed port (X-Plane
                // not running) here; replies may be queued behind it.
                Err(e) if e.kind() == ErrorKind::ConnectionReset => {
                    debug!("skipping connection reset from an earlier send: {e}");
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn resubscribe(&mut self) -> Result<(), XPlaneError> {
        for datagram in self.proto.resubscribe() {
            self.send(&datagram)?;
        }
        Ok(())
    }

    fn stats(&self) -> Stats {
        self.proto.stats()
    }
}

impl DatarefSink for XPlaneUdpBridge {
    fn write<T: Value>(
        &mut self,
        dataref: &Dataref<T, Writable>,
        value: T,
    ) -> Result<(), XPlaneError> {
        self.send(&self.proto.write(dataref, value))?;
        Ok(())
    }

    fn command(&mut self, command: &Command) -> Result<(), XPlaneError> {
        self.send(&self.proto.command(command))?;
        Ok(())
    }
}

impl Drop for XPlaneUdpBridge {
    fn drop(&mut self) {
        let datagrams = self.proto.close();
        info!(
            "unsubscribing {} from X-Plane at {}",
            datagrams.len(),
            self.addr
        );
        for datagram in &datagrams {
            if let Err(e) = self.send(datagram) {
                warn!("unsubscribe on drop failed: {e}");
            }
        }
    }
}

// A child of `udp`, so the tests can read the socket; the file sits at
// `bridge/tests.rs`, beside the traits it also exercises.
#[cfg(test)]
#[path = "tests.rs"]
mod tests;
