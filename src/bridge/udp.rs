//! The `std::net::UdpSocket` driver: moves `Protocol`'s datagrams, nothing more.

use std::io::{self, ErrorKind};
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr, ToSocketAddrs, UdpSocket};
use std::time::Instant;

use log::{debug, info, warn};
use socket2::{Domain, Socket, Type};

use crate::{
    Command, Datagram, Dataref, DatarefSink, DatarefSource, Frame, Protocol, Rate, Stats, Sub,
    Value, Writable, XPlaneError,
};

/// Receive buffer size: the largest UDP payload, so no datagram is ever
/// truncated (on Windows truncation is a `recv_from` error, not a short read).
const RECV_LEN: usize = 65_535;

/// Send buffer size. macOS defaults to 9216 bytes, which over Wi-Fi fills
/// after 4 back-to-back `RREF`s; this holds a subscribe burst of over 100.
const SEND_BUFFER: usize = 65_536;

/// X-Plane over UDP: a [`Protocol`] driven through a non-blocking
/// `std::net::UdpSocket`.
///
/// A send that finds the socket's send buffer full is dropped, not waited on,
/// and counted in [`Stats::dropped_sends`].
///
/// Dropping it unsubscribes every active subscription. It does not release
/// overrides; write those back first.
///
/// ```no_run
/// use xplane_bridge::{DatarefSource, Rate, XPlaneUdpBridge, datarefs};
///
/// let mut bridge = XPlaneUdpBridge::connect("127.0.0.1:49000")?;
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

/// The first IPv4 address `addr` resolves to, else its first address.
fn resolve(addr: impl ToSocketAddrs) -> io::Result<SocketAddr> {
    let addrs: Vec<SocketAddr> = addr.to_socket_addrs()?.collect();
    addrs
        .iter()
        .find(|a| a.is_ipv4())
        .or(addrs.first())
        .copied()
        .ok_or_else(|| io::Error::new(ErrorKind::InvalidInput, "address resolved to nothing"))
}

impl XPlaneUdpBridge {
    /// A bridge to X-Plane at `addr` (its UDP port, default 49000).
    ///
    /// `addr` is anything [`ToSocketAddrs`] takes, such as `"oya.local:49000"`;
    /// a hostname is resolved once, here, to its first IPv4 address (X-Plane
    /// listens on IPv4; mDNS names list IPv6 first), else its first address. Binds an ephemeral port on the unspecified address of `addr`'s family, so
    /// replies from another host arrive. The socket is never `connect()`ed:
    /// X-Plane replies from a different port (49001), which a connected socket
    /// would drop. No handshake exists, so success means only that the socket is
    /// ready.
    ///
    /// # Errors
    ///
    /// [`XPlaneError::Io`] if `addr` does not resolve, or the socket cannot be
    /// created, sized, bound, or made non-blocking.
    pub fn connect(addr: impl ToSocketAddrs) -> Result<Self, XPlaneError> {
        let addr = resolve(addr)?;
        let local: SocketAddr = match addr {
            SocketAddr::V4(_) => (Ipv4Addr::UNSPECIFIED, 0).into(),
            SocketAddr::V6(_) => (Ipv6Addr::UNSPECIFIED, 0).into(),
        };
        let sock = Socket::new(Domain::for_address(local), Type::DGRAM, None)?;
        sock.set_send_buffer_size(SEND_BUFFER)?;
        sock.bind(&local.into())?;
        sock.set_nonblocking(true)?;
        let sock = UdpSocket::from(sock);
        if let Ok(local) = sock.local_addr() {
            info!("bound {local} for X-Plane at {addr}");
        }
        Ok(Self {
            proto: Protocol::new(),
            sock,
            addr,
            buf: vec![0; RECV_LEN].into_boxed_slice(),
        })
    }

    /// Sends `datagram`, or drops and counts it if the send buffer is full:
    /// waiting would block the caller.
    fn send(&mut self, datagram: &Datagram) -> io::Result<()> {
        match self.sock.send_to(datagram.as_bytes(), self.addr) {
            Ok(_) => Ok(()),
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                debug!(
                    "send buffer full; dropped {}-byte datagram",
                    datagram.as_bytes().len()
                );
                self.proto.dropped_send();
                Ok(())
            }
            Err(e) => Err(e),
        }
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
        let datagram = self.proto.write(dataref, value);
        self.send(&datagram)?;
        Ok(())
    }

    fn command(&mut self, command: &Command) -> Result<(), XPlaneError> {
        let datagram = self.proto.command(command);
        self.send(&datagram)?;
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
