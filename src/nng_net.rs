//! NNG over `embassy-net`: the board's telemetry on the wire.
//!
//! Two `nng-core` sockets share the Ethernet stack:
//!
//! * **REP0 on :5555** — poll model. Any request is answered with one
//!   [`telemetry_wire::Frame`] taken at the moment of the reply.
//! * **PUB0 on :5556** — stream model. The same frame is published at
//!   [`PUBLISH_HZ`], for subscribers filtering on [`telemetry_wire::MAGIC`].
//!
//! On Embassy the pipe count is fixed: one listener task owns one `TcpSocket`
//! and serves one peer at a time, so `SocketConfig::max_pipes` equals the
//! number of listener tasks spawned here. A peer beyond that is accepted by
//! TCP and then closed at the SP level.

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Instant, Timer};
use nng_core::Message;
use nng_core::sock::driver::serve_listener;
use nng_core::sock::embassy::{EmbassyClock, EmbassyTcpListener};
use nng_core::sock::{Socket, SocketConfig, proto};
use static_cell::StaticCell;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::degree;
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

use crate::gnss::FixQuality;
use crate::telemetry::{self, Telemetry};

/// Port the REP0 socket listens on.
pub const REP_PORT: u16 = 5555;
/// Port the PUB0 socket listens on.
pub const PUB_PORT: u16 = 5556;

/// Concurrent REP peers (one `TcpSocket` each).
const REP_PIPES: usize = 2;
/// Request handlers sharing the REP socket.
const REP_WORKERS: usize = 2;
/// Concurrent PUB subscribers (one `TcpSocket` each).
const PUB_PIPES: usize = 2;

/// Telemetry frames published per second.
const PUBLISH_HZ: u64 = 5;
/// Readings older than this are flagged with the `*_STALE` bits. Matches the
/// threshold the console summary uses.
const STALE_AFTER: Duration = Duration::from_secs(3);

/// Start both sockets. Call once, after the network stack has an address.
pub fn spawn(spawner: &Spawner, stack: Stack<'static>) {
    static REP: StaticCell<Socket<proto::Rep0>> = StaticCell::new();
    let rep: &'static Socket<proto::Rep0> = REP.init(Socket::new(SocketConfig {
        max_pipes: REP_PIPES,
        ..SocketConfig::default()
    }));
    for _ in 0..REP_PIPES {
        spawner.spawn(unwrap!(rep_listener_task(rep, stack)));
    }
    for id in 0..REP_WORKERS as u8 {
        spawner.spawn(unwrap!(rep_worker_task(rep, id)));
    }

    static PUB: StaticCell<Socket<proto::Pub0>> = StaticCell::new();
    let publisher: &'static Socket<proto::Pub0> = PUB.init(Socket::new(SocketConfig {
        max_pipes: PUB_PIPES,
        ..SocketConfig::default()
    }));
    for _ in 0..PUB_PIPES {
        spawner.spawn(unwrap!(pub_listener_task(publisher, stack)));
    }
    spawner.spawn(unwrap!(publish_task(publisher)));
}

/// Build a wire frame from the current sensor state.
fn frame() -> telemetry_wire::Frame {
    let t: Telemetry = telemetry::snapshot();
    let now = Instant::now();
    let stale = |at: Option<Instant>| at.is_some_and(|at| now - at > STALE_AFTER);

    let mut f = telemetry_wire::Frame {
        uptime_ms: now.as_millis(),
        ..Default::default()
    };

    if let Some(g) = t.gga {
        f.satellites = g.satellites;
        f.fix = match g.fix {
            FixQuality::None => 0,
            FixQuality::Gnss => 1,
            FixQuality::Dgnss => 2,
            FixQuality::Other(v) => v,
        };
        if g.fix != FixQuality::None {
            f.flags |= telemetry_wire::FLAG_GNSS_FIX;
            f.latitude_deg = g.latitude.get::<degree>();
            f.longitude_deg = g.longitude.get::<degree>();
            f.altitude_m = g.altitude.get::<meter>();
        }
    }
    if stale(t.gnss_updated) {
        f.flags |= telemetry_wire::FLAG_GNSS_STALE;
    }

    if let Some(r) = t.rmc.filter(|r| r.valid) {
        f.flags |= telemetry_wire::FLAG_GNSS_COURSE;
        f.speed_mps = r.speed.get::<meter_per_second>();
        f.course_deg = r.course.get::<degree>();
    }

    if let Some(o) = t.orientation {
        let e = o.euler();
        f.flags |= telemetry_wire::FLAG_IMU;
        f.imu_status = o.status;
        f.yaw_deg = e.yaw.get::<degree>();
        f.pitch_deg = e.pitch.get::<degree>();
        f.roll_deg = e.roll.get::<degree>();
    }
    if stale(t.imu_updated) {
        f.flags |= telemetry_wire::FLAG_IMU_STALE;
    }

    if let Some(a) = t.acceleration {
        f.accel_mss = [
            a.x.get::<meter_per_second_squared>(),
            a.y.get::<meter_per_second_squared>(),
            a.z.get::<meter_per_second_squared>(),
        ];
    }

    f
}

/// A frame as an NNG message. The magic leads the body, so it doubles as the
/// SUB topic.
fn frame_message() -> Message {
    let mut msg = Message::new();
    msg.push_back(&frame().encode());
    msg
}

/// One REP pipe: accept a peer, run the SP handshake, pump it into the shared
/// socket, then go back to accepting.
#[embassy_executor::task(pool_size = REP_PIPES)]
async fn rep_listener_task(sock: &'static Socket<proto::Rep0>, stack: Stack<'static>) {
    let mut rx_buffer = [0u8; 2048];
    let mut tx_buffer = [0u8; 2048];
    let tcp = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    let mut listener = EmbassyTcpListener::new(tcp, REP_PORT);

    info!("[nng] REP0 listening on :{}", REP_PORT);
    serve_listener(sock, &mut listener, &EmbassyClock).await;
    warn!("[nng] REP listener returned — socket closed");
}

/// Answers requests arriving on any pipe of the REP socket with a telemetry
/// frame. The request body is logged but not otherwise interpreted.
#[embassy_executor::task(pool_size = REP_WORKERS)]
async fn rep_worker_task(sock: &'static Socket<proto::Rep0>, id: u8) {
    let mut ctx = unwrap!(sock.context().ok(), "no free NNG context");

    loop {
        let (req, responder) = match ctx.receive_request().await {
            Ok(pair) => pair,
            Err(e) => {
                warn!("[nng {}] receive error: {:?}", id, defmt::Debug2Format(&e));
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
        };

        info!(
            "[nng {}] request: {}",
            id,
            core::str::from_utf8(req.body()).unwrap_or("<non-utf8>")
        );

        if let Err(e) = responder.reply(frame_message()).await {
            warn!("[nng {}] reply error: {:?}", id, defmt::Debug2Format(&e));
        }
    }
}

/// One PUB pipe: accept a subscriber and feed it until it goes away.
#[embassy_executor::task(pool_size = PUB_PIPES)]
async fn pub_listener_task(sock: &'static Socket<proto::Pub0>, stack: Stack<'static>) {
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 4096];
    let tcp = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    let mut listener = EmbassyTcpListener::new(tcp, PUB_PORT);

    info!("[nng] PUB0 listening on :{}", PUB_PORT);
    serve_listener(sock, &mut listener, &EmbassyClock).await;
    warn!("[nng] PUB listener returned — socket closed");
}

/// Publishes a telemetry frame `PUBLISH_HZ` times a second. PUB drops rather
/// than blocks when a subscriber is slow, so this never stalls the sensors.
#[embassy_executor::task]
async fn publish_task(sock: &'static Socket<proto::Pub0>) -> ! {
    let interval = Duration::from_hz(PUBLISH_HZ);
    let mut ticker = Instant::now();
    let mut published = 0u32;

    loop {
        ticker += interval;
        Timer::at(ticker).await;

        if sock.pipe_count() == 0 {
            continue; // nobody listening; don't bother encoding
        }

        if let Err(e) = sock.publish(frame_message()).await {
            warn!("[nng pub] publish error: {:?}", defmt::Debug2Format(&e));
            continue;
        }

        published += 1;
        if published.is_multiple_of(PUBLISH_HZ as u32 * 10) {
            info!(
                "[nng pub] {} frames to {} subscriber(s)",
                published,
                sock.pipe_count()
            );
        }
    }
}
