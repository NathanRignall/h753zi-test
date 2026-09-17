//! Ethernet / `embassy-net` smoke test for the ST Nucleo-H753ZI.
//!
//! Brings up the on-board LAN8742A PHY over RMII, takes an address (the fixed
//! one below by default, DHCP with `--no-default-features`), then exercises the
//! stack from both directions:
//!
//! * inbound  — ICMP echo, TCP echo on :1234, UDP echo on :1234, and an
//!   `nng-core` telemetry sockets (REP0 :5555, PUB0 :5556)
//! * outbound — one connection at startup: back to the peer on :1234 for a
//!   point-to-point link, or a DNS lookup plus HTTP `HEAD` on a DHCP network
//!
//! Everything is reported over RTT with `defmt`; LD1 blinks as a heartbeat and
//! LD2 tracks link state.

#![no_std]
#![no_main]

extern crate alloc;

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
#[cfg(not(feature = "static-ip"))]
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Stack, StackResources};
use embassy_stm32::eth::{Ethernet, GenericPhy, PacketQueue, Sma};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_stm32::peripherals::{ETH, ETH_SMA};
use embassy_stm32::rng::Rng;
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, bind_interrupts, dma, eth, peripherals, rng, uid};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_alloc::LlffHeap as Heap;
use embedded_io_async::Write;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

/// `nng_core::Message` is backed by `Vec<u8>`, so the firmware needs a heap.
#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 32 * 1024;
static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

mod gnss;
mod imu;
mod nng_net;
mod telemetry;

/// I2C1, shared by the GNSS and IMU tasks. Everything runs on one executor,
/// so a `NoopRawMutex` is enough.
pub type Bus = Mutex<NoopRawMutex, I2c<'static, Async, Master>>;

/// Port used by both the TCP and the UDP echo server.
const ECHO_PORT: u16 = 1234;
/// Host the outbound self-test resolves and connects to (DHCP builds only).
#[cfg(not(feature = "static-ip"))]
const PROBE_HOST: &str = "example.com";

#[cfg(feature = "static-ip")]
mod static_ip {
    use embassy_net::{Ipv4Address, Ipv4Cidr};

    /// Address this board takes.
    pub const ADDRESS: Ipv4Cidr = Ipv4Cidr::new(Ipv4Address::new(192, 168, 50, 11), 24);
    /// The host on the other end of the cable, used as gateway, resolver and
    /// as the target of the outbound connect test.
    pub const PEER: Ipv4Address = Ipv4Address::new(192, 168, 50, 10);
}

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
    HASH_RNG => rng::InterruptHandler<peripherals::RNG>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>;
    DMA1_STREAM1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
});

type Device = Ethernet<'static, ETH, GenericPhy<Sma<'static, ETH_SMA>>>;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    // 400 MHz off the HSI PLL; HSI48 feeds the RNG.
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Default::default()); // needed for RNG
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 MHz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let p = embassy_stm32::init(config);
    info!("Nucleo-H753ZI network test — sysclk 400 MHz");

    // Before any `nng_core::Message` exists.
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }

    // LD1 (green, PB0) heartbeat, LD2 (yellow, PE1) link state, LD3 (red, PB14)
    // is left for panics to be visible via the debugger.
    let heartbeat = Output::new(p.PB0, Level::Low, Speed::Low);
    let link_led = Output::new(p.PE1, Level::Low, Speed::Low);

    // Qwiic bus: I2C1 on PB8 (SCL, CN7 pin 2 / D15) and PB9 (SDA, CN7 pin 4 /
    // D14), carrying the ZOE-M8Q (0x42) and BNO08x (0x4A). The breakouts
    // carry their own pull-ups. Started before the network so a missing
    // cable does not hide the sensors.
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz::khz(100);
    // The BNO08x clock-stretches for a while when busy; be generous.
    i2c_config.timeout = Duration::from_millis(200);
    let i2c = I2c::new(p.I2C1, p.PB8, p.PB9, p.DMA1_CH0, p.DMA1_CH1, Irqs, i2c_config);
    static I2C_BUS: StaticCell<Bus> = StaticCell::new();
    let bus: &'static Bus = I2C_BUS.init(Mutex::new(i2c));
    spawner.spawn(unwrap!(gnss::gnss_task(bus)));
    // BNO08x RST (active low) on PA5 = Arduino D13, CN7 pin 10. Held high
    // when idle; the IMU task pulses it at every bring-up.
    let imu_reset = Output::new(p.PA5, Level::High, Speed::Low);
    spawner.spawn(unwrap!(imu::imu_task(bus, imu_reset)));
    spawner.spawn(unwrap!(telemetry::print_task()));

    let mut rng = Rng::new(p.RNG, Irqs);
    let mut seed = [0; 8];
    rng.fill_bytes(&mut seed);
    let seed = u64::from_le_bytes(seed);

    // Locally-administered MAC derived from the chip's unique ID, so two boards
    // on the same segment don't collide.
    let id = uid::uid();
    let mac_addr = [0x02, 0x00, id[0], id[4], id[8], id[11]];
    info!("MAC address: {:02x}", mac_addr);

    static PACKETS: StaticCell<PacketQueue<8, 8>> = StaticCell::new();
    let device = Ethernet::new(
        PACKETS.init(PacketQueue::<8, 8>::new()),
        p.ETH,
        Irqs,
        p.PA1,  // REF_CLK
        p.PA7,  // CRS_DV
        p.PC4,  // RXD0
        p.PC5,  // RXD1
        p.PG13, // TXD0
        p.PB13, // TXD1
        p.PG11, // TX_EN
        mac_addr,
        p.ETH_SMA,
        p.PA2, // MDIO
        p.PC1, // MDC
    );

    #[cfg(not(feature = "static-ip"))]
    let net_config = {
        info!("addressing: DHCP");
        embassy_net::Config::dhcpv4(Default::default())
    };
    #[cfg(feature = "static-ip")]
    let net_config = {
        info!("addressing: static {}", static_ip::ADDRESS);
        let mut dns_servers = heapless::Vec::new();
        unwrap!(dns_servers.push(static_ip::PEER));
        embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: static_ip::ADDRESS,
            gateway: Some(static_ip::PEER),
            dns_servers,
        })
    };

    // 2 TCP echo + 1 UDP echo + 1 outbound probe + nng_net's REP and PUB
    // listeners + the stack's own DHCP/DNS sockets. Overrunning this panics
    // inside smoltcp with "adding a socket to a full SocketSet".
    static RESOURCES: StaticCell<StackResources<16>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(device, net_config, RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(unwrap!(net_task(runner)));
    spawner.spawn(unwrap!(link_task(stack, link_led)));
    spawner.spawn(unwrap!(heartbeat_task(heartbeat)));

    info!("waiting for link + address...");
    // With a static address `wait_config_up` returns immediately, so wait for
    // the PHY as well before anything tries to send.
    stack.wait_link_up().await;
    stack.wait_config_up().await;

    match stack.config_v4() {
        Some(cfg) => info!(
            "IPv4 up: addr={} gw={} dns={}",
            cfg.address,
            cfg.gateway,
            cfg.dns_servers.as_slice()
        ),
        None => warn!("config_v4() is None even though the stack reports up"),
    }

    spawner.spawn(unwrap!(tcp_echo_task(stack, 0)));
    spawner.spawn(unwrap!(tcp_echo_task(stack, 1)));
    spawner.spawn(unwrap!(udp_echo_task(stack)));

    // nng-core sockets carrying telemetry over the same stack.
    nng_net::spawn(&spawner, stack);

    // One-shot outbound test: DNS, then a TCP connect + HTTP HEAD.
    outbound_probe(stack).await;

    info!(
        "ready — echo on :{}, nng telemetry: REP :{}, PUB :{}",
        ECHO_PORT,
        nng_net::REP_PORT,
        nng_net::PUB_PORT
    );

    // Keep the main task alive; everything else runs in spawned tasks.
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// Mirror link/address state onto LD2 and the log.
#[embassy_executor::task]
async fn link_task(stack: Stack<'static>, mut led: Output<'static>) -> ! {
    let mut was_up = false;
    loop {
        let is_up = stack.is_link_up();
        if is_up != was_up {
            was_up = is_up;
            if is_up {
                info!("PHY link up");
            } else {
                warn!("PHY link down");
            }
        }
        led.set_level(if stack.is_config_up() { Level::High } else { Level::Low });
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// LD1 blinks so it is obvious the executor is still scheduling.
#[embassy_executor::task]
async fn heartbeat_task(mut led: Output<'static>) -> ! {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// TCP echo server. Two instances run so a second client is not left hanging.
#[embassy_executor::task(pool_size = 2)]
async fn tcp_echo_task(stack: Stack<'static>, id: u8) -> ! {
    let mut rx_buffer = [0u8; 2048];
    let mut tx_buffer = [0u8; 2048];
    let mut buf = [0u8; 1024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(30)));

        info!("[tcp {}] listening on :{}", id, ECHO_PORT);
        if let Err(e) = socket.accept(ECHO_PORT).await {
            warn!("[tcp {}] accept error: {:?}", id, e);
            continue;
        }
        info!("[tcp {}] connected from {:?}", id, socket.remote_endpoint());

        let banner = b"h753zi echo ready\r\n";
        if let Err(e) = socket.write_all(banner).await {
            warn!("[tcp {}] write error: {:?}", id, e);
            socket.abort();
            continue;
        }


        let mut total = 0usize;
        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    info!("[tcp {}] peer closed after {} bytes", id, total);
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("[tcp {}] read error: {:?}", id, e);
                    break;
                }
            };
            total += n;
            if let Err(e) = socket.write_all(&buf[..n]).await {
                warn!("[tcp {}] write error: {:?}", id, e);
                break;
            }
        }
        // FIN rather than RST: `abort()` would drop whatever is still queued.
        socket.close();
        let _ = with_timeout(Duration::from_secs(2), socket.flush()).await;
    }
}

/// UDP echo server — datagrams come straight back to the sender.
#[embassy_executor::task]
async fn udp_echo_task(stack: Stack<'static>) -> ! {
    let mut rx_meta = [PacketMetadata::EMPTY; 8];
    let mut rx_buffer = [0u8; 2048];
    let mut tx_meta = [PacketMetadata::EMPTY; 8];
    let mut tx_buffer = [0u8; 2048];
    let mut buf = [0u8; 1024];

    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    unwrap!(socket.bind(ECHO_PORT));
    info!("[udp] listening on :{}", ECHO_PORT);

    loop {
        match socket.recv_from(&mut buf).await {
            Ok((n, meta)) => {
                info!("[udp] {} bytes from {:?}", n, meta.endpoint);
                if let Err(e) = socket.send_to(&buf[..n], meta.endpoint).await {
                    warn!("[udp] send error: {:?}", e);
                }
            }
            Err(e) => warn!("[udp] recv error: {:?}", e),
        }
    }
}

/// Outbound test for a DHCP network: resolve a hostname through the
/// DHCP-supplied resolver, then connect to it and fetch a response.
#[cfg(not(feature = "static-ip"))]
async fn outbound_probe(stack: Stack<'static>) {
    let addr = match with_timeout(Duration::from_secs(10), stack.dns_query(PROBE_HOST, DnsQueryType::A)).await {
        Ok(Ok(addrs)) if !addrs.is_empty() => {
            info!("[out] {} resolved to {:?}", PROBE_HOST, addrs.as_slice());
            addrs[0]
        }
        Ok(Ok(_)) => {
            warn!("[out] DNS returned no records for {}", PROBE_HOST);
            return;
        }
        Ok(Err(e)) => {
            warn!("[out] DNS error: {:?}", e);
            return;
        }
        Err(_) => {
            warn!("[out] DNS timed out");
            return;
        }
    };

    let mut rx_buffer = [0u8; 2048];
    let mut tx_buffer = [0u8; 512];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    if let Err(e) = socket.connect((addr, 80)).await {
        warn!("[out] connect error: {:?}", e);
        return;
    }
    info!("[out] connected to {:?}:80", addr);

    let req = b"HEAD / HTTP/1.1\r\nHost: example.com\r\nConnection: close\r\n\r\n";
    if let Err(e) = socket.write_all(req).await {
        warn!("[out] write error: {:?}", e);
        return;
    }

    let mut buf = [0u8; 256];
    match socket.read(&mut buf).await {
        Ok(n) => {
            let line = buf[..n].split(|b| *b == b'\r').next().unwrap_or(&[]);
            info!("[out] response: {}", core::str::from_utf8(line).unwrap_or("<non-utf8>"));
        }
        Err(e) => warn!("[out] read error: {:?}", e),
    }
    socket.close();
    let _ = with_timeout(Duration::from_secs(2), socket.flush()).await;
}

/// Outbound test for a point-to-point link: connect back to the host at the
/// other end of the cable. Run `nc -l 1234` there first; if nothing is
/// listening this just logs a refusal, which is still proof that TX works.
#[cfg(feature = "static-ip")]
async fn outbound_probe(stack: Stack<'static>) {
    let peer = static_ip::PEER;

    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 512];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(5)));

    info!("[out] connecting to {}:{}...", peer, ECHO_PORT);
    match with_timeout(Duration::from_secs(5), socket.connect((peer, ECHO_PORT))).await {
        Ok(Ok(())) => {
            info!("[out] connected to {}:{}", peer, ECHO_PORT);
            if let Err(e) = socket.write_all(b"hello from h753zi\r\n").await {
                warn!("[out] write error: {:?}", e);
            } else {
                info!("[out] sent greeting");
            }
            socket.close();
            match with_timeout(Duration::from_secs(2), socket.flush()).await {
                Ok(Ok(())) => info!("[out] greeting acknowledged, connection closed"),
                Ok(Err(e)) => warn!("[out] flush error: {:?}", e),
                Err(_) => warn!("[out] flush timed out — peer never acked"),
            }
        }
        Ok(Err(e)) => warn!(
            "[out] connect to {}:{} failed ({:?}) — is `nc -l {}` running on the host?",
            peer, ECHO_PORT, e, ECHO_PORT
        ),
        Err(_) => warn!("[out] connect to {}:{} timed out", peer, ECHO_PORT),
    }
}
