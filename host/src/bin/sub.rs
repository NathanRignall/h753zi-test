//! Streams telemetry from the board's `nng-core` PUB0 socket.
//!
//!     cargo run --bin sub -- [addr] [count]
//!
//! Defaults: `tcp://192.168.50.11:5556`, runs until interrupted.
//!
//! Subscribes to `telemetry_wire::MAGIC`, which is the first field of every
//! frame, so the filter and the format stay in sync by construction.

use std::time::Instant;

use nng_core::socket::pubsub0;
use telemetry_wire::{Frame, MAGIC};

const DEFAULT_ADDR: &str = "tcp://192.168.50.11:5556";

#[tokio::main]
async fn main() {
    let mut args = std::env::args().skip(1);
    let addr = args.next().unwrap_or_else(|| DEFAULT_ADDR.into());
    let count: usize = args
        .next()
        .map(|c| c.parse().expect("count must be a number"))
        .unwrap_or(usize::MAX);

    println!("subscribing to {addr}");
    let mut sub = pubsub0::Sub0::dial(&addr).await.expect("dial failed");
    sub.subscribe_to(&MAGIC);

    let started = Instant::now();
    let mut received = 0usize;
    let mut last_uptime: Option<u64> = None;

    while received < count {
        let msg = sub.next().await.expect("receive failed");
        match Frame::decode(msg.body()) {
            Ok(frame) => {
                let gap = last_uptime.map(|prev| frame.uptime_ms.saturating_sub(prev));
                last_uptime = Some(frame.uptime_ms);
                match gap {
                    Some(ms) => println!("{frame}  (+{ms} ms)"),
                    None => println!("{frame}"),
                }
            }
            Err(e) => println!("undecodable frame ({e}): {} bytes", msg.body().len()),
        }
        received += 1;
    }

    let secs = started.elapsed().as_secs_f64();
    println!("\n{received} frames in {secs:.1}s ({:.1} Hz)", received as f64 / secs);
}
