//! Polls the board's `nng-core` REP0 socket for telemetry.
//!
//!     cargo run --bin req -- [addr] [count] [interval_ms]
//!
//! Defaults: `tcp://192.168.50.11:5555`, one request, 200 ms apart.

use std::time::{Duration, Instant};

use nng_core::{Message, socket::reqrep0};
use telemetry_wire::Frame;

const DEFAULT_ADDR: &str = "tcp://192.168.50.11:5555";

#[tokio::main]
async fn main() {
    let mut args = std::env::args().skip(1);
    let addr = args.next().unwrap_or_else(|| DEFAULT_ADDR.into());
    let count: usize = parse(args.next(), 1, "count");
    let interval = Duration::from_millis(parse(args.next(), 200, "interval_ms"));

    println!("dialing {addr}");
    let mut req = reqrep0::Req0::dial(&addr).await.expect("dial failed");

    for i in 0..count {
        if i > 0 {
            tokio::time::sleep(interval).await;
        }

        let mut msg = Message::new();
        msg.push_back(b"telemetry");

        let started = Instant::now();
        let reply = req.request(msg).await.expect("request failed");
        let rtt = started.elapsed();

        match Frame::decode(reply.body()) {
            Ok(frame) => println!("{frame}  [{:.2} ms]", rtt.as_secs_f64() * 1000.0),
            Err(e) => println!("undecodable reply ({e}): {} bytes", reply.body().len()),
        }
    }
}

fn parse<T: std::str::FromStr>(arg: Option<String>, default: T, name: &str) -> T {
    match arg {
        Some(v) => v.parse().unwrap_or_else(|_| panic!("{name} must be a number")),
        None => default,
    }
}
