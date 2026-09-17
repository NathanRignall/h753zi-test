//! REQ client for the `nng-core` REP0 socket running on the Nucleo-H753ZI.
//!
//!     cargo run -- [addr] [payload] [count]
//!
//! Defaults to `tcp://192.168.50.11:5555`, payload `hello`, one request.

use std::time::Instant;

use nng_core::{Message, socket::reqrep0};

#[tokio::main]
async fn main() {
    let mut args = std::env::args().skip(1);
    let addr = args.next().unwrap_or_else(|| "tcp://192.168.50.11:5555".into());
    let payload = args.next().unwrap_or_else(|| "hello".into());
    let count: usize = args
        .next()
        .map(|c| c.parse().expect("count must be a number"))
        .unwrap_or(1);

    println!("dialing {addr}");
    let mut req = reqrep0::Req0::dial(&addr).await.expect("dial failed");

    for i in 0..count {
        let body = if count == 1 {
            payload.clone()
        } else {
            format!("{payload} #{i}")
        };

        let mut msg = Message::new();
        msg.push_back(body.as_bytes());

        let started = Instant::now();
        let reply = req.request(msg).await.expect("request failed");
        let elapsed = started.elapsed();

        println!(
            "{:>5.2} ms  {}",
            elapsed.as_secs_f64() * 1000.0,
            String::from_utf8_lossy(reply.body())
        );
    }
}
