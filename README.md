# h753zi-net-test

An [Embassy](https://embassy.dev/book/) networking smoke test for the
**ST Nucleo-H753ZI** (STM32H753ZI, Cortex-M7). It brings up the on-board
LAN8742A PHY over RMII, takes an address, and then exercises `embassy-net`
from both directions.

The default build is wired for a board plugged straight into a host:

| | |
|---|---|
| host | `192.168.50.10` |
| board | `192.168.50.11/24` |

Both are set in the `static_ip` module at the top of `src/main.rs`. Build
`--no-default-features` to use DHCP instead.

Dev dependencies come from a Nix flake; flashing and logging go through
`probe-rs` and the Nucleo's on-board ST-LINK.

## What it tests

| Direction | Test | How to check |
|---|---|---|
| inbound | ICMP echo (smoltcp answers it) | `ping <addr>` |
| inbound | TCP echo server on `:1234` (2 concurrent clients) | `nc <addr> 1234` |
| inbound | UDP echo server on `:1234` | `nc -u <addr> 1234` |
| inbound | **`nng-core` REP0 socket on `:5555`** | `host-req` (below) |
| outbound | TCP connect to the host on `:1234` + a greeting | `nc -l 1234` on the host |
| outbound (DHCP build) | DNS lookup of `example.com`, then TCP + `HEAD /` | RTT log at startup |
| board | LD1 (green) blinks at 1 Hz — executor is alive | look at the board |
| board | LD2 (yellow) on once the stack has an IP | look at the board |

All of it is logged over RTT with `defmt`.

## Sensors on the Qwiic bus (I2C1)

Two Qwiic breakouts share I2C1 (100 kHz) behind an `embassy-sync` mutex;
each task locks it only for the duration of one transfer.

### GNSS — u-blox ZOE-M8Q (`0x42`)

`src/gnss.rs` parses the NMEA `GGA`/`RMC` sentences the module emits by
default and logs fix status, position, altitude, satellites, speed and
course. Nothing is sent to the module, so any u-blox M8 breakout with
NMEA-over-I2C enabled works.

### IMU — BNO08x (`0x4A`)

`src/imu.rs` is a small SHTP/SH-2 driver: it resets the sensor, waits for
it to initialise, reads the product ID, enables the Rotation Vector and
Accelerometer reports at 20 Hz (each one confirmed, resent if not), and
publishes orientation and acceleration. No INT pin is needed; the sensor is
polled every 10 ms.

**Wire RST.** The BNO08x's sensor hub hangs if the host disappears while
reports are streaming — which is exactly what reflashing the Nucleo does.
In that state it still answers queries (product ID, get-feature) but
silently ignores set-feature, reinitialise and the soft reset, so only the
RST pin or a power cycle recovers it. The firmware pulses **PA5 (Arduino
D13, CN7 pin 10)** low at every bring-up; connect it to the breakout's
`RST` pin. Without it, unplug and replug the IMU after each flash.

The Rotation Vector uses the magnetometer, so straight after power-up it
reports `status 0` and an accuracy of ±180° until the sensor has been
rotated through a few orientations (a slow figure-of-eight does it). If
absolute heading is not needed, switch the report to the Game Rotation
Vector (`0x08`, 12-byte report), which calibrates in seconds.

### Data

`src/telemetry.rs` holds the latest reading from every sensor in one
`Telemetry` struct (GNSS `Gga`/`Rmc`, IMU `Orientation`/`Acceleration`,
plus timestamps), typed with [`uom`](https://crates.io/crates/uom)
quantities (`Angle`, `Length`, `Velocity`, `Acceleration`, `Ratio`). Sensor
tasks call `telemetry::update`, consumers call `telemetry::snapshot`. A
printer task logs one line a second:

```
pos 52.179779,0.147946 alt 16.4m (Gnss, 6 sats) | yaw -70.8 pitch -2.9 roll -34.8 (imu 0/3)
```

### Wiring

Qwiic cable to the Nucleo's Arduino headers (daisy-chain the second
breakout from the first), plus one jumper for the IMU reset:

| Qwiic wire | Signal | Nucleo pin |
|---|---|---|
| black | GND | CN8 pin 11 (GND) |
| red | 3V3 | CN8 pin 7 (+3V3) |
| blue | SDA | CN7 pin 4 (D14 / PB9) |
| yellow | SCL | CN7 pin 2 (D15 / PB8) |
| jumper | BNO08x `RST` | CN7 pin 10 (D13 / PA5) |

The breakouts' own pull-ups are used. With nothing plugged in the tasks log
a `Timeout` warning and keep retrying. A GNSS `NAK` is normal for up to
~30 s after the module powers up (it does not acknowledge its address until
it has data). Expect `no fix yet` indoors — the ZOE-M8Q needs an antenna and
a view of the sky.

## Setup

```bash
nix develop
```

That gives you Rust 1.92 with the `thumbv7em-none-eabihf` target,
`probe-rs`, `flip-link` and `cargo-binutils`. With
[direnv](https://direnv.net/) installed, `direnv allow` loads the same shell
automatically.

The toolchain is pinned in `rust-toolchain.toml`, so if you'd rather not use
Nix, `rustup` will honour the same versions — you then need `probe-rs` and
`flip-link` on your `PATH` yourself.

## Flash and run

Plug the Nucleo into USB (CN1, the ST-LINK port) and connect it to your
network over the RJ45 jack.

```bash
nix develop --command cargo run --release
```

`cargo run` is wired to `probe-rs run --chip STM32H753ZITx` in
`.cargo/config.toml`, so this flashes the board and then streams `defmt` logs
from RTT until you hit Ctrl-C.

Expected output:

```
0.000000 [INFO ] Nucleo-H753ZI network test — sysclk 400 MHz
0.000000 [INFO ] MAC address: [02, 00, 41, 18, 31, 38]
0.010620 [INFO ] addressing: static 192.168.50.11/24
0.010742 [INFO ] waiting for link + address...
2.010803 [INFO ] IPv4 up: addr=192.168.50.11/24 gw=Some(192.168.50.10) dns=[192.168.50.10]
2.010955 [INFO ] [udp] listening on :1234
2.011016 [INFO ] [tcp 0] listening on :1234
2.011016 [INFO ] PHY link up
2.012207 [INFO ] [out] connected to 192.168.50.10:1234
2.012664 [INFO ] [out] greeting acknowledged, connection closed
2.012695 [INFO ] ready — try `ping <addr>`, `nc <addr> 1234`, `nc -u <addr> 1234`
```

The `[out]` lines need a listener on the host; start it before resetting the
board:

```bash
nc -l 1234
```

Then, from the host:

```bash
ping 192.168.50.11
```

```bash
nc 192.168.50.11 1234
```

Anything you type comes straight back, and each read is logged on the RTT
side. `nc -u 192.168.50.11 1234` does the same over UDP.

## nng-core over embassy-net

The board runs a real [`nng-core`](../nng-core) REP0 socket on `:5555`, served
straight off the same `embassy-net` stack — `nng_core::sock::Socket` with
`EmbassyTcpListener` + `serve_listener`, two listener tasks (one TCP socket
each, so two concurrent NNG pipes) feeding two worker tasks through the
socket's contexts. Each request comes back as `pong from h753zi: <body>`.

`host-req/` is the other end: an ordinary std/tokio `nng-core` REQ client.

```bash
cd host-req && cargo run --release
```

```bash
cd host-req && cargo run --release -- tcp://192.168.50.11:5555 ping 20
```

Arguments are `[addr] [payload] [count]`; with a count above 1 each request is
suffixed `#0`, `#1`, … Measured on this setup: **~0.65 ms** per round trip, and
two clients can run at once.

```
dialing tcp://192.168.50.11:5555
 0.73 ms  pong from h753zi: ping #0
 0.70 ms  pong from h753zi: ping #1
 0.61 ms  pong from h753zi: ping #2
```

Board side:

```
2.011016 [INFO ] [nng] REP0 listening on :5555
2.011016 [INFO ] [nng] REP0 listening on :5555
83.714630 [INFO ] [nng 1] request: ping #0
83.715301 [INFO ] [nng 1] request: ping #1
```

Notes:

* `nng-core` is a path dependency with `default-features = false, features =
  ["embassy"]` — no `std`, no tokio.
* `nng_core::Message` is `Vec<u8>`-backed, so `main.rs` installs an
  `embedded-alloc` heap (32 KB) before anything touches a `Message`.
* On Embassy the NNG pipe count is fixed at the number of listener tasks you
  spawn, so `SocketConfig::max_pipes` must match `NNG_PIPES`. A third
  simultaneous client is accepted at TCP level and then closed at the SP level.
* `host-req/.cargo/config.toml` points `build.target` back at the host triple,
  because cargo would otherwise inherit the Cortex-M target from this crate's
  config. Change it if you are not on Apple silicon.

## Useful commands

```bash
nix develop --command probe-rs list
```

```bash
nix develop --command probe-rs attach --chip STM32H753ZITx target/thumbv7em-none-eabihf/release/h753zi-net-test
```

`probe-rs attach` connects to a board that is already running without
re-flashing it.

```bash
nix develop --command cargo size --release -- -A
```

Set the log level per-run with `DEFMT_LOG` (`.cargo/config.toml` defaults it
to `info`):

```bash
DEFMT_LOG=trace nix develop --command cargo run --release
```

## On a network with DHCP

```bash
nix develop --command cargo run --release --no-default-features
```

The address then comes from DHCP (watch the RTT log for it), and the outbound
test becomes a DNS lookup of `example.com` followed by an HTTP `HEAD`.

## Layout

| File | Purpose |
|---|---|
| `flake.nix` | dev shell: Rust toolchain, probe-rs, flip-link |
| `rust-toolchain.toml` | pinned channel/target, read by both Nix and rustup |
| `.cargo/config.toml` | default target, `probe-rs` runner, `flip-link`, `DEFMT_LOG` |
| `memory.x` | STM32H753ZI flash/RAM map (2 MB flash, 512 KB AXI SRAM) |
| `build.rs` | installs `memory.x` and passes the `link.x`/`defmt.x` linker args |
| `src/main.rs` | the test itself |
| `host-req/` | std/tokio `nng-core` REQ client for the board's REP socket |

Versions: `embassy-stm32` 0.6.0, `embassy-net` 0.9.1, `embassy-executor`
0.10.0, `embassy-time` 0.5.1, `nng-core` 0.3.0 (path), Rust 1.92,
probe-rs 0.32.

## Notes

* Pin assignment is the Nucleo-144 RMII layout: `PA1` REF_CLK, `PA2` MDIO,
  `PA7` CRS_DV, `PC1` MDC, `PC4/PC5` RXD0/1, `PG13`/`PB13` TXD0/1, `PG11`
  TX_EN. A different H7 board will likely differ — the H747XIH, for one, uses
  `PG12` for TXD1.
* The MAC address is derived from the chip's unique ID (locally-administered,
  `02:00:…`), so two boards on one segment won't collide.
* `embassy-net`'s `auto-icmp-echo-reply` feature is required for the board to
  answer pings — smoltcp 0.13 gates the automatic echo reply behind it. Without
  it the board still ARPs and serves TCP/UDP, it is just silently unpingable.
* Ethernet DMA descriptors and buffers live in AXI SRAM. The data cache is
  left off (Embassy doesn't enable it), which is what makes that safe without
  MPU setup — if you enable the D-cache later, you'll need to mark that region
  non-cacheable or move the descriptors to SRAM3 in D2.
