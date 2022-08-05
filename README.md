# `mcp230xx`

This crate is a `no_std` driver for the
[MCP23017](http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf) and
[MCP23008](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP23008-MCP23S08-Data-Sheet-20001919F.pdf)
16-Bit/8-Bit I2C I/O Expanders.

[![Build Status](https://github.com/quartiq/mcp230xx/workflows/ci/badge.svg)](https://github.com/quartiq/mcp230xx/actions?query=workflow%3Aci)
[![crates.io](https://img.shields.io/crates/v/mcp230xx.svg)](https://crates.io/crates/mcp230xx)
[![Docs](https://docs.rs/mcp230xx/badge.svg)](https://docs.rs/mcp230xx)

## Basic usage

Include this [library](https://crates.io/crates/mcp230xx) as a dependency in your `Cargo.toml`:

```rust
[dependencies]
mcp230xx = "0.1"
```

Use [embedded-hal](https://github.com/rust-embedded/embedded-hal) implementation to get I2C handle and then create mcp23017 handle:

```rust
use mcp230xx::{Mcp230xx, Mcp23017Reg, Direction, Level};

let mut u = Mcp230xx<_, Mcp23017>::default(i2c).unwrap();
u.set_direction(Mcp23017::Pin::A0, Direction::Output).unwrap();
u.set_gpio(Mcp23017::Pin::A0, Level::High).unwrap();
u.gpio(Pin::A0).unwrap();
```

### Hardware address pins

![Address table](docs/address-pins.jpg)

## Documentation

API Docs available on [docs.rs](https://docs.rs/mcp230xx)

Thise crate has evolved from [mcp23017](https://github.com/lucazulian/mcp23017).

Minimum supported Rust version (MSRV) is 1.62.0.

## License

[MIT license](http://opensource.org/licenses/MIT)
