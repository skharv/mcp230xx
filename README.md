# `mcp230xx`

> no_std driver for [MCP23017](http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf) (16-Bit I2C I/O Expander with Serial Interface module) and MCP23008 (8-Bit version)

<!-- [![Build Status](https://github.com/lucazulian/mcp23017/workflows/mcp23017-ci/badge.svg)](https://github.com/lucazulian/mcp23017/actions?query=workflow%3Amcp23017-ci) -->
[![crates.io](https://img.shields.io/crates/v/mcp230xx.svg)](https://crates.io/crates/mcp230xx)
[![Docs](https://docs.rs/mcp230xx/badge.svg)](https://docs.rs/mcp230xx)

## Basic usage

Include this [library](https://crates.io/crates/mcp230xx) as a dependency in your `Cargo.toml`:

```rust
[dependencies.mcp230xx]
version = "<version>"
```
Use [embedded-hal](https://github.com/rust-embedded/embedded-hal) implementation to get I2C handle and then create mcp23017 handle:

```rust
use mcp230xx::{MCP23017, Pin, Mode, Level};

if let Some(mut u) = mcp230xx::MCP23017::default(i2c) {
    u.set_mode(mcp, Pin::A0, Mode::Output).unwrap();
    println!("GPIOAB {:#?}", u.gpioab().unwrap());
    match u.set_pin(Pin::A0, Level::High).unwrap();
};
```

### Hardware address pins
![](docs/address-pins.jpg)

## Documentation

API Docs available on [docs.rs](https://docs.rs/mcp230xx)

Thise crate has evolved from [mcp23017](https://github.com/lucazulian/mcp23017).

## License

[MIT license](http://opensource.org/licenses/MIT)
