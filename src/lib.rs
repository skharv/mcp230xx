#![no_std]
//! MCP23017/MCP23008, a 16/8-Bit I2C I/O Expander with I2C Interface.

use bit_field::BitField;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use paste::paste;

/// Pin modes.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Direction {
    /// Represents input mode.
    Input,
    /// Represents output mode.
    Output,
}

/// Pin levels.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Level {
    /// High level
    High,
    /// Low level
    Low,
}

/// Pin Pull Up state.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PullUp {
    /// Weak pull up enabled
    Enabled,
    /// Weak pull up disabled, pin floating
    Disabled,
}

/// Pin Input polarity inversion.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Polarity {
    /// Inverted input polarity
    Inverted,
    /// Not inverted
    NotInverted,
}

/// Port for dual-port variants.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Port {
    /// Represent port A.
    A,
    /// Represent port B.
    B,
}

/// Defines errors
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Underlying bus error
    BusError(E),
    /// Interrupt pin not found
    InterruptPinError,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

/// Trait providing a register map of a chip variant
pub trait Map {
    type Pin: Into<usize> + Copy;
    fn map(reg: Register, pin: Self::Pin) -> (u8, usize);
}

/// Base MCP230xx register map.
#[allow(clippy::upper_case_acronyms)]
#[derive(Copy, Clone, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Register {
    IODIR = 0x00,
    IPOL = 0x01,
    GPINTEN = 0x02,
    DEFVAL = 0x03,
    INTCON = 0x04,
    IOCON = 0x05,
    GPPU = 0x06,
    INTF = 0x07,
    INTCAP = 0x08,
    GPIO = 0x09,
    OLAT = 0x10,
}

/// MCP23017 register map.
///
/// Note: This operates the chip in `IOCON.BANK=0` mode, i.e. even register addresses are bank 0.
/// This driver does not set `IOCON.BANK`, but the factory default is `0` and this driver does
/// not change that value.
///
/// See [the datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf) for more
/// information on the device.
#[allow(
    non_camel_case_types,
    clippy::upper_case_acronyms,
    missing_docs,
    dead_code
)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mcp23017;

#[derive(Debug, Copy, Clone, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(usize)]
pub enum Mcp23017Pin {
    A0 = 0,
    A1 = 1,
    A2 = 2,
    A3 = 3,
    A4 = 4,
    A5 = 5,
    A6 = 6,
    A7 = 7,
    B0 = 8,
    B1 = 9,
    B2 = 10,
    B3 = 11,
    B4 = 12,
    B5 = 13,
    B6 = 14,
    B7 = 15,
}

impl Map for Mcp23017 {
    type Pin = Mcp23017Pin;
    fn map(reg: Register, pin: Self::Pin) -> (u8, usize) {
        let mut addr = (reg as u8) << 1;
        let bit = pin as usize;
        if bit & 8 != 0 {
            addr |= 1;
        }
        (addr, bit & 7)
    }
}

/// MCP23008 Register mapping
/// See [the datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP23008-MCP23S08-Data-Sheet-20001919F.pdf) for more
/// information on the device.
#[allow(
    non_camel_case_types,
    clippy::upper_case_acronyms,
    missing_docs,
    dead_code
)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mcp23008;

#[derive(Debug, Copy, Clone, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(usize)]
pub enum Mcp23008Pin {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
    P4 = 4,
    P5 = 5,
    P6 = 6,
    P7 = 7,
}

impl Map for Mcp23008 {
    type Pin = Mcp23008Pin;
    fn map(reg: Register, pin: Self::Pin) -> (u8, usize) {
        (reg as u8, pin as usize)
    }
}

/// MCP230xx I2C GPIO extender.
#[derive(Clone, Copy, Debug)]
pub struct MCP230xx<I2C, V> {
    i2c: I2C,
    address: u8,
    variant: core::marker::PhantomData<V>,
}

macro_rules! bit_getter_setter {
    ($(#[$outer:meta])*
        $name:ident = ($reg:ident, $typ:ident, $set:ident, $clear:ident)
     ) => {
        $(#[$outer])*
        pub fn $name(&mut self, pin: V::Pin) -> Result<$typ, E> {
            let (addr, bit) = V::map(Register::$reg, pin);
            Ok(if self.bit(addr, bit)? {
                $typ::$set
            } else {
                $typ::$clear
            })
        }

        paste! {
            $(#[$outer])*
            pub fn [< set_ $name >](&mut self, pin: V::Pin, value: $typ) -> Result<(), E> {
                let (addr, bit) = V::map(Register::$reg, pin);
                self.set_bit(addr, bit, value == $typ::$set)
            }
        }
    };
}

impl<I2C, E, V> MCP230xx<I2C, V>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    V: Map,
{
    const DEFAULT_ADDRESS: u8 = 0x20;

    /// Creates an expander with the default configuration.
    pub fn new_default(i2c: I2C) -> Result<Self, Error<E>> {
        Self::new(i2c, Self::DEFAULT_ADDRESS)
    }

    /// Creates an expander with specific address.
    pub fn new(i2c: I2C, address: u8) -> Result<Self, Error<E>> {
        Ok(Self {
            i2c,
            address,
            variant: core::marker::PhantomData,
        })
    }

    /// Return the I2C address
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Read an 8 bit register
    fn read(&mut self, addr: u8) -> Result<u8, E> {
        let mut data = [0u8];
        self.i2c.write_read(self.address, &[addr], &mut data)?;
        Ok(data[0])
    }

    /// Write an 8 bit register
    fn write(&mut self, addr: u8, data: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[addr, data])
    }

    /// Get a single bit in a register
    fn bit(&mut self, addr: u8, bit: usize) -> Result<bool, E> {
        let data = self.read(addr)?;
        Ok(data.get_bit(bit))
    }

    /// Set a single bit in a register
    fn set_bit(&mut self, addr: u8, bit: usize, value: bool) -> Result<(), E> {
        let mut data = self.read(addr)?;
        data.set_bit(bit, value);
        self.write(addr, data)
    }

    bit_getter_setter!(
        /// Pin direction
        direction = (IODIR, Direction, Input, Output)
    );
    bit_getter_setter!(
        /// GPIO Pin value. This reads the actual hardware pin but writes to the
        /// OLAT register.
        gpio = (GPIO, Level, High, Low)
    );
    bit_getter_setter!(
        /// Output latch state
        output_latch = (OLAT, Level, High, Low)
    );
    bit_getter_setter!(
        /// Weak pull up resistor state
        pull_up = (GPPU, PullUp, Enabled, Disabled)
    );
    bit_getter_setter!(
        /// Input polarity inversion
        input_polarity = (IPOL, Polarity, Inverted, NotInverted)
    );
}
