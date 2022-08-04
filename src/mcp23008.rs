//! MCP23008, 8-Bit I2C I/O Expander with Serial Interface module.
//!
//! See [the datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP23008-MCP23S08-Data-Sheet-20001919F.pdf) for more
//! information on the device.

pub use super::*;
use bit_field::BitField;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use num_enum::TryFromPrimitive;

/// MCP23008 Register addresses
#[allow(non_camel_case_types, clippy::upper_case_acronyms, dead_code)]
#[derive(Debug, Copy, Clone, PartialEq)]
enum Register {
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

/// GPIO pin
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Pin {
    Gp0 = 0,
    Gp1 = 1,
    Gp2 = 2,
    Gp3 = 3,
    Gp4 = 4,
    Gp5 = 5,
    Gp6 = 6,
    Gp7 = 7,
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

/// MCP23008 I2C GPIO extender.
/// See the crate-level documentation for general info on the device and the operation of this
/// driver.
#[derive(Clone, Copy, Debug)]
pub struct MCP23008<I2C> {
    com: I2C,
    /// The I2C slave address of this device.
    pub address: u8,
}

impl<I2C, E> MCP23008<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// The default I2C address of the MCP23008.
    const DEFAULT_ADDRESS: u8 = 0x20;

    /// Creates an expander with the default configuration.
    pub fn new_default(i2c: I2C) -> Result<MCP23008<I2C>, Error<E>> {
        MCP23008::new(i2c, Self::DEFAULT_ADDRESS)
    }

    /// Creates an expander with specific address.
    pub fn new(i2c: I2C, address: u8) -> Result<MCP23008<I2C>, Error<E>> {
        Ok(MCP23008 { com: i2c, address })
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        let mut data = [0u8];
        self.com.write_read(self.address, &[reg as u8], &mut data)?;
        Ok(data[0])
    }

    fn write_register(&mut self, reg: Register, data: u8) -> Result<(), E> {
        self.com.write(self.address, &[reg as u8, data])
    }

    /// Reads the GPIO register and returns its current 8 bit value.
    pub fn read_gpio(&mut self) -> Result<u8, E> {
        self.read_register(Register::GPIO)
    }

    /// Writes all the pins (GPIO) with the value at the same time.
    pub fn write_gpio(&mut self, value: u8) -> Result<(), E> {
        self.write_register(Register::GPIO, value)
    }

    fn bit(&mut self, pin: Pin, reg: Register) -> Result<bool, E> {
        let data = self.read_register(reg)?;
        Ok(data.get_bit(pin as usize))
    }

    /// Updates a single bit in the register associated with the given pin.
    /// This will read the GPIO register,
    /// set the bit (as specified by the pin position within the register), and write the register
    /// back to the device.
    fn set_bit(&mut self, pin: Pin, value: bool, reg: Register) -> Result<(), E> {
        let mut data = self.read_register(reg)?;
        data.set_bit(pin as usize, value);
        self.write_register(reg, data)
    }

    /// Sets the mode for a single pin to either `Mode::Input` or `Mode::Output`.
    pub fn set_mode(&mut self, pin: Pin, mode: Mode) -> Result<(), E> {
        self.set_bit(pin, mode == Mode::Input, Register::IODIR)
    }

    /// Writes a single bit to a single pin (GPIOA).
    /// This function accesses the GPIO registers.
    pub fn write_pin(&mut self, pin: Pin, value: Level) -> Result<(), E> {
        self.set_bit(pin, value == Level::High, Register::GPIO)
    }

    /// Reads a single pin (GPIO).
    pub fn read_pin(&mut self, pin: Pin) -> Result<bool, E> {
        self.bit(pin, Register::GPIO)
    }

    /// Writes a single bit to a single pin (OLAT).
    pub fn write_output_latch(&mut self, pin: Pin, value: Level) -> Result<(), E> {
        self.set_bit(pin, value == Level::High, Register::OLAT)
    }

    /// Enables or disables the internal pull-up resistor for a single pin (GPPU).
    pub fn set_pull_up(&mut self, pin: Pin, value: PullUp) -> Result<(), E> {
        self.set_bit(pin, value == PullUp::Enabled, Register::GPPU)
    }

    /// Inverts the input polarity for a single pin (IPOL).
    pub fn set_input_polarity(&mut self, pin: Pin, value: Polarity) -> Result<(), E> {
        self.set_bit(pin, value == Polarity::Inverted, Register::IPOL)
    }
}
