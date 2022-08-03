#![no_std]

// #![deny(
//     missing_docs,
//     missing_debug_implementations,
//     missing_copy_implementations,
//     unstable_features,
//     warnings
// )]

pub mod mcp23008;
pub mod mcp23017;

/// Pin modes.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Mode {
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

/// Generic port definitions.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Port {
    /// Represent port A.
    GpioA,
    /// Represent port B.
    GpioB,
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
