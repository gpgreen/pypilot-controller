//! Board Support Crate for [PyPilot Controller] (and compatible boards; [see below](#compatible-boards)).
//!
//! This crate provides abstractions for interfacing with the hardware of PyPilot Controller.  It
//! re-exports functionality from the underlying HAL in ways that make more sense for this
//! particular board.  For example, the pins are named by what is printed on the PCB instead of the
//! MCU names.
//!
//! # Examples
//! A number of examples can be found in the [`examples/`][ex] subdirectory of this crate.
//!
//! [ex]: https://github.com/gpgreen/power-monitor-rust/tree/master/chart-plotter-hat/examples
//!
//! # Getting Started
//! Please follow the guide from [`avr-hal`'s README][guide] for steps on how to set up a project
//! with this board.  A rough skeleton for an application looks like this:
//!
//! ```no_run
//! #![no_std]
//! #![no_main]
//!
//! // Pull in the panic handler from panic-halt
//! extern crate panic_halt;
//!
//! // The prelude just exports all HAL traits anonymously which makes
//! // all trait methods available.  This is probably something that
//! // should always be added.
//! use pypilot_controller_board::prelude::*;
//!
//! // Define the entry-point for the application.  This can only be
//! // done once in the entire dependency tree.
//! #[pypilot_controller_board::entry]
//! fn main() -> ! {
//!     // Get the peripheral singletons for interacting with them.
//!     let dp = pypilot_controller_board::Peripherals::take().unwrap();
//!
//!     unimplemented!()
//! }
//! ```
//!
//! [guide]: https://github.com/Rahix/avr-hal#starting-your-own-project
//!
//! [`examples/cph-adc.rs`]: https://github.com/Rahix/avr-hal/blob/master/boards/chart-plotter-hat/examples/uno-adc.rs
//!
//! [Arduino Uno]: https://store.arduino.cc/usa/chart-plotter-hat-rev3
//! [Arduino Nano]: https://store.arduino.cc/arduino-nano

#![no_std]

// Expose hal & pac crates
pub use crate::hal::pac;
pub use atmega328p_hal as hal;

/// See [`avr_device::entry`](https://docs.rs/avr-device/latest/avr_device/attr.entry.html).
#[cfg(feature = "rt")]
pub use crate::hal::entry;

pub use crate::pac::Peripherals;

mod pins;
pub use crate::pins::*;

pub mod prelude {
    pub use crate::hal::prelude::*;
    pub use crate::hal::usart::BaudrateArduinoExt as _;
}

/// Busy-Delay
///
/// **Note**: For just delaying, using [`pypilot_controller_board::delay_ms()`][delay_ms] or
/// [`pypilot_controller_board::delay_us()`][delay_us] is probably the better choice.  This type is more useful
/// when an `embedded-hal` driver needs a delay implementation.
///
/// [delay_ms]: fn.delay_ms.html
/// [delay_us]: fn.delay_us.html
pub type Delay = hal::delay::Delay<hal::clock::MHz8>;

/// Wait (busy spin) for `ms` milliseconds
pub fn delay_ms(ms: u16) {
    use prelude::*;

    Delay::new().delay_ms(ms)
}

/// Wait (busy spin) for `us` microseconds
pub fn delay_us(us: u16) {
    use prelude::*;

    Delay::new().delay_us(us)
}

/// Support for the Serial Peripheral Interface
///
/// # Example
/// For a full example, see [`examples/cph-spi-feedback.rs`][ex-spi].  In short:
/// ```no_run
/// let dp = pypilot_controller_board::Peripherals::take().unwrap();
///
/// let mut pins = pypilot_controller_board::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// // Create SPI interface.
/// let (mut spi, mut cs) = pypilot_controller_board::spi::Spi::new(
///     dp.SPI,
///     pins.d13.into_output(&mut pins.ddr),
///     pins.d11.into_output(&mut pins.ddr),
///     pins.d12.into_pull_up_input(&mut pins.ddr),
///     pins.d10.into_output(&mut pins.ddr),
///     pypilot_controller_board::spi::Settings::default(),
/// );
/// ```
///
/// [ex-spi]: https://github.com/Rahix/avr-hal/blob/master/boards/chart-plotter-hat/examples/cph-spi-feedback.rs
pub mod spi {
    pub use atmega328p_hal::spi::*;
}

/// Support for the Analog to Digital Converter
///
/// # Example
/// For a full example, see [`examples/cph-adc.rs`][ex-adc].  In short:
/// ```no_run
/// let dp = pypilot_controller_board::Peripherals::take().unwrap();
///
/// let mut pins = pypilot_controller_board::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// let mut adc = pypilot_controller_board::adc::Adc::new(dp.ADC, pypilot_controller_board::adc::AdcSettings::default());
///
/// // Convert pin to Analog input
/// let mut a0 = pins.a0.into_analog_input(&mut adc);
///
/// let aread: u16 = nb::block!{adc.read(&mut a0)}.void_unwrap();
/// ```
///
/// [ex-adc]: https://github.com/Rahix/avr-hal/blob/master/boards/chart-plotter-hat/examples/cph-adc.rs
pub mod adc {
    pub use atmega328p_hal::adc::*;
}

/// Support for PWM pins
///
/// The 3 timers of ATmega328P can be used for PWM on certain pins.
/// The PWM methods are from `embedded_hal::PwmPin`.
///
/// # Example
/// For a full example, see [`examples/uno-pwm.rs`][ex-pwm].  In short:
/// ```
/// let mut pins = arduino_uno::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// let mut timer1 = arduino_uno::pwm::Timer1Pwm::new(
///     dp.TC1,
///     arduino_uno::pwm::Prescaler::Prescale64,
/// );
///
/// let mut pin = pins.d9.into_output(&mut pins.ddr).into_pwm(&mut timer1);
///
/// pin.set_duty(128);
/// pin.enable();
/// ```
///
/// Here is an overview of pins and which timer they work with:
///
/// | Pin | Conversion Method |
/// | --- | --- |
/// | `pins.d3` | `.into_pwm(&mut timer2)` |
/// | `pins.d5` | `.into_pwm(&mut timer0)` |
/// | `pins.d6` | `.into_pwm(&mut timer0)` |
/// | `pins.d9` | `.into_pwm(&mut timer1)` |
/// | `pins.d10` | `.into_pwm(&mut timer1)` |
/// | `pins.d11` | `.into_pwm(&mut timer2)` |
///
/// [ex-pwm]: https://github.com/Rahix/avr-hal/blob/master/boards/arduino-uno/examples/uno-pwm.rs
pub mod pwm {
    pub use atmega328p_hal::pwm::*;
}

/// Serial (UART) interface on pins `D0` (RX) and `D1` (TX)
///
/// # Example
/// For a full example, see [`examples/leonardo-serial.rs`][ex-serial].  In short:
/// ```no_run
/// let dp = pypilot_controller_board::Peripherals::take().unwrap();
///
/// let mut pins = pypilot_controller_board::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// let mut serial = pypilot_controller_board::Serial::new(
///     dp.USART0,
///     pins.d0,
///     pins.d1.into_output(&mut pins.ddr),
///     57600.into_baudrate(),
/// );
///
/// ufmt::uwriteln!(&mut serial, "Hello from Arduino!\r").void_unwrap();
/// ```
///
/// [ex-serial]: https://github.com/Rahix/avr-hal/blob/master/boards/chart-plotter-hat/examples/cph-serial.rs
pub type Serial<IMODE> = hal::usart::Usart0<hal::clock::MHz16, IMODE>;

/// Support for the WatchDog Timer
///
/// # Note
/// Changing the watchdog configuration requires two separate writes to WDTCSR where the second
/// write must occur within 4 cycles of the first or the configuration will not change. You may need
/// to adjust optimization settings to prevent other operations from being emitted between these two
/// writes.
///
/// # Example
/// ```
/// let mut watchdog = pypilot_controller_board::wdt::Wdt::new(&dp.CPU.mcusr, dp.WDT);
/// watchdog.start(pypilot_controller_board::wdt::Timeout::Ms8000);
///
/// loop {
///     watchdog.feed();
/// }
/// ```
pub mod wdt {
    pub use atmega328p_hal::wdt::*;
}
