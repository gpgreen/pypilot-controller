//! This example demonstrates how to set up a SPI interface and communicate
//! over it.  The physical hardware configuation consists of connecting a
//! jumper directly from pin `~11` to pin `~12`.
//!
//! Once this program is written to the board, the serial output can be
//! accessed with
//!
//! ```
//! sudo screen /dev/ttyACM0 57600
//! ```
//!
//! You should see it output the line `data: 15` repeatedly (aka 0b00001111).
//! If the output you see is `data: 255`, you may need to check your jumper.

#![no_std]
#![no_main]

use panic_halt as _;
use pypilot_controller_board::prelude::*;
use pypilot_controller_board::spi;

#[pypilot_controller_board::entry]
fn main() -> ! {
    let dp = pypilot_controller_board::Peripherals::take().unwrap();

    let mut pins = pypilot_controller_board::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
    // set up serial interface for text output
    let mut serial = pypilot_controller_board::Serial::new(
        dp.USART0,
        pins.d0,
        pins.d1.into_output(&mut pins.ddr),
        57600.into_baudrate(),
    );

    // Create SPI interface.
    let (mut spi, _) = spi::Spi::new(
        dp.SPI,
        pins.d13.into_output(&mut pins.ddr),
        pins.d11.into_output(&mut pins.ddr),
        pins.d12.into_pull_up_input(&mut pins.ddr),
        pins.d10.into_output(&mut pins.ddr),
        spi::Settings::default(),
    );

    loop {
        // Send a byte
        nb::block!(spi.send(0b00001111)).void_unwrap();
        // Because MISO is connected to MOSI, the read data should be the same
        let data = nb::block!(spi.read()).void_unwrap();

        ufmt::uwriteln!(&mut serial, "data: {}\r", data).void_unwrap();
        pypilot_controller_board::delay_ms(1000);
    }
}
