#![no_std]
#![no_main]

use panic_halt as _;
use pypilot_controller_board::prelude::*;

#[pypilot_controller_board::entry]
fn main() -> ! {
    let dp = pypilot_controller_board::Peripherals::take().unwrap();

    let mut pins = pypilot_controller_board::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);

    // Digital pin 13 is also connected to an onboard LED marked "L"
    let mut led = pins.d13.into_output(&mut pins.ddr);

    led.set_high().void_unwrap();

    loop {
        led.toggle().void_unwrap();
        pypilot_controller_board::delay_ms(200);
        led.toggle().void_unwrap();
        pypilot_controller_board::delay_ms(200);
        led.toggle().void_unwrap();
        pypilot_controller_board::delay_ms(200);
        led.toggle().void_unwrap();
        pypilot_controller_board::delay_ms(800);
    }
}
