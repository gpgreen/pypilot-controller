#![no_std]
#![no_main]

use panic_halt as _;
use pypilot_controller_board::prelude::*;
use pypilot_controller_board::wdt;

#[pypilot_controller_board::entry]
fn main() -> ! {
    let dp = pypilot_controller_board::Peripherals::take().unwrap();

    let mut watchdog = wdt::Wdt::new(&dp.CPU.mcusr, dp.WDT);
    watchdog.start(wdt::Timeout::Ms8000);

    loop {
        watchdog.feed();
    }
}
