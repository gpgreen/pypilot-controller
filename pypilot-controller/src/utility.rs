//! utility functions for debug builds

#[cfg(debug_assertions)]
use hal::port::mode::Floating;
#[cfg(debug_assertions)]
use pypilot_controller_board::hal;
#[cfg(debug_assertions)]
use pypilot_controller_board::prelude::*;
#[cfg(debug_assertions)]
use ufmt;

#[cfg(debug_assertions)]
const HEX_CHARS: [char; 16] = [
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
];

#[cfg(debug_assertions)]
pub fn send_hex_byte(serial: &mut pypilot_controller_board::Serial<Floating>, b: u8) {
    ufmt::uwrite!(
        serial,
        "{}{}",
        HEX_CHARS[((b & 0xF0) >> 4) as usize],
        HEX_CHARS[(b & 0xF) as usize]
    )
    .ok();
}

#[cfg(debug_assertions)]
pub fn send_reg(serial: &mut pypilot_controller_board::Serial<Floating>, addr: u8) {
    let ptr = unsafe { &mut *(addr as *mut u8) };
    let b = unsafe { core::ptr::read_volatile(ptr) };
    send_hex_byte(serial, b);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}

#[cfg(debug_assertions)]
pub fn send_u16(serial: &mut pypilot_controller_board::Serial<Floating>, n: u16) {
    send_hex_byte(serial, ((n & 0xFF00) >> 8) as u8);
    send_hex_byte(serial, (n & 0xFF) as u8);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}
