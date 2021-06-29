use crate::State;
use avr_device::interrupt;

//==========================================================

fn _eeprom_read(state: &State, address: u16) -> u8 {
    unsafe {
        interrupt::free(|_| {
            // poll until write op finished
            while (state.eeprom.eecr.read().bits() & 0x2) == 0x2 {}
            // set address
            state.eeprom.eear.write(|w| w.bits(address));
            // flag a read
            state.eeprom.eecr.modify(|r, w| w.bits(r.bits() | 0x1));
            state.eeprom.eedr.read().bits()
        })
    }
}

//==========================================================

fn _eeprom_write(state: &State, address: u16, val: u8) {
    unsafe {
        interrupt::free(|_| {
            // wait for completion of previous write
            while (state.eeprom.eecr.read().bits() & 0x2) == 0x2 {}
            // wait for CPU write to flash memory
            while (state.cpu.spmcsr.read().bits() & 0x1) == 0x1 {}
            // set address
            state.eeprom.eear.write(|w| w.bits(address));
            // write data to data register
            state.eeprom.eedr.write(|w| w.bits(val));
            // write a '1' to the EEMPE bit while writing a zero to EEPE in EECR
            state.eeprom.eecr.write(|w| w.bits(0x4));
            // within 4 cycles, write a '1' to EEPE
            state.eeprom.eecr.modify(|r, w| w.bits(r.bits() | 0x2));
        });
    }
}

//==========================================================

pub fn eeprom_read8(state: &State, address: u16) -> u8 {
    let mut v: [u8; 3] = [0; 3];
    // use 3 banks
    for i in 0..3 {
        let addr = i * 256 + address;
        v[i as usize] = _eeprom_read(state, addr);
    }
    if v[1] == v[2] {
        v[2]
    } else {
        v[0]
    }
}

//==========================================================

pub fn eeprom_update8(state: &State, address: u16, val: u8) {
    // write in 3 locations
    for i in 0..3 {
        let addr = i * 256 + address;
        _eeprom_write(state, addr, val);
    }
}

//==========================================================
