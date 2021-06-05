use crate::crc::crc8;
use crate::packet::{OutgoingPacketState, OUT_BYTES, OUT_SYNC_STATE};
use crate::{CommandToExecute, Hardware, INVALID, SYNC};
use pypilot_controller_board::prelude::*;
//use ufmt;

//==========================================================

/// check for incoming bytes, convert to packets if possible
pub fn process_serial(mut hdwr: Hardware) -> Hardware {
    static mut IN_BYTES: [u8; 3] = [0; 3];
    static mut SYNC_B: usize = 0;
    static mut IN_SYNC_COUNT: u8 = 0;

    // is there serial input available?
    match hdwr.serial.borrow_mut().read() {
        Ok(word) => {
            // reset the serial timeout now, since we've received data
            hdwr.serial_data_timeout = 0;
            // this is safe because none of these static structures are modified
            // elsewhere, they are only used here in this function
            unsafe {
                if SYNC_B < 3 {
                    IN_BYTES[SYNC_B] = word;
                    SYNC_B += 1;
                } else {
                    if word == crc8(IN_BYTES) {
                        if IN_SYNC_COUNT >= 2 {
                            hdwr.pending_cmd = CommandToExecute::ProcessPacket(IN_BYTES);
                        } else {
                            IN_SYNC_COUNT += 1;
                        }
                        SYNC_B = 0;
                        hdwr.flags &= !INVALID;
                    } else {
                        // invalid packet
                        hdwr.flags &= !SYNC;
                        hdwr.pending_cmd = CommandToExecute::Stop;
                        IN_SYNC_COUNT = 0;
                        IN_BYTES[0] = IN_BYTES[1];
                        IN_BYTES[1] = IN_BYTES[2];
                        IN_BYTES[2] = word;
                        hdwr.flags |= INVALID;
                    }
                }
            };
        }
        Err(_) => {}
    }
    // process outgoing packet if necessary
    unsafe {
        OUT_SYNC_STATE = match OUT_SYNC_STATE {
            OutgoingPacketState::None => {
                if hdwr.serialin >= 4 {
                    OutgoingPacketState::Build
                } else {
                    OutgoingPacketState::None
                }
            }
            OutgoingPacketState::Build => OutgoingPacketState::Build,
            OutgoingPacketState::Start => match hdwr.serial.borrow_mut().write(OUT_BYTES[0]) {
                Ok(_) => OutgoingPacketState::Byte0,
                Err(_) => OutgoingPacketState::None,
            },
            OutgoingPacketState::Byte0 => match hdwr.serial.borrow_mut().write(OUT_BYTES[1]) {
                Ok(_) => OutgoingPacketState::Byte1,
                Err(_) => OutgoingPacketState::None,
            },
            OutgoingPacketState::Byte1 => match hdwr.serial.borrow_mut().write(OUT_BYTES[2]) {
                Ok(_) => OutgoingPacketState::Byte2,
                Err(_) => OutgoingPacketState::None,
            },
            OutgoingPacketState::Byte2 => match hdwr.serial.borrow_mut().write(crc8(OUT_BYTES)) {
                Ok(_) => OutgoingPacketState::None,
                Err(_) => OutgoingPacketState::None,
            },
            // for serial, this will never happen
            OutgoingPacketState::Sent => OutgoingPacketState::None,
        };
    }
    Hardware {
        cpu: hdwr.cpu,
        exint: hdwr.exint,
        timer0: hdwr.timer0,
        timer2: hdwr.timer2,
        ena_pin: hdwr.ena_pin,
        enb_pin: hdwr.enb_pin,
        ina_pin: hdwr.ina_pin,
        inb_pin: hdwr.inb_pin,
        pwm_pin: hdwr.pwm_pin,
        adc: hdwr.adc,
        a1: hdwr.a1,
        #[cfg(debug_assertions)]
        serial: hdwr.serial,
        machine_state: hdwr.machine_state,
        prev_state: hdwr.prev_state,
        timeout: hdwr.timeout,
        serial_data_timeout: hdwr.serial_data_timeout,
        command_value: hdwr.command_value,
        lastpos: hdwr.lastpos,
        max_slew_speed: hdwr.max_slew_speed,
        max_slew_slow: hdwr.max_slew_slow,
        max_voltage: hdwr.max_voltage,
        max_current: hdwr.max_current,
        max_controller_temp: hdwr.max_controller_temp,
        max_motor_temp: hdwr.max_motor_temp,
        rudder_min: hdwr.rudder_min,
        rudder_max: hdwr.rudder_max,
        serialin: hdwr.serialin,
        flags: hdwr.flags,
        pending_cmd: hdwr.pending_cmd,
    }
}

//==========================================================
