use crate::crc::crc8;
use crate::packet::{OutgoingPacketState, OUT_BYTES, OUT_SYNC_STATE};
use crate::{CommandToExecute, Hardware, INVALID, SYNC};
use pypilot_controller_board::prelude::*;

//==========================================================

/// check for incoming bytes, convert to packets if possible
pub fn process_serial(mut hdwr: Hardware) -> Hardware {
    static mut IN_BYTES: [u8; 3] = [0; 3];
    static mut SYNC_B: usize = 0;
    static mut IN_SYNC_COUNT: u8 = 0;

    let mut do_more = true;
    // is there serial input available?
    while do_more {
        let word = match hdwr.serial.borrow_mut().read() {
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
                            if OUT_SYNC_STATE == OutgoingPacketState::None {
                                OUT_SYNC_STATE = OutgoingPacketState::Build;
                            }
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
                do_more = true;
                word
            }
            Err(_) => {
                do_more = false;
                0
            }
        };
        if word != 0 {
            hdwr.serial.borrow_mut().write_byte(word);
        }
    }

    // process outgoing packet if necessary
    unsafe {
        OUT_SYNC_STATE = match OUT_SYNC_STATE {
            OutgoingPacketState::None => OutgoingPacketState::None,
            OutgoingPacketState::Build => OutgoingPacketState::Build,
            OutgoingPacketState::Start => {
                hdwr.serial.borrow_mut().write_byte(OUT_BYTES[0]);
                OutgoingPacketState::Byte0
            }
            OutgoingPacketState::Byte0 => {
                hdwr.serial.borrow_mut().write_byte(OUT_BYTES[1]);
                OutgoingPacketState::Byte1
            }
            OutgoingPacketState::Byte1 => {
                hdwr.serial.borrow_mut().write_byte(OUT_BYTES[2]);
                OutgoingPacketState::Byte2
            }
            OutgoingPacketState::Byte2 => {
                hdwr.serial.borrow_mut().write_byte(crc8(OUT_BYTES));
                OutgoingPacketState::None
            }
            #[cfg(not(feature = "serial_packets"))]
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
        flags: hdwr.flags,
        pending_cmd: hdwr.pending_cmd,
    }
}

//==========================================================
