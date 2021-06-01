use crate::{
    CommandToExecute, Hardware, BADVOLTAGEFAULT, INVALID, LOWCURRENT, MAXRUDDERFAULT,
    MINRUDDERFAULT, OVERCURRENTFAULT, OVERTEMPFAULT, PORTPINFAULT, STBDPINFAULT, SYNC,
};
use core::convert::TryFrom;
use pypilot_controller_board::prelude::*;
//use ufmt;

//==========================================================

/// type of serial packet
/// has the raw numeric value of it's representation
#[derive(Debug, Copy, Clone)]
pub enum PacketType {
    ReprogramCode,
    ResetCode,
    CommandCode,
    MaxCurrentCode,
    MaxControllerTempCode,
    MaxMotorTempCode,
    RudderRangeCode,
    RudderMinCode,
    RudderMaxCode,
    DisengageCode,
    MaxSlewCode,
    EEPROMReadCode,
    EEPROMWriteCode,
}

impl From<PacketType> for u8 {
    fn from(original: PacketType) -> u8 {
        match original {
            PacketType::ReprogramCode => 0x19,
            PacketType::ResetCode => 0xe7,
            PacketType::CommandCode => 0xc7,
            PacketType::MaxCurrentCode => 0x1e,
            PacketType::MaxControllerTempCode => 0xa4,
            PacketType::MaxMotorTempCode => 0x5a,
            PacketType::RudderRangeCode => 0xb6,
            PacketType::RudderMinCode => 0x2b,
            PacketType::RudderMaxCode => 0x4d,
            PacketType::DisengageCode => 0x68,
            PacketType::MaxSlewCode => 0x71,
            PacketType::EEPROMReadCode => 0x91,
            PacketType::EEPROMWriteCode => 0x53,
        }
    }
}

pub enum ParseError {
    InvalidPacketType(u8),
}

impl TryFrom<u8> for PacketType {
    type Error = ParseError;

    fn try_from(original: u8) -> Result<Self, Self::Error> {
        match original {
            0x19 => Ok(PacketType::ReprogramCode),
            0xe7 => Ok(PacketType::ResetCode),
            0xc7 => Ok(PacketType::CommandCode),
            0x1e => Ok(PacketType::MaxCurrentCode),
            0xa4 => Ok(PacketType::MaxControllerTempCode),
            0x5a => Ok(PacketType::MaxMotorTempCode),
            0xb6 => Ok(PacketType::RudderRangeCode),
            0x2b => Ok(PacketType::RudderMinCode),
            0x4d => Ok(PacketType::RudderMaxCode),
            0x68 => Ok(PacketType::DisengageCode),
            0x71 => Ok(PacketType::MaxSlewCode),
            0x91 => Ok(PacketType::EEPROMReadCode),
            0x53 => Ok(PacketType::EEPROMWriteCode),
            n => Err(ParseError::InvalidPacketType(n)),
        }
    }
}

//==========================================================

fn crc8(_array: [u8; 3]) -> u8 {
    0
}

//==========================================================

pub fn process_packet(pkt: [u8; 3], mut hdwr: Hardware) -> Hardware {
    hdwr.flags |= SYNC;
    let mut val: u16 = pkt[1] as u16 | ((pkt[2] as u16) << 8);
    let pkt_type = PacketType::try_from(pkt[0]);
    match pkt_type {
        Ok(ty) => match ty {
            PacketType::CommandCode => {
                hdwr.timeout = 0;
                if hdwr.serialin < 12 {
                    hdwr.serialin += 4; // output at input rate
                }
                // check for valid range and none of these faults
                if val <= 2000
                    && (hdwr.flags & (OVERTEMPFAULT | OVERCURRENTFAULT | BADVOLTAGEFAULT)) == 0
                {
                    if (hdwr.flags & (PORTPINFAULT | MAXRUDDERFAULT)) != 0
                        || ((hdwr.flags & (STBDPINFAULT | MINRUDDERFAULT)) != 0 && val < 1000)
                    {
                        hdwr.pending_cmd = CommandToExecute::Stop;
                    } else {
                        hdwr.command_value = val;
                        hdwr.pending_cmd = CommandToExecute::Engage;
                    }
                }
            }
            PacketType::MaxCurrentCode => {
                let max_max_current = if LOWCURRENT { 2000 } else { 5000 };
                if val > max_max_current {
                    val = max_max_current;
                }
                hdwr.max_current = val;
            }
            PacketType::MaxControllerTempCode => {
                if val > 10000 {
                    val = 10000;
                }
                hdwr.max_controller_temp = val;
            }
            PacketType::MaxMotorTempCode => {
                if val > 10000 {
                    val = 10000;
                }
                hdwr.max_motor_temp = val;
            }
            PacketType::RudderMinCode => hdwr.rudder_min = val,
            PacketType::RudderMaxCode => hdwr.rudder_max = val,
            PacketType::DisengageCode => {
                if hdwr.serialin < 12 {
                    hdwr.serialin += 4; // output at input rate
                }
                hdwr.pending_cmd = CommandToExecute::Disengage;
            }
            PacketType::MaxSlewCode => {
                hdwr.max_slew_speed = pkt[1];
                hdwr.max_slew_speed = pkt[2];
                // if set at the end of the rage (up to 255) no slew limit
                if hdwr.max_slew_speed > 250 {
                    hdwr.max_slew_speed = 250;
                }
                if hdwr.max_slew_slow > 250 {
                    hdwr.max_slew_slow = 250;
                }
                // must have some slew
                if hdwr.max_slew_speed == 0 {
                    hdwr.max_slew_speed = 1;
                }
                if hdwr.max_slew_slow == 0 {
                    hdwr.max_slew_slow = 1;
                }
            }
            PacketType::EEPROMReadCode => {}
            PacketType::EEPROMWriteCode => {}
            PacketType::ReprogramCode => {}
            PacketType::ResetCode => {}
            PacketType::RudderRangeCode => {}
        },
        Err(_) => {}
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
