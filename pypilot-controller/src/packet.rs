use crate::{
    AdcChannel, CommandToExecute, State, ADC_RESULTS, BADVOLTAGEFAULT, LOWCURRENT, MAXRUDDERFAULT,
    MINRUDDERFAULT, OVERCURRENTFAULT, OVERTEMPFAULT, PORTPINFAULT, REBOOTED, STBDPINFAULT, SYNC,
};
use avr_device::interrupt;
use core::convert::TryFrom;

//==========================================================

/// type of packet
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
    // output codes
    CurrentCode,
    VoltageCode,
    ControllerTempCode,
    MotorTempCode,
    RudderSenseCode,
    FlagsCode,
    EEPROMValueCode,
    InvalidCode,
    // debugging codes
    #[cfg(debug_assertions)]
    CurrentFSM,
    #[cfg(debug_assertions)]
    PreviousFSM,
}

/// convert to u8
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
            PacketType::CurrentCode => 0x1c,
            PacketType::VoltageCode => 0xb3,
            PacketType::ControllerTempCode => 0xf9,
            PacketType::MotorTempCode => 0x48,
            PacketType::RudderSenseCode => 0xa7,
            PacketType::FlagsCode => 0x8f,
            PacketType::EEPROMValueCode => 0x9a,
            PacketType::InvalidCode => 0x00,
            #[cfg(debug_assertions)]
            PacketType::CurrentFSM => 0x20,
            #[cfg(debug_assertions)]
            PacketType::PreviousFSM => 0x21,
        }
    }
}

pub enum ParseError {
    InvalidPacketType(u8),
}

/// convert from u8
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
            0x1c => Ok(PacketType::CurrentCode),
            0xb3 => Ok(PacketType::VoltageCode),
            0xf9 => Ok(PacketType::ControllerTempCode),
            0x48 => Ok(PacketType::MotorTempCode),
            0xa7 => Ok(PacketType::RudderSenseCode),
            0x8f => Ok(PacketType::FlagsCode),
            0x9a => Ok(PacketType::EEPROMValueCode),
            #[cfg(debug_assertions)]
            0x20 => Ok(PacketType::CurrentFSM),
            #[cfg(debug_assertions)]
            0x21 => Ok(PacketType::PreviousFSM),
            n => Err(ParseError::InvalidPacketType(n)),
        }
    }
}

//==========================================================

#[cfg(feature = "motor_temp")]
fn build_motor_temp_pkt() -> (bool, u16, PacketType) {
    interrupt::free(|_cs| {
        if ADC_RESULTS.count(AdcChannel::MotorTemp, 0) > 0 {
            (
                true,
                ADC_RESULTS.take_motor_temp(0),
                PacketType::MotorTempCode,
            )
        } else {
            (false, 0, PacketType::InvalidCode)
        }
    })
}

#[cfg(not(feature = "motor_temp"))]
fn build_motor_temp_pkt() -> (bool, u16, PacketType) {
    (false, 0, PacketType::InvalidCode)
}

//==========================================================

#[cfg(feature = "controller_temp")]
fn build_controller_temp_pkt() -> (bool, u16, PacketType) {
    interrupt::free(|_cs| {
        if ADC_RESULTS.count(AdcChannel::ControllerTemp, 0) > 0 {
            (
                true,
                ADC_RESULTS.take_controller_temp(0),
                PacketType::ControllerTempCode,
            )
        } else {
            (false, 0, PacketType::InvalidCode)
        }
    })
}

#[cfg(not(feature = "controller_temp"))]
fn build_controller_temp_pkt() -> (bool, u16, PacketType) {
    (false, 0, PacketType::InvalidCode)
}

//==========================================================

#[cfg(feature = "rudder_angle")]
fn build_rudder_pkt() -> (bool, u16, PacketType) {
    interrupt::free(|_cs| {
        if ADC_RESULTS.count(AdcChannel::Rudder, 0) < 10 {
            (false, 0, PacketType::InvalidCode)
        } else {
            (true, ADC_RESULTS.take_rudder(0), RUDDER_SENSE_CODE)
        }
    })
}

#[cfg(not(feature = "rudder_angle"))]
fn build_rudder_pkt() -> (bool, u16, PacketType) {
    (false, 0, PacketType::InvalidCode)
}

//==========================================================

/// handle a received packet
pub fn process_packet(pkt: [u8; 3], mut state: State) -> State {
    state.flags |= SYNC;
    let mut val: u16 = pkt[1] as u16 | ((pkt[2] as u16) << 8);
    match PacketType::try_from(pkt[0]) {
        Ok(ty) => match ty {
            PacketType::CommandCode => {
                state.timeout = 0;
                // check for valid range and none of these faults
                if val <= 2000
                    && (state.flags & (OVERTEMPFAULT | OVERCURRENTFAULT | BADVOLTAGEFAULT)) == 0
                {
                    if (state.flags & (PORTPINFAULT | MAXRUDDERFAULT)) != 0
                        || ((state.flags & (STBDPINFAULT | MINRUDDERFAULT)) != 0 && val < 1000)
                    {
                        state.pending_cmd = CommandToExecute::Stop;
                    } else {
                        state.command_value = val;
                        state.pending_cmd = CommandToExecute::Engage;
                    }
                }
            }
            PacketType::MaxCurrentCode => {
                let max_max_current = if LOWCURRENT { 2000 } else { 5000 };
                if val > max_max_current {
                    val = max_max_current;
                }
                state.max_current = val;
            }
            PacketType::MaxControllerTempCode => {
                if val > 10000 {
                    val = 10000;
                }
                state.max_controller_temp = val;
            }
            PacketType::MaxMotorTempCode => {
                if val > 10000 {
                    val = 10000;
                }
                state.max_motor_temp = val;
            }
            PacketType::RudderMinCode => state.rudder_min = val,
            PacketType::RudderMaxCode => state.rudder_max = val,
            PacketType::DisengageCode => state.pending_cmd = CommandToExecute::Disengage,
            PacketType::MaxSlewCode => {
                state.max_slew_speed = pkt[1];
                state.max_slew_slow = pkt[2];
                // if set at the end of the rage (up to 255) no slew limit
                if state.max_slew_speed > 250 {
                    state.max_slew_speed = 250;
                }
                if state.max_slew_slow > 250 {
                    state.max_slew_slow = 250;
                }
                // must have some slew
                if state.max_slew_speed == 0 {
                    state.max_slew_speed = 1;
                }
                if state.max_slew_slow == 0 {
                    state.max_slew_slow = 1;
                }
            }
            PacketType::EEPROMReadCode => {}
            PacketType::EEPROMWriteCode => {}
            PacketType::ReprogramCode => {}
            PacketType::ResetCode => state.flags &= !OVERCURRENTFAULT,
            PacketType::RudderRangeCode => {}
            _ => {}
        },
        Err(_) => {}
    }
    State {
        cpu: state.cpu,
        exint: state.exint,
        timer0: state.timer0,
        timer2: state.timer2,
        ena_pin: state.ena_pin,
        enb_pin: state.enb_pin,
        ina_pin: state.ina_pin,
        inb_pin: state.inb_pin,
        pwm_pin: state.pwm_pin,
        adc: state.adc,
        machine_state: state.machine_state,
        prev_state: state.prev_state,
        timeout: state.timeout,
        comm_timeout: state.comm_timeout,
        command_value: state.command_value,
        lastpos: state.lastpos,
        max_slew_speed: state.max_slew_speed,
        max_slew_slow: state.max_slew_slow,
        max_voltage: state.max_voltage,
        max_current: state.max_current,
        max_controller_temp: state.max_controller_temp,
        max_motor_temp: state.max_motor_temp,
        rudder_min: state.rudder_min,
        rudder_max: state.rudder_max,
        flags: state.flags,
        pending_cmd: state.pending_cmd,
        outgoing_state: state.outgoing_state,
    }
}

//==========================================================

/// buffer to hold outgoing packet
pub static mut OUT_BYTES: [u8; 3] = [0; 3];

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum OutgoingPacketState {
    None,
    Build,
    Start,
    Byte0,
    Byte1,
    Byte2,
    #[cfg(not(feature = "serial_packets"))]
    Sent,
}

/// position of outgoing packet already sent
pub static mut OUT_SYNC_STATE: OutgoingPacketState = OutgoingPacketState::None;

/// build a new packet to send
pub fn build_outgoing(mut state: State) -> State {
    // which position in the cycle
    static mut OUT_SYNC_POS: u8 = 0;

    // build outgoing packet and post for sending if required
    // unsafe is ok, because OUT_SYNC_POS and OUT_BYTES only used outside of interrupts
    // ADC_RESULTS is used in isr, so it needs wrapped in interrupt::free context
    unsafe {
        let (use_it, v, code) = match OUT_SYNC_POS {
            0 | 10 | 20 | 30 => {
                state.flags &= !REBOOTED;
                (true, state.flags, PacketType::FlagsCode)
            }
            1 | 4 | 7 | 11 | 14 | 17 | 21 | 24 | 27 | 31 | 34 | 37 | 40 => interrupt::free(|_cs| {
                if ADC_RESULTS.count(AdcChannel::Current, 0) < 50 {
                    (false, 0, PacketType::InvalidCode)
                } else {
                    (true, ADC_RESULTS.take_amps(0), PacketType::CurrentCode)
                }
            }),
            3 | 13 | 23 | 33 => interrupt::free(|_cs| {
                if ADC_RESULTS.count(AdcChannel::Voltage, 0) < 2 {
                    (false, 0, PacketType::InvalidCode)
                } else {
                    (true, ADC_RESULTS.take_volts(0), PacketType::VoltageCode)
                }
            }),
            2 | 5 | 8 | 12 | 15 | 18 | 22 | 25 | 28 | 32 | 35 | 38 | 41 => build_rudder_pkt(),
            6 => build_controller_temp_pkt(),
            9 => build_motor_temp_pkt(),
            16 | 26 | 36 => {
                // eeprom reads
                (false, 0, PacketType::InvalidCode)
            }
            // catch all the undefined ranges
            _ => (false, 0, PacketType::InvalidCode),
        };
        // check if outgoing packet is ready
        if use_it {
            OUT_BYTES[0] = code.into();
            OUT_BYTES[1] = (v & 0xFF) as u8;
            OUT_BYTES[2] = ((v & 0xFF00) >> 8) as u8;
            OUT_SYNC_STATE = OutgoingPacketState::Start;
        }
        if OUT_SYNC_POS < 41 {
            OUT_SYNC_POS += 1;
        } else {
            OUT_SYNC_POS = 0;
        }
    }
    State {
        cpu: state.cpu,
        exint: state.exint,
        timer0: state.timer0,
        timer2: state.timer2,
        ena_pin: state.ena_pin,
        enb_pin: state.enb_pin,
        ina_pin: state.ina_pin,
        inb_pin: state.inb_pin,
        pwm_pin: state.pwm_pin,
        adc: state.adc,
        machine_state: state.machine_state,
        prev_state: state.prev_state,
        timeout: state.timeout,
        comm_timeout: state.comm_timeout,
        command_value: state.command_value,
        lastpos: state.lastpos,
        max_slew_speed: state.max_slew_speed,
        max_slew_slow: state.max_slew_slow,
        max_voltage: state.max_voltage,
        max_current: state.max_current,
        max_controller_temp: state.max_controller_temp,
        max_motor_temp: state.max_motor_temp,
        rudder_min: state.rudder_min,
        rudder_max: state.rudder_max,
        flags: state.flags,
        pending_cmd: state.pending_cmd,
        outgoing_state: state.outgoing_state,
    }
}
