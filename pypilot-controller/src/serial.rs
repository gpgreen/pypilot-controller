use crate::crc;
use crate::packet::{OutgoingPacketState, PacketType, OUT_BYTES};
use crate::{CommandToExecute, State, INVALID, SYNC};
use avr_device::interrupt;
use avr_device::interrupt::Mutex;
use core::cell::RefCell;
use hal::port::mode::{Floating, Input, Output};
use heapless::spsc::{Consumer, Producer, Queue};
use nb;
use pypilot_controller_board::hal;
use pypilot_controller_board::prelude::*;

type SerialWriter = pypilot_controller_board::hal::usart::UsartWriter<
    pypilot_controller_board::pac::USART0,
    hal::port::portd::PD0<Input<Floating>>,
    hal::port::portd::PD1<Output>,
    hal::clock::MHz8,
>;
type SerialReader = pypilot_controller_board::hal::usart::UsartReader<
    pypilot_controller_board::pac::USART0,
    hal::port::portd::PD0<Input<Floating>>,
    hal::port::portd::PD1<Output>,
    hal::clock::MHz8,
>;
//==========================================================

// static variables for moving Producer, Consumer into their functions

/// used in interrupt function
static mut PROD_QUEUE: Mutex<RefCell<Option<Producer<u8, 64>>>> = Mutex::new(RefCell::new(None));

/// used in process_serial
static mut CONS_QUEUE: Mutex<RefCell<Option<Consumer<u8, 64>>>> = Mutex::new(RefCell::new(None));

/// used in process_serial and send_tuple
static USARTWRITER: Mutex<RefCell<Option<SerialWriter>>> = Mutex::new(RefCell::new(None));

/// used in isr
static mut USARTREADER: Mutex<RefCell<Option<SerialReader>>> = Mutex::new(RefCell::new(None));

//==========================================================

/// setup the queues, move them into the static variables so they can be moved into the fns
pub fn setup_serial(reader: SerialReader, writer: SerialWriter) {
    static mut QUEUE: Queue<u8, 64> = Queue::new();

    // SAFETY: the queues are in a interrupt::free context so access is safe
    unsafe {
        let (wr, rd) = QUEUE.split();
        interrupt::free(|cs| {
            CONS_QUEUE.borrow(cs).replace(Some(rd));
            PROD_QUEUE.borrow(cs).replace(Some(wr));
            USARTWRITER.borrow(cs).replace(Some(writer));
            USARTREADER.borrow(cs).replace(Some(reader));
        });
    }
}

fn get_next_byte() -> Option<u8> {
    unsafe {
        interrupt::free(|cs| match CONS_QUEUE.borrow(cs).borrow_mut().as_mut() {
            Some(q) => q.dequeue(),
            None => None,
        })
    }
}

/// check for incoming bytes, convert to packets if possible
pub fn process_serial(mut state: State) -> State {
    static mut IN_BYTES: [u8; 3] = [0; 3];
    static mut SYNC_B: usize = 0;
    static mut IN_SYNC_COUNT: u8 = 0;

    let mut do_more = true;
    while do_more {
        match get_next_byte() {
            Some(word) => {
                // reset the serial timeout now, since we've received data
                state.comm_timeout = 0;
                // this is safe because none of these static structures are modified
                // elsewhere, they are only used here in this function
                unsafe {
                    if SYNC_B < 3 {
                        IN_BYTES[SYNC_B] = word;
                        SYNC_B += 1;
                    } else {
                        if word == crc::crc8(IN_BYTES) {
                            if IN_SYNC_COUNT >= 2 {
                                state.pending_cmd = CommandToExecute::ProcessPacket(IN_BYTES);
                            } else {
                                IN_SYNC_COUNT += 1;
                            }
                            SYNC_B = 0;
                            state.flags &= !INVALID;
                            if state.outgoing_state == OutgoingPacketState::None {
                                state.outgoing_state = OutgoingPacketState::Build;
                            }
                        } else {
                            // invalid packet
                            state.flags &= !SYNC;
                            state.pending_cmd = CommandToExecute::Stop;
                            IN_SYNC_COUNT = 0;
                            IN_BYTES[0] = IN_BYTES[1];
                            IN_BYTES[1] = IN_BYTES[2];
                            IN_BYTES[2] = word;
                            state.flags |= INVALID;
                        }
                    };
                }
                do_more = true;
            }
            None => do_more = false,
        }
    }

    // process outgoing packet if necessary
    unsafe {
        interrupt::free(|cs| {
            if let Some(wr) = USARTWRITER.borrow(cs).borrow_mut().as_mut() {
                state.outgoing_state = match state.outgoing_state {
                    OutgoingPacketState::None => OutgoingPacketState::None,
                    OutgoingPacketState::Build => OutgoingPacketState::Build,
                    OutgoingPacketState::Start => match wr.write(OUT_BYTES[0]) {
                        Ok(()) => OutgoingPacketState::Byte0,
                        Err(_) => OutgoingPacketState::Start,
                    },
                    OutgoingPacketState::Byte0 => match wr.write(OUT_BYTES[1]) {
                        Ok(()) => OutgoingPacketState::Byte1,
                        Err(_) => OutgoingPacketState::Byte0,
                    },
                    OutgoingPacketState::Byte1 => match wr.write(OUT_BYTES[2]) {
                        Ok(()) => OutgoingPacketState::Byte2,
                        Err(_) => OutgoingPacketState::Byte1,
                    },
                    OutgoingPacketState::Byte2 => match wr.write(crc::crc8(OUT_BYTES)) {
                        Ok(()) => OutgoingPacketState::None,
                        Err(_) => OutgoingPacketState::Byte2,
                    },
                    #[cfg(not(feature = "serial_packets"))]
                    OutgoingPacketState::Sent => OutgoingPacketState::None,
                }
            }
        })
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

/// send the state transition to serial, new and previous states
#[cfg(debug_assertions)]
pub fn send_tuple(state: &mut State /*, state: (PyPilotStateMachine, PyPilotStateMachine)*/) {
    let ty = PacketType::CurrentFSM.into();
    let pkt: [u8; 3] = [ty, state.machine_state.0.into(), 0x00];
    let crc = crc::crc8(pkt);
    interrupt::free(|cs| {
        if let Some(wr) = USARTWRITER.borrow(cs).borrow_mut().as_mut() {
            for b in pkt.iter() {
                nb::block!(wr.write(*b)).void_unwrap();
            }
            nb::block!(wr.write(crc)).void_unwrap();
            let ty = PacketType::PreviousFSM.into();
            let pkt = [ty, state.machine_state.1.into(), 0x00];
            let crc = crc::crc8(pkt);
            for b in pkt.iter() {
                nb::block!(wr.write(*b)).void_unwrap();
            }
            nb::block!(wr.write(crc)).void_unwrap();
        }
    })
}

//==========================================================

#[interrupt(atmega328p)]
unsafe fn USART_RX() {
    // the usart reader
    static mut RDR: Option<SerialReader> = None;

    interrupt::free(|cs| {
        // if USARTREADER has been moved into exception handler, use it
        if let Some(rdr) = &mut RDR {
            match rdr.read() {
                Ok(word) => match PROD_QUEUE.borrow(cs).borrow_mut().as_mut() {
                    Some(pq) => match pq.enqueue(word) {
                        Ok(()) => {}
                        Err(_) => {}
                    },
                    None => {}
                },
                Err(_) => {}
            }
        }
        // otherwise move it out of the Mutex pretected shared region to here
        else {
            RDR.replace(USARTREADER.borrow(cs).replace(None).unwrap());
        }
    })
}

//==========================================================
