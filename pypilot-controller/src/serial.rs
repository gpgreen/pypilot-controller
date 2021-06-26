use crate::crc;
use crate::packet::{OutgoingPacketState, PacketType};
use crate::{CommandToExecute, Flags, State};
use avr_device::interrupt;
use avr_device::interrupt::Mutex;
use core::cell::RefCell;
use core::ptr;
use hal::port::mode::{Floating, Input, Output};
use heapless::spsc::{Consumer, Producer, Queue};
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

/// used in USART_RX isr
static mut READ_PROD_QUEUE: Mutex<RefCell<Option<Producer<u8, 64>>>> =
    Mutex::new(RefCell::new(None));
/// used in get_next_byte
static mut READ_CONSUME_QUEUE: Option<Consumer<u8, 64>> = None;
/// used in transmit_bytes
static mut WRITE_PROD_QUEUE: Mutex<RefCell<Option<Producer<u8, 64>>>> =
    Mutex::new(RefCell::new(None));
/// used in USART_UDRE isr
static mut WRITE_CONSUME_QUEUE: Mutex<RefCell<Option<Consumer<u8, 64>>>> =
    Mutex::new(RefCell::new(None));
/// used in isr
static mut USARTREADER: Mutex<RefCell<Option<SerialReader>>> = Mutex::new(RefCell::new(None));
/// used in isr
static mut USARTWRITER: Mutex<RefCell<Option<SerialWriter>>> = Mutex::new(RefCell::new(None));

/// utility function for getting next available byte received on USART
fn get_next_byte() -> Option<u8> {
    // SAFETY: dequeue is a lockless function
    unsafe {
        if let Some(q) = &mut READ_CONSUME_QUEUE {
            q.dequeue()
        } else {
            None
        }
    }
}

/// utility function for putting bytes into transmit queue for USART
fn transmit_bytes(bytes: &[u8]) -> usize {
    let mut count: usize = 0;
    // SAFETY: queue in a interrupt::free context so access is safe
    unsafe {
        interrupt::free(|cs| {
            match WRITE_PROD_QUEUE.borrow(cs).borrow_mut().as_mut() {
                Some(q) => {
                    if q.len() == 0 {
                        ptr::write_volatile(0xC6 as *mut u8, bytes[0]);
                        count = 1;
                        for b in bytes[1..].iter() {
                            match q.enqueue(*b) {
                                Ok(()) => count += 1,
                                Err(_) => break,
                            }
                        }
                    } else {
                        for b in bytes.iter() {
                            match q.enqueue(*b) {
                                Ok(()) => count += 1,
                                Err(_) => break,
                            }
                        }
                    }
                }
                None => {}
            };
            // turn on UDRE interrupt
            ptr::write_volatile(0xC1 as *mut u8, 0xb8);
        });
    }
    count
}

//==========================================================

/// setup the queues, move them into the static variables so they can be moved into the fns
pub fn setup_serial(reader: SerialReader, writer: SerialWriter) {
    static mut READ_QUEUE: Queue<u8, 64> = Queue::new();
    static mut WRITE_QUEUE: Queue<u8, 64> = Queue::new();

    // SAFETY: the queues are in a interrupt::free context so access is safe
    unsafe {
        interrupt::free(|cs| {
            let (rdwr, rdrd) = READ_QUEUE.split();
            let (wrwr, wrrd) = WRITE_QUEUE.split();

            READ_CONSUME_QUEUE.replace(rdrd);
            READ_PROD_QUEUE.borrow(cs).replace(Some(rdwr));
            WRITE_CONSUME_QUEUE.borrow(cs).replace(Some(wrrd));
            WRITE_PROD_QUEUE.borrow(cs).replace(Some(wrwr));
            USARTREADER.borrow(cs).replace(Some(reader));
            USARTWRITER.borrow(cs).replace(Some(writer));
        });
    }
}

/// if outgoing state says to send a packet, do so
pub fn send_serial(outgoing_state: &mut OutgoingPacketState) {
    // process outgoing packet if necessary
    *outgoing_state = match *outgoing_state {
        OutgoingPacketState::None => OutgoingPacketState::None,
        OutgoingPacketState::Build => OutgoingPacketState::Build,
        OutgoingPacketState::Send(pkt) => {
            transmit_bytes(&[pkt[0], pkt[1], pkt[2], crc::crc8(pkt)]);
            OutgoingPacketState::None
        }
    };
}

/// check for incoming bytes, convert to packets if possible
/// if outgoing state says to send a packet, do so
/// if a packet is received, set outgoing state to build a new packet
/// if a packet is received, set pending_cmd to process it
/// set flags as required for communication state
pub fn process_serial(
    flags: &mut Flags,
    pending_cmd: &mut CommandToExecute,
    outgoing_state: &mut OutgoingPacketState,
) -> bool {
    static mut IN_BYTES: [u8; 3] = [0; 3];
    static mut SYNC_B: usize = 0;
    static mut IN_SYNC_COUNT: u8 = 0;

    let mut reset_comm_timeout = false;
    loop {
        match get_next_byte() {
            Some(word) => {
                // reset the serial timeout now, since we've received data
                //state.comm_timeout = 0;
                reset_comm_timeout = true;
                // SAFETY: this is safe because none of these static structures are modified
                // elsewhere, they are only used here in this function
                unsafe {
                    if SYNC_B < 3 {
                        IN_BYTES[SYNC_B] = word;
                        SYNC_B += 1;
                    } else {
                        let pkt_crc = crc::crc8(IN_BYTES);
                        if word == pkt_crc {
                            SYNC_B = 0;
                            if IN_SYNC_COUNT >= 2 {
                                *pending_cmd = CommandToExecute::ProcessPacket(IN_BYTES);
                            } else {
                                IN_SYNC_COUNT += 1;
                            }
                            flags.insert(Flags::SYNC);
                            flags.remove(Flags::INVALID);
                            *outgoing_state = OutgoingPacketState::Build;
                        } else {
                            // invalid packet
                            IN_SYNC_COUNT = 0;
                            IN_BYTES[0] = IN_BYTES[1];
                            IN_BYTES[1] = IN_BYTES[2];
                            IN_BYTES[2] = word;
                            flags.remove(Flags::SYNC);
                            flags.insert(Flags::INVALID);
                            //*outgoing_state = OutgoingPacketState::Build;
                            *pending_cmd = CommandToExecute::Stop;
                        }
                    }
                }
            }
            None => break,
        }
    }
    reset_comm_timeout
}

//==========================================================

/// send the state transition to serial, new and previous states
#[cfg(debug_assertions)]
pub fn send_tuple(state: &mut State /*, state: (PyPilotStateMachine, PyPilotStateMachine)*/) {
    let ty = PacketType::CurrentFSM.into();
    let pkt: [u8; 3] = [ty, state.machine_state.0.into(), 0x00];
    let crc: [u8; 1] = [crc::crc8(pkt)];
    transmit_bytes(&pkt);
    transmit_bytes(&crc);
    let ty = PacketType::PreviousFSM.into();
    let pkt = [ty, state.machine_state.1.into(), 0x00];
    let crc = [crc::crc8(pkt)];
    transmit_bytes(&pkt);
    transmit_bytes(&crc);
}

//==========================================================

#[interrupt(atmega328p)]
unsafe fn USART_UDRE() {
    static mut WTR: Option<SerialWriter> = None;
    static mut CQ: Option<Consumer<u8, 64>> = None;
    // if usart writer has been moved here, use it
    if let Some(wtr) = &mut WTR {
        if let Some(cq) = &mut CQ {
            match cq.dequeue() {
                Some(word) => match wtr.write(word) {
                    Ok(()) => {}
                    Err(_) => {}
                },
                None => {
                    // turn off UDRE interrupt
                    ptr::write_volatile(0xC1 as *mut u8, 0x98);
                }
            }
        } else {
            interrupt::free(|cs| {
                CQ.replace(WRITE_CONSUME_QUEUE.borrow(cs).replace(None).unwrap());
            })
        }
    }
    // otherwise move the writer into this isr
    else {
        interrupt::free(|cs| {
            WTR.replace(USARTWRITER.borrow(cs).replace(None).unwrap());
        })
    }
}

//==========================================================

#[interrupt(atmega328p)]
unsafe fn USART_RX() {
    // the usart reader
    static mut RDR: Option<SerialReader> = None;
    static mut PQ: Option<Producer<u8, 64>> = None;
    // with the sequence of moving 2 functions into this isr, we will lose
    // some words, but that is ok, the controller will keep sending them
    // if USARTREADER has been moved into exception handler, use it
    if let Some(rdr) = &mut RDR {
        match rdr.read() {
            Ok(word) => {
                // if read producer here, use it
                if let Some(pq) = &mut PQ {
                    match pq.enqueue(word) {
                        Ok(()) => {}
                        Err(_) => {}
                    }
                }
                // otherwise move it into this isr
                else {
                    interrupt::free(|cs| {
                        PQ.replace(READ_PROD_QUEUE.borrow(cs).replace(None).unwrap());
                    })
                }
            }
            Err(_) => {}
        }
    }
    // otherwise move it out of the Mutex pretected shared region to here
    else {
        interrupt::free(|cs| {
            RDR.replace(USARTREADER.borrow(cs).replace(None).unwrap());
        })
    }
}

//==========================================================
