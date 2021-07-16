use crate::crc;
use crate::packet::{OutgoingPacketState, PacketType};
use crate::{CommandToExecute, Flags, State};
use arduino_hal::{
    clock,
    hal::{
        port,
        port::{mode, Pin},
        usart::Event,
    },
};
use avr_device::{
    interrupt,
    interrupt::{CriticalSection, Mutex},
};
use core::{
    cell::Cell,
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};
use embedded_hal::serial::{Read, Write};
use heapless::spsc::Queue;

//==========================================================

type Serial = arduino_hal::hal::usart::Usart<
    arduino_hal::pac::USART0,
    Pin<mode::Input, port::PD0>,
    Pin<mode::Output, port::PD1>,
    clock::MHz16,
>;

type SerialReader = arduino_hal::hal::usart::UsartReader<
    arduino_hal::pac::USART0,
    Pin<mode::Input, port::PD0>,
    Pin<mode::Output, port::PD1>,
    clock::MHz16,
>;

type SerialWriter = arduino_hal::hal::usart::UsartWriter<
    arduino_hal::pac::USART0,
    Pin<mode::Input, port::PD0>,
    Pin<mode::Output, port::PD1>,
    clock::MHz16,
>;

//==========================================================

/// queue for bytes received on uart
static mut READ_QUEUE: Queue<u8, 64> = Queue::new();
/// queue for bytes to send on uart
static mut WRITE_QUEUE: Queue<u8, 128> = Queue::new();
/// flag for read queue overflow
static mut READ_OVF: AtomicBool = AtomicBool::new(false);
/// flag for write queue overflow
static mut WRITE_OVF: AtomicBool = AtomicBool::new(false);

/// used in USART_RX isr
static mut USARTREADER: Mutex<Cell<Option<SerialReader>>> = Mutex::new(Cell::new(None));
/// used in USART_UDRE isr
static mut USARTWRITER: Mutex<Cell<Option<SerialWriter>>> = Mutex::new(Cell::new(None));

//==========================================================

/// initialize the queues and populate the static structures
pub fn init(cs: &CriticalSection, mut uart: Serial) {
    // turn on RX and UDRE interrupt
    uart.listen(Event::RxComplete);
    uart.listen(Event::DataRegisterEmpty);

    // split into reader and write and send to static variables
    let (reader, writer) = uart.split();
    // SAFETY: the queues are in a interrupt::free context so access is safe
    unsafe {
        USARTREADER.borrow(cs).replace(Some(reader));
        USARTWRITER.borrow(cs).replace(Some(writer));
    }
}

//==========================================================

/// get the next byte, if available, from the read queue
pub fn get_next_byte(_cs: &CriticalSection) -> Option<u8> {
    // SAFETY: inside of critical section
    unsafe { READ_QUEUE.split().1.dequeue() }
}

//==========================================================

/// send bytes to the write queue, returns number of bytes put on queue
pub fn transmit_bytes(_cs: &CriticalSection, bytes: &[u8]) -> usize {
    let mut count: usize = 0;
    for b in bytes.iter() {
        // SAFETY: inside of critical section
        match unsafe { WRITE_QUEUE.split().0.enqueue(*b) } {
            Ok(()) => count += 1,
            // SAFETY: inside a critical section
            Err(_) => unsafe { WRITE_OVF.store(true, Ordering::Relaxed) },
        }
    }
    // SAFETY: inside critical section
    // turn on UDRE interrupt
    unsafe { ptr::write_volatile(0xC1 as *mut u8, 0xb8) };
    count
}

//==========================================================

/// has the read queue overflowed
pub fn read_overflow() -> bool {
    // SAFETY: read of u8 should be safe
    unsafe { READ_OVF.load(Ordering::Relaxed) }
}

//==========================================================

/// has the write queue overflowed
pub fn write_overflow() -> bool {
    // SAFETY: read of u8 should be safe
    unsafe { WRITE_OVF.load(Ordering::Relaxed) }
}

//==========================================================

/// if outgoing state says to send a packet, do so
#[inline]
pub fn send_serial(outgoing_state: OutgoingPacketState) {
    // process outgoing packet if necessary
    match outgoing_state {
        OutgoingPacketState::Send(pkt) => {
            let cr = crc::crc8(pkt);
            interrupt::free(|cs| transmit_bytes(&cs, &[pkt[0], pkt[1], pkt[2], cr]));
        }
        _ => {}
    }
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
    static mut SYNC_B: u8 = 0;
    static mut IN_SYNC_COUNT: u8 = 0;

    // check overflow flags
    if read_overflow() {
        flags.insert(Flags::READQUEUEOVF);
    }
    if write_overflow() {
        flags.insert(Flags::WRITEQUEUEOVF);
    }
    let mut reset_comm_timeout = false;
    if let Some(word) = interrupt::free(|cs| get_next_byte(&cs)) {
        // reset the serial timeout now, since we've received data
        reset_comm_timeout = true;
        // SAFETY: this is safe because none of these static structures are modified
        // elsewhere, they are only used here in this function
        unsafe {
            if SYNC_B < 3 {
                IN_BYTES[SYNC_B as usize] = word;
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
                    *pending_cmd = CommandToExecute::Stop;
                }
            }
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
    interrupt::free(|cs| {
        transmit_bytes(&cs, &pkt);
        transmit_bytes(&cs, &crc);
    });
    let ty = PacketType::PreviousFSM.into();
    let pkt = [ty, state.machine_state.1.into(), 0x00];
    let crc = [crc::crc8(pkt)];
    interrupt::free(|cs| {
        transmit_bytes(&cs, &pkt);
        transmit_bytes(&cs, &crc);
    });
}

//==========================================================

#[interrupt(atmega328p)]
unsafe fn USART_UDRE() {
    static mut WTR: Option<SerialWriter> = None;
    interrupt::free(|cs| {
        // if usart writer has been moved here, use it
        if let Some(wtr) = &mut WTR {
            match WRITE_QUEUE.split().1.dequeue() {
                Some(word) => match wtr.write(word) {
                    Ok(()) => {}
                    Err(_) => {}
                },
                None => {
                    // turn off UDRE interrupt
                    ptr::write_volatile(0xC1 as *mut u8, 0x98);
                }
            }
        }
        // otherwise move the writer into this isr
        else {
            WTR.replace(USARTWRITER.borrow(cs).replace(None).unwrap());
        }
    })
}

//==========================================================

#[interrupt(atmega328p)]
unsafe fn USART_RX() {
    // the usart reader
    static mut RDR: Option<SerialReader> = None;
    interrupt::free(|cs| {
        // if reader moved here, use it
        if let Some(rdr) = &mut RDR {
            match rdr.read() {
                Ok(word) => match READ_QUEUE.split().0.enqueue(word) {
                    Ok(()) => {}
                    Err(_) => READ_OVF.store(true, Ordering::Relaxed),
                },
                Err(_) => {}
            };
        }
        // otherwise move the usart reader into this isr
        else {
            RDR.replace(USARTREADER.borrow(cs).replace(None).unwrap());
        }
    })
}

//==========================================================
