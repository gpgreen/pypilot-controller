//! firmware for pypilot controller board

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(llvm_asm)]

extern crate panic_halt;

use avr_device::interrupt;
use core::cell::RefCell;
use core::convert::TryInto;
use core::ptr;
use hal::port::mode::{Floating, Input, Output, PullUp, Pwm};
use pypilot_controller_board::hal;
use pypilot_controller_board::prelude::*;
use pypilot_controller_board::pwm::{Prescaler, Timer0Pwm};
use pypilot_controller_board::wdt::{Timeout, Wdt};

// modules from project
mod adc;
use adc::ADC_RESULTS;
#[macro_use]
extern crate bitflags;
mod crc;
mod packet;
use packet::OutgoingPacketState;
mod eeprom;
#[cfg(feature = "serial_packets")]
mod serial;

//==========================================================
// Hardware.flags

bitflags! {
    pub struct Flags: u16 {
    const SYNC = 0b0000_0000_0000_0001;
    const OVERTEMPFAULT = 0b0000_0000_0000_0010;
    const OVERCURRENTFAULT = 0b0000_0000_0000_0100;
    const ENGAGED = 0b0000_0000_0000_1000;
    const INVALID = 0b0000_0000_0001_0000;
    const PORTPINFAULT = 0b0000_0000_0010_0000;
    const STBDPINFAULT = 0b0000_0000_0100_0000;
    const BADVOLTAGEFAULT = 0b0000_0000_1000_0000;
    const MINRUDDERFAULT = 0b0000_0001_0000_0000;
    const MAXRUDDERFAULT = 0b0000_0010_0000_0000;
    const CURRENTRANGE = 0b0000_0100_0000_0000;
    const BADFUSES = 0b0000_1000_0000_0000;
    const REBOOTED = 0b0001_0000_0000_0000;
    }
}

impl From<Flags> for u16 {
    fn from(original: Flags) -> u16 {
        original.bits
    }
}

impl Flags {
    pub fn clear(&mut self) {
        self.bits = 0;
    }
}

//==========================================================

/// using low or high current
const LOWCURRENT: bool = true;

/// maximum current allowed
const MAX_CURRENT: u16 = 2000; // 20 amps
const MAX_VOLTAGE: u16 = 1600; // 16 volts max in 12 volt mode
const MAX_CONTROLLER_TEMP: u16 = 7000; // 70C
const MAX_MOTOR_TEMP: u16 = 7000; // 70C

//==========================================================

/// command to execute
/// in a function that changes hardware state variables,
/// use this enum to flag a function that needs to be
/// executed following. Also used in state transitions
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CommandToExecute {
    Stop,
    Engage,
    Disengage,
    ProcessPacket([u8; 3]),
    SendPacket([u8; 3]),
    None,
}

//==========================================================

/// the state variables for the hardware
pub struct State {
    // hardware stuff
    cpu: pypilot_controller_board::pac::CPU,
    exint: pypilot_controller_board::pac::EXINT,
    timer0: pypilot_controller_board::pwm::Timer0Pwm,
    timer2: pypilot_controller_board::pac::TC2,
    adc: RefCell<avr_device::atmega328p::ADC>,
    led_pin: RefCell<Option<hal::port::portb::PB5<Output>>>,
    ena_pin: hal::port::portd::PD4<Input<PullUp>>,
    enb_pin: hal::port::portd::PD5<Input<PullUp>>,
    ina_pin: hal::port::portd::PD2<Output>,
    inb_pin: hal::port::portd::PD3<Output>,
    pwm_pin: hal::port::portd::PD6<Pwm<Timer0Pwm>>,
    stbd_stop_pin: hal::port::portd::PD7<Input<PullUp>>,
    port_stop_pin: hal::port::portb::PB0<Input<PullUp>>,
    wdt: hal::wdt::Wdt,
    eeprom: avr_device::atmega328p::EEPROM,
    // state stuff
    machine_state: (PyPilotStateMachine, PyPilotStateMachine),
    prev_state: (PyPilotStateMachine, PyPilotStateMachine),
    timeout: u16,
    comm_timeout: u16,
    command_value: u16,
    lastpos: u16,
    max_slew_speed: u8,
    max_slew_slow: u8,
    max_voltage: u16,
    max_current: u16,
    max_controller_temp: u16,
    max_motor_temp: u16,
    rudder_min: u16,
    rudder_max: u16,
    flags: Flags,
    pending_cmd: CommandToExecute,
    outgoing_state: OutgoingPacketState,
}

//==========================================================

#[hal::entry]
fn main() -> ! {
    let mut state = setup();
    let mut led_cnt = 0;
    let mut led_on = true;
    static mut LED: Option<hal::port::portb::PB5<Output>> = None;

    loop {
        state.wdt.feed();
        unsafe {
            if let Some(led) = &mut LED {
                if led_cnt == 100 {
                    led_cnt = 0;
                    if led_on {
                        led.set_low().void_unwrap();
                        led_on = false;
                    } else {
                        led.set_high().void_unwrap();
                        led_on = true;
                    }
                } else {
                    led_cnt += 1;
                }
            } else {
                LED.replace(state.led_pin.replace(None).unwrap());
            }
        }

        // in serial processing
        #[cfg(feature = "serial_packets")]
        if serial::process_serial(
            &mut state.flags,
            &mut state.pending_cmd,
            &mut state.outgoing_state,
        ) {
            state.comm_timeout = 0;
        }
        state.wdt.feed();

        state = process_fsm(state);

        // out serial processing
        #[cfg(feature = "serial_packets")]
        serial::send_serial(&mut state.outgoing_state);
    }
}

//==========================================================

/// finite state machine state's
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PyPilotStateMachine {
    Start,
    WaitEntry,
    Wait,
    Engage,
    Operational,
    DisengageEntry,
    Disengage,
    DetachEntry,
    Detach,
    PowerDown,
}

/// transition from one state to another, records previous state
fn change_state(
    new_state: PyPilotStateMachine,
    state_memo: (PyPilotStateMachine, PyPilotStateMachine),
) -> (PyPilotStateMachine, PyPilotStateMachine) {
    if let PyPilotStateMachine::Start
    | PyPilotStateMachine::Wait
    | PyPilotStateMachine::Engage
    | PyPilotStateMachine::Operational
    | PyPilotStateMachine::Disengage
    | PyPilotStateMachine::Detach
    | PyPilotStateMachine::PowerDown = state_memo.0
    {
        (new_state, state_memo.0)
    } else {
        (new_state, state_memo.1)
    }
}

/// convert to u8
impl From<PyPilotStateMachine> for u8 {
    /// send the state to serial
    fn from(original: PyPilotStateMachine) -> u8 {
        match original {
            PyPilotStateMachine::Start => 0x00,
            PyPilotStateMachine::WaitEntry => 0x01,
            PyPilotStateMachine::Wait => 0x02,
            PyPilotStateMachine::Engage => 0x03,
            PyPilotStateMachine::Operational => 0x04,
            PyPilotStateMachine::DisengageEntry => 0x05,
            PyPilotStateMachine::Disengage => 0x06,
            PyPilotStateMachine::DetachEntry => 0x07,
            PyPilotStateMachine::Detach => 0x08,
            PyPilotStateMachine::PowerDown => 0x09,
        }
    }
}

//==========================================================

/// setup the hardware for the main loop
fn setup() -> State {
    let dp = pypilot_controller_board::Peripherals::take().unwrap();
    let flags = Flags::empty();

    /*
        // read fuses, and report this as flag if they are wrong
        let fuse = dp.FUSE;
        let low_fuse = fuse.low.read().bits();
        let high_fuse = fuse.high.read().bits();
        let extended_fuse = fuse.extended.read().bits();
        if (low_fuse != 0xFF && low_fuse != 0x7f)
            || (high_fuse != 0xDA && high_fuse != 0xDE)
            || (extended_fuse != 0xFD && extended_fuse != 0xFC)
        {
            flags.insert(Flags::BADFUSES);
        }
    */
    // turn off unused modules
    let cpu = dp.CPU;
    cpu.prr.write(|w| {
        w.prtim1().set_bit();
        w.prtwi().set_bit()
    });
    #[cfg(not(feature = "serial_packets"))]
    #[cfg(not(debug_assertions))]
    cpu.prr.modify(|_, w| w.prusart0().set_bit());
    // turn off analog comparator
    let ac = dp.AC;
    ac.acsr.write(|w| w.acd().set_bit());

    // sort out the pins
    let mut pins = pypilot_controller_board::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);

    let stbd_stop_pin = pins.d7.into_pull_up_input(&mut pins.ddr);
    let port_stop_pin = pins.d8.into_pull_up_input(&mut pins.ddr);

    #[cfg(not(debug_assertions))]
    #[cfg(not(feature = "serial_packets"))]
    pins.rx.into_pull_up_input(&mut pins.ddr);
    #[cfg(not(debug_assertions))]
    #[cfg(not(feature = "serial_packets"))]
    pins.tx.into_pull_up_input(&mut pins.ddr);

    // turn on timer2
    let timer2 = dp.TC2;
    timer2.tccr2b.modify(|_, w| w.cs2().prescale_1024());

    // pwm
    let mut timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale8);
    let pwm_pin = pins.d6.into_output(&mut pins.ddr).into_pwm(&mut timer0);
    // clockwise
    let ina_pin = pins.d2.into_output(&mut pins.ddr);
    // counterclockwise
    let inb_pin = pins.d3.into_output(&mut pins.ddr);

    // half-bridge fault pins, also enable pins, but carrier has them pulled high
    // already
    let ena_pin = pins.d4.into_pull_up_input(&mut pins.ddr);
    let enb_pin = pins.d5.into_pull_up_input(&mut pins.ddr);

    // SPI pins
    let mut led = pins.d13.into_output(&mut pins.ddr);
    led.set_high().void_unwrap();

    //let _sck = pins.d13.into_pull_up_input(&mut pins.ddr);
    let _miso = pins.d12.into_output(&mut pins.ddr);
    let _mosi = pins.d11.into_pull_up_input(&mut pins.ddr);
    let _cs = pins.d10.into_pull_up_input(&mut pins.ddr);
    let _can_int = pins.d9.into_floating_input(&mut pins.ddr);

    // now module setup

    // setup serial
    #[cfg(feature = "serial_packets")]
    let uart0 = pypilot_controller_board::Serial::<Floating>::new(
        dp.USART0,
        pins.rx,
        pins.tx.into_output(&mut pins.ddr),
        115200.into_baudrate(),
    );
    #[cfg(feature = "serial_packets")]
    serial::setup_serial(uart0);

    // setup adc, not using the HAL, because it doesn't do interrupts
    let adc = dp.ADC;
    adc.adcsra.write(|w| {
        w.aden()
            .set_bit()
            .adsc()
            .set_bit()
            .adie()
            .set_bit()
            .adps()
            .prescaler_128()
    });
    adc.didr0.write(|w| {
        w.adc0d()
            .set_bit()
            .adc1d()
            .set_bit()
            .adc2d()
            .set_bit()
            .adc3d()
            .set_bit()
            .adc4d()
            .set_bit()
            .adc5d()
            .set_bit()
    });

    // setup adc_results, and enable interrupts
    interrupt::free(|cs| ADC_RESULTS.setup(&cs));
    unsafe {
        interrupt::enable();
    }

    // watchdog
    let mut watchdog = Wdt::new(&cpu.mcusr, dp.WDT);
    watchdog.start(Timeout::Ms250);

    State {
        cpu: cpu,
        exint: dp.EXINT,
        timer0: timer0,
        timer2: timer2,
        ena_pin: ena_pin,
        enb_pin: enb_pin,
        ina_pin: ina_pin,
        inb_pin: inb_pin,
        pwm_pin: pwm_pin,
        stbd_stop_pin: stbd_stop_pin,
        port_stop_pin: port_stop_pin,
        adc: RefCell::new(adc),
        led_pin: RefCell::new(Some(led)),
        wdt: watchdog,
        eeprom: dp.EEPROM,
        machine_state: (PyPilotStateMachine::Start, PyPilotStateMachine::Start),
        prev_state: (PyPilotStateMachine::Start, PyPilotStateMachine::Start),
        timeout: 0,
        comm_timeout: 250,
        command_value: 1000,
        lastpos: 1000,
        max_slew_speed: 15,
        max_slew_slow: 35,
        // 16V
        max_voltage: 1600,
        // 20 amps
        max_current: 2000,
        // 70C
        max_controller_temp: 7000,
        // 70C
        max_motor_temp: 7000,
        rudder_min: 0,
        rudder_max: 65535,
        flags: flags,
        pending_cmd: CommandToExecute::None,
        outgoing_state: OutgoingPacketState::None,
    }
}

//==========================================================

/// power down the cpu
fn power_down_mode(state: &mut State) {
    // delay for a bit, so that usart can finish any transmission
    #[cfg(any(debug_assertions, feature = "serial_packets"))]
    pypilot_controller_board::delay_ms(10);

    // disable modules
    // adc off
    state
        .adc
        .borrow_mut()
        .adcsra
        .write(|w| w.aden().clear_bit().adif().clear_bit());

    // release hal serial which turns it off
    #[cfg(any(debug_assertions, feature = "serial_packets"))]
    //let (usart, rxpin, txpin) = state.serial.into_inner().release();

    // turn off clocks
    state.cpu.prr.modify(|_, w| {
        #[cfg(any(debug_assertions, feature = "serial_packets"))]
        w.prusart0().set_bit();
        w.prspi().set_bit();
        w.prtim0().set_bit();
        w.prtim2().set_bit();
        w.pradc().set_bit()
    });

    // set PCINT2 interrupt
    unsafe {
        state.exint.pcicr.modify(|r, w| w.bits(r.bits() | 4));
        // set RX pin interrupt (PD0 = PCINT16)
        state.exint.pcmsk2.modify(|r, w| w.bits(r.bits() | 1));
    }

    // do the power down, if PCINT2 interrupt hasn't happened
    // no other interrupts are important, as the Pi will
    // be powered off in this state, so only need
    // to detect receive activity
    state.cpu.smcr.write(|w| w.sm().pdown());
    interrupt::disable();
    unsafe {
        if !ptr::read_volatile(&PENDINGPCINT2) {
            // set TCNT2 to 0
            state.timer2.tcnt2.write(|w| w.bits(0));
            // sleep enable
            state.cpu.smcr.modify(|_, w| w.se().set_bit());
            interrupt::enable();
            // sleep cpu
            avr_device::asm::sleep();
            // sleep disable
            state.cpu.smcr.modify(|_, w| w.se().clear_bit());
        } else {
            // PCINT2 pending
            ptr::write_volatile(&mut PENDINGPCINT2, false);
        }
        interrupt::enable();
    }

    // stop PCINT2 interrupt
    unsafe {
        state.exint.pcicr.modify(|r, w| w.bits(r.bits() & !4));
        state.exint.pcmsk2.modify(|r, w| w.bits(r.bits() & !1));
    }

    // turn on clocks
    state.cpu.prr.modify(|_, w| {
        #[cfg(any(debug_assertions, feature = "serial_packets"))]
        w.prusart0().clear_bit();
        w.prspi().clear_bit();
        w.prtim0().clear_bit();
        w.prtim2().clear_bit();
        w.pradc().clear_bit()
    });

    // enable modules
    state.adc.borrow_mut().adcsra.write(|w| {
        w.aden()
            .set_bit()
            .adsc()
            .set_bit()
            .adie()
            .set_bit()
            .adps()
            .prescaler_128()
    });
}

//==========================================================

/// stop the motor
fn stop(state: &mut State) {
    state.command_value = 1000;
    position(state, 1000)
}

//==========================================================

#[inline]
fn housekeeping(state: &mut State) {
    let mut do_stop = false;

    /*
    // rudder_stop_checks
    if state.stbd_stop_pin.is_low().void_unwrap() {
        state.flags.insert(Flags::STBDPINFAULT);
        do_stop = true;
    } else {
        state.flags.remove(Flags::STBDPINFAULT);
    }
    if state.port_stop_pin.is_low().void_unwrap() {
        state.flags.insert(Flags::PORTPINFAULT);
        do_stop = true;
    } else {
        state.flags.remove(Flags::PORTPINFAULT);
    }

    // test fault pins
    if state.ena_pin.is_low().void_unwrap() || state.enb_pin.is_low().void_unwrap() {
        state.flags.insert(Flags::OVERCURRENTFAULT);
        do_stop = true;
    } else {
        state.flags.remove(Flags::OVERCURRENTFAULT);
    }
        // test current
        let cnt;
        unsafe {
            cnt = ADC_RESULTS.count(adc::AdcChannel::Current, 1);
        }
        if cnt > 400 {
            let amps;
            unsafe {
                amps = ADC_RESULTS.take_amps(1);
            }
            if amps >= MAX_CURRENT {
                do_stop = true;
                state.flags.insert(Flags::OVERCURRENTFAULT);
            } else {
                state.flags.remove(Flags::OVERCURRENTFAULT);
            }
        }
    */
    // stop if asked
    if do_stop {
        stop(state);
    }
}

/*
//==========================================================

/// test voltage
fn test_voltage(state: &mut State) {
    /*
        let cnt;
        unsafe {
            cnt = ADC_RESULTS.count(adc::AdcChannel::Voltage, 1);
        }
        if cnt > 400 {
            let volts;
            unsafe {
                volts = ADC_RESULTS.take_volts(1);
            }
            if volts <= 900 || volts >= MAX_VOLTAGE {
                stop(state);
                state.flags.insert(Flags::BADVOLTAGEFAULT);
            } else {
                state.flags.remove(Flags::BADVOLTAGEFAULT);
            }
        }
    */
}
 */

//==========================================================

/// test motor or controller temp
#[cfg(any(feature = "motor_temp", feature = "controller_temp"))]
fn test_overtemp(state: &mut State) {
    let ccnt;
    let mcnt;
    unsafe {
        ccnt = ADC_RESULTS.count(adc::AdcChannel::ControllerTemp, 1);
        mcnt = ADC_RESULTS.count(adc::AdcChannel::MotorTemp, 1);
    }
    if ccnt > 100 && mcnt > 100 {
        let ctemp;
        let mtemp;
        unsafe {
            ctemp = ADC_RESULTS.take_controller_temp(1);
            mtemp = ADC_RESULTS.take_motor_temp(1);
        }
        if ctemp >= MAX_CONTROLLER_TEMP || mtemp > MAX_MOTOR_TEMP {
            stop(state);
            state.flags.insert(Flags::OVERTEMPFAULT);
        } else {
            state.flags.remove(Flags::OVERTEMPFAULT);
        }
    }
}

//==========================================================

/// update the pwm based on motor command
fn position(state: &mut State, value: u16) {
    // foPWM = fclk/(prescale*256),prescale=8
    // foPWM = 7.8125kHz
    let newpos = if value >= 1000 {
        (value - 1000) / 4
    } else {
        (1000 - value) / 4
    };
    state.pwm_pin.set_duty(newpos.try_into().unwrap_or(0));
    if value > 1040 {
        state.ina_pin.set_low().unwrap();
        state.inb_pin.set_high().unwrap();
    } else if value < 960 {
        state.ina_pin.set_high().unwrap();
        state.inb_pin.set_low().unwrap();
    } else {
        // set brake
        state.ina_pin.set_low().unwrap();
        state.inb_pin.set_low().unwrap();
    }
}

//==========================================================

/// the main state machine processer
fn process_fsm(mut state: State) -> State {
    // check timer2 counter, boolean 'do_update' s/b set
    // if motor position to be updated
    // timer2 is set to 1024 prescale, so each timer tick=15625hz=64ns
    // that means each update is at 5ms
    let do_update = {
        if state.timer2.tcnt2.read().bits() > 78 {
            state.timeout += 1;
            state.comm_timeout += 1;
            unsafe {
                state.timer2.tcnt2.modify(|r, w| w.bits(r.bits() - 78));
            }
            true
        } else {
            false
        }
    };

    if do_update {
        housekeeping(&mut state);
    }

    // process packet
    match state.pending_cmd {
        CommandToExecute::ProcessPacket(pkt) => packet::process_packet(pkt, &mut state),
        CommandToExecute::Stop => {
            stop(&mut state);
            state.pending_cmd = CommandToExecute::None;
        }
        _ => {}
    }
    // check for stop from process packet
    if state.pending_cmd == CommandToExecute::Stop {
        stop(&mut state);
    }

    // build outgoing packet if asked
    if state.outgoing_state == OutgoingPacketState::Build {
        state.outgoing_state = packet::build_outgoing(&state);
    }

    // FSM
    let newstate = match state.machine_state.0 {
        PyPilotStateMachine::Start => {
            change_state(PyPilotStateMachine::WaitEntry, state.machine_state)
        }
        PyPilotStateMachine::WaitEntry => {
            state.timeout = 0;
            state.ina_pin.set_low().void_unwrap();
            state.inb_pin.set_low().void_unwrap();
            change_state(PyPilotStateMachine::Wait, state.machine_state)
        }
        PyPilotStateMachine::Wait => {
            if state.timeout > 30 || state.pending_cmd == CommandToExecute::Disengage {
                change_state(PyPilotStateMachine::DisengageEntry, state.machine_state)
            } else if state.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, state.machine_state)
            } else {
                state.machine_state
            }
        }
        PyPilotStateMachine::Engage => {
            state.timeout = 0;
            state.flags.insert(Flags::ENGAGED);
            // start PWM
            state.ina_pin.set_low().void_unwrap();
            state.ina_pin.set_low().void_unwrap();
            state.pwm_pin.set_duty(0);
            state.pwm_pin.enable();
            position(&mut state, 1000);
            change_state(PyPilotStateMachine::Operational, state.machine_state)
        }
        PyPilotStateMachine::Operational => {
            if do_update {
                let speed_rate: i16 = match state.max_slew_speed.try_into() {
                    Ok(v) => v,
                    Err(_) => 250,
                };
                // value of 20 is 1 second full range at 50hz
                let slow_rate: i16 = match state.max_slew_slow.try_into() {
                    Ok(v) => v,
                    Err(_) => 250,
                };
                let cmd_value: i16 = match state.command_value.try_into() {
                    Ok(v) => v,
                    Err(_) => 1000,
                };
                let cur_value: i16 = match state.lastpos.try_into() {
                    Ok(v) => v,
                    Err(_) => 1000,
                };
                // this should always be in range
                let mut diff = cmd_value - cur_value;
                // limit motor speed change to within speed and slow slew rates
                if diff > 0 {
                    if cur_value < 1000 {
                        if diff > slow_rate {
                            diff = slow_rate;
                        }
                    } else {
                        if diff > speed_rate {
                            diff = speed_rate
                        }
                    }
                } else {
                    if cur_value > 1000 {
                        if diff < -slow_rate {
                            diff = -slow_rate;
                        }
                    } else {
                        if diff < -speed_rate {
                            diff = -speed_rate;
                        }
                    }
                }
                let mut new_pos = state.lastpos;
                if diff < 0 {
                    // check if potential pos is less 0
                    if (-diff) as u16 > new_pos {
                        new_pos = 0;
                    } else {
                        new_pos -= (-diff) as u16;
                    }
                } else if diff > 0 {
                    // check if potential pos is greater 2000
                    if diff as u16 + new_pos > 2000 {
                        new_pos = 2000;
                    } else {
                        new_pos += diff as u16;
                    }
                }
                position(&mut state, new_pos);
            }
            if state.timeout > 30 || state.pending_cmd == CommandToExecute::Disengage {
                change_state(PyPilotStateMachine::DisengageEntry, state.machine_state)
            } else {
                state.machine_state
            }
        }
        PyPilotStateMachine::DisengageEntry => {
            stop(&mut state);
            state.flags.remove(Flags::ENGAGED);
            if state.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, state.machine_state)
            } else {
                change_state(PyPilotStateMachine::Disengage, state.machine_state)
            }
        }
        PyPilotStateMachine::Disengage => {
            if state.timeout > 62 {
                change_state(PyPilotStateMachine::DetachEntry, state.machine_state)
            } else if state.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, state.machine_state)
            } else {
                state.machine_state
            }
        }
        PyPilotStateMachine::DetachEntry => {
            state.ina_pin.set_low().void_unwrap();
            state.inb_pin.set_low().void_unwrap();
            state.pwm_pin.set_duty(0);
            state.pwm_pin.disable();
            change_state(PyPilotStateMachine::Detach, state.machine_state)
        }
        PyPilotStateMachine::Detach => {
            if state.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, state.machine_state)
            } else if state.timeout > 62 && state.comm_timeout > 250 {
                change_state(PyPilotStateMachine::PowerDown, state.machine_state)
            } else {
                state.machine_state
            }
        }
        PyPilotStateMachine::PowerDown => {
            // cpu goes to sleep, then woken up
            power_down_mode(&mut state);
            state.timeout = 0;
            state.comm_timeout -= 250;
            state.command_value = 1000;
            state.lastpos = 1000;
            state.flags.clear();
            state.pending_cmd = CommandToExecute::None;
            state.outgoing_state = OutgoingPacketState::None;
            change_state(PyPilotStateMachine::WaitEntry, state.machine_state)
        }
    };
    // check and see if machine state has changed
    if state.prev_state.0 != state.machine_state.0 || state.prev_state.1 != state.machine_state.1 {
        //#[cfg(debug_assertions)]
        //serial::send_tuple(&mut state);
        state.prev_state = state.machine_state;
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
        stbd_stop_pin: state.stbd_stop_pin,
        port_stop_pin: state.port_stop_pin,
        adc: state.adc,
        led_pin: state.led_pin,
        wdt: state.wdt,
        eeprom: state.eeprom,
        machine_state: newstate,
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

static mut PENDINGPCINT2: bool = false;

#[interrupt(atmega328p)]
fn PCINT2() {
    unsafe {
        ptr::write_volatile(&mut PENDINGPCINT2, true);
    }
}

//=====================================================
