//! firmware for pypilot controller board

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

#[cfg(not(debug_assertions))]
use panic_halt as _;

use arduino_hal::hal::{
    pac, port,
    port::{mode, Pin},
};
use avr_device::interrupt;
use avr_hal_generic::pwm::Prescaler;
use core::{convert::TryInto, ptr};

// modules from project
mod adc;
use adc::{AdcMvgAvgIndex, AdcResults};
#[macro_use]
extern crate bitflags;
mod crc;
mod packet;
use packet::OutgoingPacketState;
mod eeprom;
mod nanopwm;
use nanopwm::PwmPin;
#[cfg(feature = "serial_packets")]
mod serial;

#[panic_handler]
fn panic(_pi: &core::panic::PanicInfo<'_>) -> ! {
    unsafe {
        if let Some(led1) = &mut LED1_PIN {
            led1.set_low();
        }
        if let Some(led2) = &mut LED2_PIN {
            led2.set_low();
        }
        if let Some(led3) = &mut LED3_PIN {
            led3.set_low();
        }
        if let Some(led4) = &mut LED4_PIN {
            if adc::sample_overflow() {
                led4.set_high();
            } else {
                led4.set_low();
            }
        }
        if let Some(led5) = &mut LED5_PIN {
            if serial::write_overflow() {
                led5.set_high();
            } else {
                led5.set_low();
            }
        }
        if let Some(led6) = &mut LED6_PIN {
            if serial::read_overflow() {
                led6.set_high();
            } else {
                led6.set_low();
            }
        }
    }
    loop {}
}

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
    const READQUEUEOVF = 0b0010_0000_0000_0000;
    const WRITEQUEUEOVF = 0b0100_0000_0000_0000;
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
    cpu: arduino_hal::pac::CPU,
    exint: arduino_hal::pac::EXINT,
    timer0: nanopwm::Timer0Pwm,
    timer2: arduino_hal::pac::TC2,
    //ena_pin: Pin<mode::Input<mode::PullUp>, port::PD4>,
    //enb_pin: Pin<mode::Input<mode::PullUp>, port::PD5>,
    //ina_pin: Pin<mode::Output, port::PD2>,
    //inb_pin: Pin<mode::Output, port::PD3>,
    pwm_pin: nanopwm::PwmPin<Pin<mode::Output, port::PD6>>,
    //stbd_stop_pin: Pin<mode::Input<mode::PullUp>, port::PD7>,
    //port_stop_pin: Pin<mode::Input<mode::PullUp>, port::PB0>,
    //    wdt: hal::wdt::Wdt,
    wdt: avr_device::atmega328p::WDT,
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
    adc_results: AdcResults,
}

//==========================================================

static mut LED1_PIN: Option<Pin<mode::Output, port::PB5>> = None;
static mut LED2_PIN: Option<Pin<mode::Output, port::PB0>> = None;
static mut LED3_PIN: Option<Pin<mode::Output, port::PD7>> = None;
static mut LED4_PIN: Option<Pin<mode::Output, port::PD5>> = None;
static mut LED5_PIN: Option<Pin<mode::Output, port::PD4>> = None;
static mut LED6_PIN: Option<Pin<mode::Output, port::PD3>> = None;
static mut LED7_PIN: Option<Pin<mode::Output, port::PD2>> = None;

//==========================================================

#[arduino_hal::entry]
fn main() -> ! {
    let mut state = setup();

    loop {
        avr_device::asm::wdr();
        //state.wdt.feed();
        // check for watchdog interrupt
        if unsafe { ptr::read_volatile(&WDT_TRIGGERED) } {
            interrupt::free(|_| {
                // follow timed sequence, to change the wdt
                state
                    .wdt
                    .wdtcsr
                    .modify(|_, w| w.wdce().set_bit().wde().set_bit());
                // set reset enabled, and to 32K cycles
                state
                    .wdt
                    .wdtcsr
                    //                    .write(|w| w.wde().set_bit().wdpl().cycles_32k());
                    .write(|w| w.wde().set_bit());
                // loop until reset
                loop {}
            });
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
        state = process_fsm(state);
        state.adc_results.process();

        // out serial processing
        #[cfg(feature = "serial_packets")]
        serial::send_serial(state.outgoing_state);
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
    let dp = arduino_hal::Peripherals::take().unwrap();
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
    let pins = arduino_hal::pins!(dp);

    //let stbd_stop_pin = pins.d7.into_pull_up_input();
    //let port_stop_pin = pins.d8.into_pull_up_input();

    #[cfg(not(debug_assertions))]
    #[cfg(not(feature = "serial_packets"))]
    pins.d0.into_pull_up_input();
    #[cfg(not(debug_assertions))]
    #[cfg(not(feature = "serial_packets"))]
    pins.d1.into_pull_up_input();

    // turn on timer2
    let timer2 = dp.TC2;
    timer2.tccr2b.modify(|_, w| w.cs2().prescale_1024());

    // pwm
    let mut timer0 = nanopwm::Timer0Pwm::new(dp.TC0, Prescaler::Prescale8);
    let pwm_pin = PwmPin::into_pwm(pins.d6.into_output(), &mut timer0);
    // clockwise
    let ina_pin = pins.d2.into_output();
    // counterclockwise
    let inb_pin = pins.d3.into_output();

    // half-bridge fault pins, also enable pins, but carrier has them pulled high
    // already
    //let ena_pin = pins.d4.into_pull_up_input();
    //let enb_pin = pins.d5.into_pull_up_input();

    // SPI pins
    //let _sck = pins.d13.into_pull_up_input(&mut pins.ddr);
    let _miso = pins.d12.into_output();
    let _mosi = pins.d11.into_pull_up_input();
    let _cs = pins.d10.into_pull_up_input();
    let _can_int = pins.d9.into_floating_input();

    // debugging pins
    unsafe {
        LED1_PIN = Some(pins.d13.into_output());
        LED2_PIN = Some(pins.d8.into_output());
        LED3_PIN = Some(pins.d7.into_output());
        LED4_PIN = Some(pins.d5.into_output());
        LED5_PIN = Some(pins.d4.into_output());
        LED6_PIN = Some(inb_pin);
        LED7_PIN = Some(ina_pin);
        if let Some(led2) = &mut LED2_PIN {
            led2.set_high();
        }
    }
    // now module setup

    // setup serial
    #[cfg(feature = "serial_packets")]
    let uart = arduino_hal::default_serial!(dp, pins, 57600);
    #[cfg(feature = "serial_packets")]
    interrupt::free(|cs| serial::init(&cs, uart));

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

    // setup adc_results
    let mut adc_results = AdcResults::new();
    interrupt::free(|cs| adc_results.init(&cs, adc));

    // watchdog
    let wdt = dp.WDT;
    interrupt::free(|_cs| {
        cpu.mcusr.modify(|_, w| w.wdrf().clear_bit());
        avr_device::asm::wdr();
        // follow timed sequence to change wdt
        wdt.wdtcsr.modify(|_, w| w.wdce().set_bit().wde().set_bit());
        // no watchdog reset, set watchdog interrupt
        wdt.wdtcsr
            .write(|w| w.wde().clear_bit().wdie().set_bit().wdpl().cycles_32k());
    });

    unsafe {
        if let Some(led3) = &mut LED3_PIN {
            led3.set_high();
        }
        interrupt::enable();
    }

    State {
        cpu: cpu,
        exint: dp.EXINT,
        timer0: timer0,
        timer2: timer2,
        //ena_pin: ena_pin,
        //enb_pin: enb_pin,
        //ina_pin: ina_pin,
        //inb_pin: inb_pin,
        pwm_pin: pwm_pin,
        //stbd_stop_pin: stbd_stop_pin,
        //port_stop_pin: port_stop_pin,
        wdt: wdt, //watchdog,
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
        adc_results: adc_results,
    }
}

//==========================================================

/// power down the cpu
fn power_down_mode(state: &mut State) {
    // delay for a bit, so that usart can finish any transmission
    #[cfg(any(debug_assertions, feature = "serial_packets"))]
    arduino_hal::delay_ms(10);

    // disable modules
    // adc off
    let adc = unsafe { &*<pac::ADC>::ptr() };
    adc.adcsra
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
    if let Some(led6) = unsafe { &mut LED6_PIN } {
        led6.toggle();
    }
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
        if state.stbd_stop_pin.is_low() {
            state.flags.insert(Flags::STBDPINFAULT);
            do_stop = true;
        } else {
            state.flags.remove(Flags::STBDPINFAULT);
        }
        if state.port_stop_pin.is_low() {
            state.flags.insert(Flags::PORTPINFAULT);
            do_stop = true;
        } else {
            state.flags.remove(Flags::PORTPINFAULT);
        }

        // test fault pins
        if state.ena_pin.is_low() || state.enb_pin.is_low() {
            state.flags.insert(Flags::OVERCURRENTFAULT);
            do_stop = true;
        } else {
            state.flags.remove(Flags::OVERCURRENTFAULT);
        }
    */
    // test current
    if let Some(amps) = state.adc_results.get_current(400, AdcMvgAvgIndex::Second) {
        if amps >= MAX_CURRENT {
            do_stop = true;
            state.flags.insert(Flags::OVERCURRENTFAULT);
        } else {
            state.flags.remove(Flags::OVERCURRENTFAULT);
        }
    }
    // test voltage
    if let Some(volts) = state.adc_results.get_voltage(400, AdcMvgAvgIndex::Second) {
        if volts <= 900 || volts >= MAX_VOLTAGE {
            do_stop = true;
            state.flags.insert(Flags::BADVOLTAGEFAULT);
        } else {
            state.flags.remove(Flags::BADVOLTAGEFAULT);
        }
    }

    // schedule stop if asked
    if do_stop {
        state.pending_cmd = CommandToExecute::Stop;
    }
}

//==========================================================

/// test motor or controller temp
#[cfg(any(feature = "motor_temp", feature = "controller_temp"))]
fn test_overtemp(state: &mut State) {
    let ccnt;
    let mcnt;
    unsafe {
        ccnt = AdcResults.count(adc::AdcChannel::ControllerTemp, 1);
        mcnt = AdcResults.count(adc::AdcChannel::MotorTemp, 1);
    }
    if ccnt > 100 && mcnt > 100 {
        let ctemp;
        let mtemp;
        unsafe {
            ctemp = AdcResults.take_controller_temp(1);
            mtemp = AdcResults.take_motor_temp(1);
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
    /*
    if value > 1040 {
        state.ina_pin.set_low();
        state.inb_pin.set_high();
    } else if value < 960 {
        state.ina_pin.set_high();
        state.inb_pin.set_low();
    } else {
        // set brake
        state.ina_pin.set_low();
        state.inb_pin.set_low();
    }
     */
    unsafe {
        if let Some(led4) = &mut LED4_PIN {
            if let Some(led5) = &mut LED5_PIN {
                if value > 1040 {
                    led4.set_low();
                    led5.set_high();
                } else if value < 960 {
                    led4.set_high();
                    led5.set_low();
                } else {
                    // set brake
                    led4.set_low();
                    led5.set_low();
                }
            }
        }
    }
}

//==========================================================

/// the main state machine processer
fn process_fsm(mut state: State) -> State {
    // check timer2 counter, boolean 'do_update' s/b set
    // if motor position to be updated
    // timer2 is set to 1024 prescale, so each timer tick=15625hz=64ns
    // that means each update is at 5ms
    static mut LED_CNT: u8 = 0;
    let do_update = {
        if state.timer2.tcnt2.read().bits() > 78 {
            state.timeout += 1;
            state.comm_timeout += 1;
            unsafe {
                interrupt::free(|_| state.timer2.tcnt2.modify(|r, w| w.bits(r.bits() - 78)));
            }
            unsafe {
                if LED_CNT == 100 {
                    LED_CNT = 0;
                    if let Some(led1) = &mut LED1_PIN {
                        led1.toggle();
                    }
                } else {
                    LED_CNT += 1;
                }
            }
            true
        } else {
            false
        }
    };

    // process packet
    match state.pending_cmd {
        CommandToExecute::ProcessPacket(pkt) => packet::process_packet(pkt, &mut state),
        _ => {}
    }

    if do_update {
        housekeeping(&mut state);
    }

    // check for stop
    if state.pending_cmd == CommandToExecute::Stop {
        stop(&mut state);
    }

    // build outgoing packet if asked
    if state.outgoing_state == OutgoingPacketState::Build {
        state.outgoing_state = packet::build_outgoing(&mut state);
    }
    // FSM
    let newstate = match state.machine_state.0 {
        PyPilotStateMachine::Start => {
            change_state(PyPilotStateMachine::WaitEntry, state.machine_state)
        }
        PyPilotStateMachine::WaitEntry => {
            state.timeout = 0;
            //state.ina_pin.set_low();
            //state.inb_pin.set_low();
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
            //state.ina_pin.set_low();
            //state.inb_pin.set_low();
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
            //state.ina_pin.set_low();
            //state.inb_pin.set_low();
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
        //ena_pin: state.ena_pin,
        //enb_pin: state.enb_pin,
        //ina_pin: state.ina_pin,
        //inb_pin: state.inb_pin,
        pwm_pin: state.pwm_pin,
        //stbd_stop_pin: state.stbd_stop_pin,
        //port_stop_pin: state.port_stop_pin,
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
        adc_results: state.adc_results,
    }
}

//==========================================================

static mut WDT_TRIGGERED: bool = false;

#[interrupt(atmega328p)]
fn WDT() {
    unsafe {
        ptr::write_volatile(&mut WDT_TRIGGERED, true);
        if let Some(led7) = &mut LED7_PIN {
            led7.set_high();
        }
    }
    /*
        wdt_reset();
        wdt_disable();
        if(!calculated_clock) {
            calculated_clock = 1; // use watchdog interrupt once at startup to compute the crystal's frequency
            return;
        }
        // normal watchdog event (program stuck)
        disengage();
        _delay_ms(50);
        detach();

        asm volatile ("ijmp" ::"z" (0x0000)); // soft reset
    */
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
