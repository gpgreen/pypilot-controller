//! firmware for pypilot controller board

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

extern crate panic_halt;

use avr_device::interrupt;
use core::cell::RefCell;
use core::convert::TryInto;
use core::ptr;
use hal::port::mode::{Floating, Input, Output, Pwm};
use pypilot_controller_board::hal;
use pypilot_controller_board::prelude::*;
use pypilot_controller_board::pwm;
use pypilot_controller_board::pwm::Timer0Pwm;

// modules from project
mod crc;
mod packet;
use packet::OutgoingPacketState;
#[cfg(feature = "serial_packets")]
mod serial;

//==========================================================
// Hardware.flags property masks

const SYNC: u16 = 1;
const OVERTEMPFAULT: u16 = 2;
const OVERCURRENTFAULT: u16 = 4;
const ENGAGED: u16 = 8;
const INVALID: u16 = 16;
const PORTPINFAULT: u16 = 32;
const STBDPINFAULT: u16 = 64;
const BADVOLTAGEFAULT: u16 = 128;
const MINRUDDERFAULT: u16 = 256;
const MAXRUDDERFAULT: u16 = 512;
const CURRENTRANGE: u16 = 1024;
const BADFUSES: u16 = 2048;
const REBOOTED: u16 = 32768;

//==========================================================

/// using low or high current
const LOWCURRENT: bool = true;

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

/// adc channels being monitored
#[derive(Debug, Copy, Clone)]
pub enum AdcChannel {
    Current,
    Voltage,
    ControllerTemp,
    MotorTemp,
    Rudder,
}

impl From<AdcChannel> for usize {
    fn from(original: AdcChannel) -> usize {
        match original {
            AdcChannel::Current => 0,
            AdcChannel::Voltage => 1,
            AdcChannel::ControllerTemp => 2,
            AdcChannel::MotorTemp => 3,
            AdcChannel::Rudder => 4,
        }
    }
}

/// adc sample moving averages
struct AdcMvgAvg {
    total: [u32; 2],
    count: [u16; 2],
}

/// adc sample data for each channel, selected channel, enabled channels
pub struct AdcResults {
    enabled: u8,
    selected: AdcChannel,
    data: [AdcMvgAvg; 5],
}

impl AdcResults {
    /// setup the array, used for setting the enable flags
    fn setup(&mut self) {
        // setup the enabled flags, if it hasn't yet been done
        if self.enabled == 0 {
            // Current and Voltage are always on
            let mut enabled_flags = 0x3;
            if cfg!(controller_temp) {
                enabled_flags |= 0x4;
            }
            if cfg!(motor_temp) {
                enabled_flags |= 0x8;
            }
            if cfg!(rudder_angle) {
                enabled_flags |= 0x10;
            }
            self.enabled = enabled_flags;
        }
    }

    /// convert the selected channel to an index into data array
    #[inline]
    fn selected_channel(&self) -> usize {
        self.selected.into()
    }

    /// is a channel enabled
    #[inline]
    fn is_channel_enabled(&self, channel: AdcChannel) -> bool {
        match channel {
            AdcChannel::Current => self.enabled & 0x1 == 0x1,
            AdcChannel::Voltage => self.enabled & 0x2 == 0x2,
            AdcChannel::ControllerTemp => self.enabled & 0x4 == 0x4,
            AdcChannel::MotorTemp => self.enabled & 0x8 == 0x8,
            AdcChannel::Rudder => self.enabled & 0x10 == 0x10,
        }
    }

    /// get the adc mux register value for selected channel
    #[inline]
    fn adc_mux(&self) -> u8 {
        //let mux = _BV(REFS0) | _BV(REFS1);
        let mux = 0xC0
            & match self.selected {
                // Current is on ADC1
                AdcChannel::Current => 0x1,
                // Voltage on pin ADC0
                AdcChannel::Voltage => 0x0,
                // Controller Temp on pin ADC2
                AdcChannel::ControllerTemp => 0x2,
                // Motor Temp on pin ADC3
                AdcChannel::MotorTemp => 0x4,
                // Rudder angle on pin ADC4
                AdcChannel::Rudder => 0x8,
            };
        mux
    }

    /// switch to the next adc channel
    #[inline]
    fn next_channel(&mut self) {
        // get the next 'enabled' channel
        let mut sel = self.selected;
        loop {
            let nxt = match sel {
                AdcChannel::Current => AdcChannel::Voltage,
                AdcChannel::Voltage => AdcChannel::ControllerTemp,
                AdcChannel::ControllerTemp => AdcChannel::MotorTemp,
                AdcChannel::MotorTemp => AdcChannel::Rudder,
                AdcChannel::Rudder => AdcChannel::Current,
            };
            if self.is_channel_enabled(nxt) {
                self.selected = nxt;
                break;
            } else {
                sel = nxt;
            }
        }
    }

    /// count the results in a channel array
    pub fn count(&self, channel: AdcChannel, narray: usize) -> u16 {
        let i: usize = channel.into();
        self.data[i].count[narray]
    }

    /// get result in a channel array
    fn take(&mut self, channel: AdcChannel, narray: usize) -> u16 {
        let i: usize = channel.into();
        let (cnt, val) = interrupt::free(|_cs| {
            let cnt = self.data[i].count[narray];
            let val = self.data[i].total[narray];
            self.data[i].total[narray] = 0;
            self.data[i].count[narray] = 0;
            (cnt, val)
        });
        if cnt == 0 {
            0
        } else {
            let avg: u32 = 16 * val / cnt as u32;
            avg as u16
        }
    }

    /// get current from sample data
    pub fn take_amps(&mut self, narray: usize) -> u16 {
        let v = self.take(AdcChannel::Current, narray);
        v * 9 / 34 / 16
    }

    /// get voltage from sample data
    pub fn take_volts(&mut self, narray: usize) -> u16 {
        // voltage in 10mV increments 1.1ref, 560 and 10k resistors
        let v = self.take(AdcChannel::Voltage, narray);
        v * 1790 / 896 / 16
    }

    /// convert a raw mvg avg value to thermistor temperature
    #[cfg(any(feature = "controller_temp", feature = "motor_temp"))]
    fn thermistor_conversion(&self, raw: u32) -> u16 {
        let r = 100061 * raw / (74464 - raw);
        (30000000 / (r + 2600) + 200) as u16
    }

    /// get controller temperature from sample data
    #[cfg(feature = "controller_temp")]
    pub fn take_controller_temp(&mut self, narray: usize) -> u16 {
        let v: u32 = self.take(AdcChannel::ControllerTemp, narray).into();
        self.thermistor_conversion(v)
    }

    /// get motor temperature from sample data
    #[cfg(feature = "motor_temp")]
    pub fn take_motor_temp(&mut self, narray: usize) -> u16 {
        let v: u32 = self.take(AdcChannel::MotorTemp, narray).into();
        self.thermistor_conversion(v)
    }

    /// get rudder angle from sample data
    #[cfg(feature = "rudder_angle")]
    pub fn take_rudder(&mut self, narray: usize) -> u16 {
        self.take(AdcChannel::Rudder, narray) * 4
    }
}

/// static variable for AdcResults and initializer
static mut ADC_RESULTS: AdcResults = AdcResults {
    // first channel is current
    selected: AdcChannel::Current,
    // which channels are enabled
    enabled: 0,
    // AdcMvgAvg struct for each channel
    data: [
        AdcMvgAvg {
            total: [0; 2],
            count: [0; 2],
        },
        AdcMvgAvg {
            total: [0; 2],
            count: [0; 2],
        },
        AdcMvgAvg {
            total: [0; 2],
            count: [0; 2],
        },
        AdcMvgAvg {
            total: [0; 2],
            count: [0; 2],
        },
        AdcMvgAvg {
            total: [0; 2],
            count: [0; 2],
        },
    ],
};

//==========================================================

/// the state variables for the hardware
pub struct State {
    // hardware stuff
    cpu: pypilot_controller_board::pac::CPU,
    exint: pypilot_controller_board::pac::EXINT,
    timer0: pypilot_controller_board::pwm::Timer0Pwm,
    timer2: pypilot_controller_board::pac::TC2,
    adc: RefCell<avr_device::atmega328p::ADC>,
    ena_pin: hal::port::portd::PD4<Input<Floating>>,
    enb_pin: hal::port::portd::PD5<Input<Floating>>,
    ina_pin: hal::port::portd::PD2<Output>,
    inb_pin: hal::port::portd::PD3<Output>,
    pwm_pin: hal::port::portd::PD6<Pwm<Timer0Pwm>>,
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
    flags: u16,
    pending_cmd: CommandToExecute,
    outgoing_state: OutgoingPacketState,
}

//==========================================================

#[hal::entry]
fn main() -> ! {
    let mut state = setup();
    loop {
        state = serial::process_serial(state);
        // check if we need to process packet
        state = match state.pending_cmd {
            CommandToExecute::ProcessPacket(pkt) => packet::process_packet(pkt, state),
            _ => state,
        };
        // deal with packet induced stop's
        match state.pending_cmd {
            CommandToExecute::Stop => stop(&mut state),
            _ => {}
        };
        // build outgoing packet if asked
        if state.outgoing_state == OutgoingPacketState::Build {
            state = packet::build_outgoing(state);
        }
        state = process_fsm(state);
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

    // turn on pullups on unused pins
    pins.d7.into_pull_up_input(&mut pins.ddr);
    pins.d8.into_pull_up_input(&mut pins.ddr);

    #[cfg(not(debug_assertions))]
    #[cfg(not(feature = "serial_packets"))]
    let _rx = pins.rx.into_pull_up_input(&mut pins.ddr);
    #[cfg(not(debug_assertions))]
    #[cfg(not(feature = "serial_packets"))]
    let _tx = pins.tx.into_pull_up_input(&mut pins.ddr);

    // turn on timer2
    let timer2 = dp.TC2;
    timer2.tccr2b.modify(|_, w| w.cs2().prescale_1024());

    // pwm
    let mut timer0 = pwm::Timer0Pwm::new(dp.TC0, pwm::Prescaler::Prescale8);
    let pwm_pin = pins.d6.into_output(&mut pins.ddr).into_pwm(&mut timer0);
    // clockwise
    let ina_pin = pins.d2.into_output(&mut pins.ddr);
    // counterclockwise
    let inb_pin = pins.d3.into_output(&mut pins.ddr);

    // half-bridge fault pins, also enable pins, but carrier has them pulled high
    // already
    let ena_pin = pins.d4.into_floating_input(&mut pins.ddr);
    let enb_pin = pins.d5.into_floating_input(&mut pins.ddr);

    // SPI pins
    let _sck = pins.d13.into_pull_up_input(&mut pins.ddr);
    let _miso = pins.d12.into_output(&mut pins.ddr);
    let _mosi = pins.d11.into_pull_up_input(&mut pins.ddr);
    let _cs = pins.d10.into_pull_up_input(&mut pins.ddr);
    let _can_int = pins.d9.into_floating_input(&mut pins.ddr);

    // now module setup

    // setup serial
    #[cfg(any(debug_assertions, feature = "serial_packets"))]
    let mut serial = pypilot_controller_board::Serial::<Floating>::new(
        dp.USART0,
        pins.rx,
        pins.tx.into_output(&mut pins.ddr),
        57600.into_baudrate(),
    );
    // turn on RX interrupt
    #[cfg(any(debug_assertions, feature = "serial_packets"))]
    serial.listen(hal::usart::Event::RxComplete);
    // split into reader and write and send to static variables
    #[cfg(any(debug_assertions, feature = "serial_packets"))]
    let (reader, writer) = serial.split();
    #[cfg(any(debug_assertions, feature = "serial_packets"))]
    serial::setup_serial(reader, writer);

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
    unsafe {
        // SAFETY: ok because interrupts aren't yet on
        ADC_RESULTS.setup();
        interrupt::enable();
    }
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
        adc: RefCell::new(adc),
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
        flags: 0x00,
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

/// update the pwm based on motor command
fn position(state: &mut State, value: u16) {
    // foPWM = fclk/(prescale*256)
    // foPWM = 3.906kHz
    let newpos = if value >= 1000 {
        (value - 1000) / 4
    } else {
        (1000 - value) / 4
    };
    state.pwm_pin.set_duty(newpos.try_into().unwrap());
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
    /*
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
    */
}

//==========================================================

/// the main state machine processer
fn process_fsm(mut state: State) -> State {
    // first check the times, set do_update boolean
    // if motor position should be updated
    let do_update = interrupt::free(|_cs| {
        let ticks = state.timer2.tcnt2.read().bits();
        if ticks > 78 {
            state.timeout += 1;
            state.comm_timeout += 1;
            unsafe {
                state.timer2.tcnt2.modify(|r, w| w.bits(r.bits() - 78));
            }
            true
        } else {
            false
        }
    });

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
            state.flags |= ENGAGED;
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
                let speed_rate: i16 = state.max_slew_speed.into();
                // value of 20 is 1 second full range at 50hz
                let slow_rate: i16 = state.max_slew_slow.into();
                let cur_value: u16 = state.lastpos;
                // this should always be in range
                let mut diff: i16 = (state.command_value - cur_value).try_into().unwrap();

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
                let mut new_pos = cur_value;
                if diff < 0 {
                    if (-diff) as u16 > cur_value {
                        new_pos = 0;
                    } else {
                        new_pos -= (-diff) as u16;
                    }
                } else if diff > 0 {
                    if diff as u16 + cur_value > 2000 {
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
            state.flags &= !ENGAGED;
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
            if state.timeout > 62 && state.comm_timeout > 250 {
                change_state(PyPilotStateMachine::PowerDown, state.machine_state)
            } else if state.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, state.machine_state)
            } else {
                state.machine_state
            }
        }
        PyPilotStateMachine::PowerDown => {
            //#[cfg(any(debug_assertions, feature = "serial_packets"))]
            //nb::block!(state.serial.flush()).void_unwrap();
            // cpu goes to sleep, then woken up
            power_down_mode(&mut state);
            state.timeout = 0;
            state.comm_timeout = state.comm_timeout - 250;
            state.command_value = 1000;
            state.lastpos = 1000;
            state.pending_cmd = CommandToExecute::None;
            change_state(PyPilotStateMachine::WaitEntry, state.machine_state)
        }
    };
    // check and see if machine state has changed, if so, send it via serial port
    if state.prev_state.0 != state.machine_state.0 || state.prev_state.1 != state.machine_state.1 {
        //#[cfg(any(debug_assertions, feature = "serial_packets"))]
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
        adc: state.adc,
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
        pending_cmd: CommandToExecute::None,
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

//==========================================================

#[interrupt(atmega328p)]
fn ADC() {
    static mut SAMPLE_CNT: u8 = 0;
    // unsafe is ok here, because we are in interrupt context
    // avr interrupts aren't nesting
    // we are modifying the ADC_RESULTS static structure
    // so can't reenable interrupts till this isr is finished
    // we are using the raw pointer to ADC registers also, due
    // to the same reason
    unsafe {
        // magic address
        const ADCW_REG: u16 = 0x78;
        // get the sample value
        let sample = ptr::read_volatile(ADCW_REG as *mut u16);
        // add the sample to the array
        let sel = ADC_RESULTS.selected_channel();
        SAMPLE_CNT += 1;
        // throw out the first 3 samples
        if SAMPLE_CNT > 3 {
            for i in 0..2 {
                if ADC_RESULTS.data[sel].count[i] < 4000 {
                    ADC_RESULTS.data[sel].total[i] += sample as u32;
                    ADC_RESULTS.data[sel].count[i] += 1;
                }
            }
        }
        // check to see if we need more samples for this channel
        let do_more = match ADC_RESULTS.selected {
            AdcChannel::Current => SAMPLE_CNT < 50,
            AdcChannel::Voltage => SAMPLE_CNT < 8,
            AdcChannel::Rudder => SAMPLE_CNT < 16,
            _ => false,
        };
        if !do_more {
            SAMPLE_CNT = 0;
            ADC_RESULTS.next_channel();
            // magic address
            const ADCMUX_REG: u8 = 0x7C;
            // write the mux
            ptr::write_volatile(ADCMUX_REG as *mut u8, ADC_RESULTS.adc_mux());
        }
        // do the next conversion
        // magic address
        const ADCSRA_REG: u8 = 0x7A;
        ptr::write_volatile(ADCSRA_REG as *mut u8, 0xC8);
    }
}

//==========================================================
