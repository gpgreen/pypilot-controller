//! firmware for pypilot controller board

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

extern crate panic_halt;

use avr_device::interrupt;
use core::cell::RefCell;
use core::convert::TryInto;
use core::ptr;
use hal::port::mode::{Analog, Floating, Input, Output, Pwm};
use pypilot_controller_board::hal;
use pypilot_controller_board::prelude::*;
use pypilot_controller_board::pwm;
use pypilot_controller_board::pwm::Timer0Pwm;

#[cfg(debug_assertions)]
use ufmt;

#[cfg(debug_assertions)]
mod serial;
#[cfg(debug_assertions)]
use serial::{process_packet, process_serial};

mod utility;
#[cfg(debug_assertions)]
use utility::*;

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

//==========================================================

/// using low or high current
const LOWCURRENT: bool = true;

//==========================================================

/// command to execute
/// in a function that changes hardware state variables,
/// use this enum to flag a function that needs to be
/// executed following. Also used in state transitions
#[derive(Clone, Copy, PartialEq)]
pub enum CommandToExecute {
    Stop,
    Engage,
    Disengage,
    ProcessPacket([u8; 3]),
    None,
}

//==========================================================

/// adc channels being monitored
#[derive(Copy, Clone)]
enum AdcChannel {
    Current,
    Voltage,
    ControllerTemp,
    MotorTemp,
    Rudder,
}

/// struct for storing data for adc channel sample moving averages
struct AdcMvgAvg {
    total: [u32; 2],
    count: [u16; 2],
}

/** struct to hold adc samples for each channel, which channel is
currently being sampled
*/
struct AdcResults {
    enabled: u8,
    selected: AdcChannel,
    data: [AdcMvgAvg; 5],
}

impl AdcResults {
    /// convert the selected channel to an index into data array
    #[inline]
    fn selected_channel(&self) -> usize {
        match self.selected {
            AdcChannel::Current => 0,
            AdcChannel::Voltage => 1,
            AdcChannel::ControllerTemp => 2,
            AdcChannel::MotorTemp => 3,
            AdcChannel::Rudder => 4,
        }
    }

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

pub struct Hardware {
    // hardware stuff
    cpu: pypilot_controller_board::pac::CPU,
    exint: pypilot_controller_board::pac::EXINT,
    timer0: pypilot_controller_board::pwm::Timer0Pwm,
    timer2: pypilot_controller_board::pac::TC2,
    adc: RefCell<pypilot_controller_board::adc::Adc>,
    ena_pin: hal::port::portd::PD4<Input<Floating>>,
    enb_pin: hal::port::portd::PD5<Input<Floating>>,
    ina_pin: hal::port::portd::PD2<Output>,
    inb_pin: hal::port::portd::PD3<Output>,
    pwm_pin: hal::port::portd::PD6<Pwm<Timer0Pwm>>,
    a1: hal::port::portc::PC1<Analog>,
    #[cfg(debug_assertions)]
    serial: RefCell<pypilot_controller_board::Serial<Floating>>,
    // state stuff
    machine_state: (PyPilotStateMachine, PyPilotStateMachine),
    prev_state: (PyPilotStateMachine, PyPilotStateMachine),
    timeout: u16,
    serial_data_timeout: u16,
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
    serialin: u8,
    flags: u16,
    pending_cmd: CommandToExecute,
}

//==========================================================

#[hal::entry]
fn main() -> ! {
    let mut hdwr = setup();
    loop {
        hdwr = process_serial(hdwr);
        hdwr = match hdwr.pending_cmd {
            CommandToExecute::ProcessPacket(pkt) => process_packet(pkt, hdwr),
            _ => hdwr,
        };
        hdwr = match hdwr.pending_cmd {
            CommandToExecute::Stop => stop(hdwr),
            _ => hdwr,
        };
        hdwr = process_fsm(hdwr);
    }
}

//==========================================================
// INT0 Flag

static mut PENDINGINT0: bool = false;

//==========================================================

#[derive(Copy, Clone, PartialEq)]
enum PyPilotStateMachine {
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

#[cfg(debug_assertions)]
impl PyPilotStateMachine {
    fn send(&self, serial: &mut pypilot_controller_board::Serial<Floating>) {
        let s = match self {
            PyPilotStateMachine::Start => r"Start",
            PyPilotStateMachine::WaitEntry => r"WaitEntry",
            PyPilotStateMachine::Wait => r"Wait",
            PyPilotStateMachine::Engage => r"Engage",
            PyPilotStateMachine::Operational => r"Operational",
            PyPilotStateMachine::DisengageEntry => r"DisengageEntry",
            PyPilotStateMachine::Disengage => r"Disengage",
            PyPilotStateMachine::DetachEntry => r"DetachEntry",
            PyPilotStateMachine::Detach => r"Detach",
            PyPilotStateMachine::PowerDown => r"PowerDown",
        };
        ufmt::uwrite!(serial, "{}", s).void_unwrap();
    }
}

#[cfg(debug_assertions)]
fn send_tuple(
    state: (PyPilotStateMachine, PyPilotStateMachine),
    serial: &mut pypilot_controller_board::Serial<Floating>,
) {
    ufmt::uwrite!(serial, "Current:").void_unwrap();
    state.0.send(serial);
    ufmt::uwrite!(serial, " Previous:").void_unwrap();
    state.1.send(serial);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}

//==========================================================

/// setup the hardware for the main loop
fn setup() -> Hardware {
    let dp = pypilot_controller_board::Peripherals::take().unwrap();

    // turn off unused modules
    let cpu = dp.CPU;
    cpu.prr.write(|w| {
        w.prtim1().set_bit();
        w.prtwi().set_bit()
    });
    #[cfg(not(debug_assertions))]
    cpu.prr.modify(|_, w| w.prusart0().set_bit());
    // turn off analog comparator
    let ac = dp.AC;
    ac.acsr.write(|w| w.acd().set_bit());

    // sort out the pins
    let mut pins = pypilot_controller_board::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);

    // turn on pullups on unused pins (proto-board, no unused pins)
    #[cfg(not(debug_assertions))]
    let _rx = pins.rx.into_pull_up_input(&mut pins.ddr);
    #[cfg(not(debug_assertions))]
    let _tx = pins.tx.into_pull_up_input(&mut pins.ddr);

    // turn on timer2
    let timer2 = dp.TC2;
    timer2.tccr2b.modify(|_, w| w.cs2().prescale_1024());

    // pwm
    let mut timer0 = pwm::Timer0Pwm::new(dp.TC0, pwm::Prescaler::Prescale8);
    let pwm_pin = pins.d6.into_output(&mut pins.ddr).into_pwm(&mut timer0);
    let ina_pin = pins.d2.into_output(&mut pins.ddr);
    let inb_pin = pins.d3.into_output(&mut pins.ddr);

    // SPI pins
    let _sck = pins.d13.into_pull_up_input(&mut pins.ddr);
    let _miso = pins.d12.into_output(&mut pins.ddr);
    let _mosi = pins.d11.into_pull_up_input(&mut pins.ddr);
    let _cs = pins.d10.into_pull_up_input(&mut pins.ddr);

    // now module setup

    // setup serial
    #[cfg(debug_assertions)]
    let mut serial = pypilot_controller_board::Serial::<Floating>::new(
        dp.USART0,
        pins.rx,
        pins.tx.into_output(&mut pins.ddr),
        57600.into_baudrate(),
    );

    // setup adc, default is 128 clock division, and AVcc voltage reference
    let mut adc = pypilot_controller_board::adc::Adc::new(dp.ADC, Default::default());

    // adc pins
    let a1 = pins.a1.into_analog_input(&mut adc);

    // enable interrupts
    unsafe {
        interrupt::enable();
    }

    // print out some status
    #[cfg(debug_assertions)]
    ufmt::uwriteln!(&mut serial, "\r\nPyPilot Controller Start\r").void_unwrap();

    #[cfg(debug_assertions)]
    ufmt::uwrite!(serial, "spcr:").void_unwrap();
    #[cfg(debug_assertions)]
    send_reg(&mut serial, 0x4c);
    #[cfg(debug_assertions)]
    ufmt::uwrite!(serial, "prr:").void_unwrap();
    #[cfg(debug_assertions)]
    send_reg(&mut serial, 0x64);

    Hardware {
        cpu: cpu,
        exint: dp.EXINT,
        timer0: timer0,
        timer2: timer2,
        ena_pin: pins.d4,
        enb_pin: pins.d5,
        ina_pin: ina_pin,
        inb_pin: inb_pin,
        pwm_pin: pwm_pin,
        adc: RefCell::new(adc),
        a1: a1,
        #[cfg(debug_assertions)]
        serial: RefCell::new(serial),
        machine_state: (PyPilotStateMachine::Start, PyPilotStateMachine::Start),
        prev_state: (PyPilotStateMachine::Start, PyPilotStateMachine::Start),
        timeout: 0,
        serial_data_timeout: 250,
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
        serialin: 0,
        flags: 0x00,
        pending_cmd: CommandToExecute::None,
    }
}

//==========================================================

/// power down the cpu
fn power_down_mode(hdwr: Hardware) -> Hardware {
    // delay for a bit, so that usart can finish any transmission
    #[cfg(debug_assertions)]
    pypilot_controller_board::delay_ms(10);

    // disable modules
    // release hal adc which turns it off
    let adcp = hdwr.adc.into_inner().release();
    // release hal serial which turns it off
    #[cfg(debug_assertions)]
    let (usart, rxpin, txpin) = hdwr.serial.into_inner().release();

    // turn off clocks
    hdwr.cpu.prr.modify(|_, w| {
        #[cfg(debug_assertions)]
        w.prusart0().set_bit();
        w.prtim0().set_bit();
        w.pradc().set_bit()
    });

    // set INTO interrupt
    hdwr.exint.eimsk.modify(|_, w| w.int0().set_bit());

    // do the power down, if INT0 interrupt hasn't happened
    // no other interrupts are important, as the Pi will
    // be powered off in this state, so only need
    // to detect button pushes
    hdwr.cpu.smcr.write(|w| w.sm().pdown());
    interrupt::disable();
    unsafe {
        if !ptr::read_volatile(&PENDINGINT0) {
            // sleep enable
            hdwr.cpu.smcr.modify(|_, w| w.se().set_bit());
            interrupt::enable();
            // sleep cpu
            avr_device::asm::sleep();
            // sleep disable
            hdwr.cpu.smcr.modify(|_, w| w.se().clear_bit());
        } else {
            // INT0 pending
            ptr::write_volatile(&mut PENDINGINT0, false);
        }
        interrupt::enable();
    }

    // stop INTO interrupt
    hdwr.exint.eimsk.modify(|_, w| w.int0().clear_bit());

    // turn on clocks
    hdwr.cpu.prr.modify(|_, w| {
        #[cfg(debug_assertions)]
        w.prusart0().clear_bit();
        w.prtim0().clear_bit();
        w.pradc().clear_bit()
    });

    // enable modules
    let newadc = pypilot_controller_board::adc::Adc::new(adcp, Default::default());
    let newserial = pypilot_controller_board::Serial::<Floating>::new(
        usart,
        rxpin,
        txpin,
        57600.into_baudrate(),
    );

    // start adc and serial again
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
        adc: RefCell::new(newadc),
        a1: hdwr.a1,
        #[cfg(debug_assertions)]
        serial: RefCell::new(newserial),
        machine_state: hdwr.machine_state,
        prev_state: hdwr.prev_state,
        timeout: 0,
        serial_data_timeout: hdwr.serial_data_timeout - 250,
        command_value: 1000,
        lastpos: 1000,
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
        pending_cmd: CommandToExecute::None,
    }
}

//==========================================================

/// stop the motor
fn stop(mut hdwr: Hardware) -> Hardware {
    hdwr.command_value = 1000;
    position(hdwr, 1000)
}

//==========================================================

/// update the pwm based on motor command
fn position(mut hdwr: Hardware, value: u16) -> Hardware {
    // foPWM = fclk/(prescale*256)
    // foPWM = 3.906kHz
    let newpos = if value >= 1000 {
        (value - 1000) / 4
    } else {
        (1000 - value) / 4
    };
    hdwr.pwm_pin.set_duty(newpos.try_into().unwrap());
    if value > 1040 {
        hdwr.ina_pin.set_low().unwrap();
        hdwr.inb_pin.set_high().unwrap();
    } else if value < 960 {
        hdwr.ina_pin.set_high().unwrap();
        hdwr.inb_pin.set_low().unwrap();
    } else {
        // set brake
        hdwr.ina_pin.set_low().unwrap();
        hdwr.inb_pin.set_low().unwrap();
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
        lastpos: value,
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

/// the main state machine processer
fn process_fsm(mut hdwr: Hardware) -> Hardware {
    // first check the times
    let do_update = interrupt::free(|_cs| {
        let ticks = hdwr.timer2.tcnt2.read().bits();
        if ticks > 78 {
            hdwr.timeout += 1;
            hdwr.serial_data_timeout += 1;
            unsafe {
                hdwr.timer2.tcnt2.write(|w| w.bits(ticks - 78));
            }
            true
        } else {
            false
        }
    });

    // FSM
    let newstate = match hdwr.machine_state.0 {
        PyPilotStateMachine::Start => {
            change_state(PyPilotStateMachine::WaitEntry, hdwr.machine_state)
        }
        PyPilotStateMachine::WaitEntry => {
            hdwr.timeout = 0;
            hdwr.ina_pin.set_low().void_unwrap();
            hdwr.inb_pin.set_low().void_unwrap();
            change_state(PyPilotStateMachine::Wait, hdwr.machine_state)
        }
        PyPilotStateMachine::Wait => {
            if hdwr.timeout > 30 || hdwr.pending_cmd == CommandToExecute::Disengage {
                change_state(PyPilotStateMachine::DisengageEntry, hdwr.machine_state)
            } else if hdwr.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, hdwr.machine_state)
            } else {
                hdwr.machine_state
            }
        }
        PyPilotStateMachine::Engage => {
            hdwr.timeout = 0;
            hdwr.flags |= ENGAGED;
            // start PWM
            hdwr.ina_pin.set_low().void_unwrap();
            hdwr.ina_pin.set_low().void_unwrap();
            hdwr.pwm_pin.set_duty(0);
            hdwr.pwm_pin.enable();
            hdwr = position(hdwr, 1000);
            change_state(PyPilotStateMachine::Operational, hdwr.machine_state)
        }
        PyPilotStateMachine::Operational => {
            if do_update {
                let speed_rate: i16 = hdwr.max_slew_speed.into();
                // value of 20 is 1 second full range at 50hz
                let slow_rate: i16 = hdwr.max_slew_slow.into();
                let cur_value: u16 = hdwr.lastpos;
                // this should always be in range
                let mut diff: i16 = (hdwr.command_value - cur_value).try_into().unwrap();

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
                hdwr = position(hdwr, new_pos);
            }

            if hdwr.timeout > 30 || hdwr.pending_cmd == CommandToExecute::Disengage {
                change_state(PyPilotStateMachine::DisengageEntry, hdwr.machine_state)
            } else {
                hdwr.machine_state
            }
        }
        PyPilotStateMachine::DisengageEntry => {
            hdwr = stop(hdwr);
            hdwr.flags &= !ENGAGED;
            if hdwr.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, hdwr.machine_state)
            } else {
                change_state(PyPilotStateMachine::Disengage, hdwr.machine_state)
            }
        }
        PyPilotStateMachine::Disengage => {
            if hdwr.timeout > 62 {
                change_state(PyPilotStateMachine::DetachEntry, hdwr.machine_state)
            } else if hdwr.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, hdwr.machine_state)
            } else {
                hdwr.machine_state
            }
        }
        PyPilotStateMachine::DetachEntry => {
            hdwr.ina_pin.set_low().void_unwrap();
            hdwr.inb_pin.set_low().void_unwrap();
            hdwr.pwm_pin.set_duty(0);
            hdwr.pwm_pin.disable();
            change_state(PyPilotStateMachine::Detach, hdwr.machine_state)
        }
        PyPilotStateMachine::Detach => {
            if hdwr.timeout > 62 && hdwr.serial_data_timeout > 250 {
                change_state(PyPilotStateMachine::PowerDown, hdwr.machine_state)
            } else if hdwr.pending_cmd == CommandToExecute::Engage {
                change_state(PyPilotStateMachine::Engage, hdwr.machine_state)
            } else {
                hdwr.machine_state
            }
        }
        PyPilotStateMachine::PowerDown => {
            #[cfg(debug_assertions)]
            hdwr.serial.borrow_mut().flush();
            // cpu goes to sleep, then woken up
            hdwr = power_down_mode(hdwr);
            change_state(PyPilotStateMachine::WaitEntry, hdwr.machine_state)
        }
    };
    // check and see if machine state has changed, if so, send it via serial port
    if hdwr.prev_state.0 != hdwr.machine_state.0 || hdwr.prev_state.1 != hdwr.machine_state.1 {
        #[cfg(debug_assertions)]
        send_tuple(hdwr.machine_state, &mut hdwr.serial.borrow_mut());
        hdwr.prev_state = hdwr.machine_state;
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
        machine_state: newstate,
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
        pending_cmd: CommandToExecute::None,
    }
}

//==========================================================

// interrupt handler for ADC complete
#[interrupt(atmega328p)]
fn ADC() {
    static mut SAMPLE_CNT: u8 = 0;
    // unsafe is ok here, because we are in interrupt context
    // avr interrupts aren't nesting
    // we are modifying the ADC_RESULTS static structure
    // so can't reenable interrupts till this isr is finished
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

// interrupt handler for INT0
#[interrupt(atmega328p)]
fn INT0() {
    unsafe {
        core::ptr::write_volatile(&mut PENDINGINT0, true);
    }
}

//==========================================================
