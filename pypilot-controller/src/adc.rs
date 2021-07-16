use arduino_hal::hal::{
    port,
    port::{mode, Pin},
};
use avr_device::{
    atmega328p, interrupt,
    interrupt::{CriticalSection, Mutex},
};
use core::{
    cell::Cell,
    convert::TryFrom,
    sync::atomic::{AtomicBool, Ordering},
    u32,
};
use heapless::spsc::Queue;

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

impl From<AdcChannel> for u8 {
    fn from(original: AdcChannel) -> u8 {
        match original {
            AdcChannel::Current => 0,
            AdcChannel::Voltage => 1,
            AdcChannel::ControllerTemp => 2,
            AdcChannel::MotorTemp => 3,
            AdcChannel::Rudder => 4,
        }
    }
}

#[derive(Debug)]
pub struct AdcParseError {}

/// convert from u8
impl TryFrom<u8> for AdcChannel {
    type Error = AdcParseError;

    fn try_from(original: u8) -> Result<Self, Self::Error> {
        match original {
            0 => Ok(AdcChannel::Current),
            1 => Ok(AdcChannel::Voltage),
            2 => Ok(AdcChannel::ControllerTemp),
            3 => Ok(AdcChannel::MotorTemp),
            4 => Ok(AdcChannel::Rudder),
            _ => Err(AdcParseError {}),
        }
    }
}

//==========================================================

/// which moving average index we want to use
#[derive(Debug, Copy, Clone)]
pub enum AdcMvgAvgIndex {
    First,
    Second,
}

impl From<AdcMvgAvgIndex> for usize {
    fn from(original: AdcMvgAvgIndex) -> usize {
        match original {
            AdcMvgAvgIndex::First => 0,
            AdcMvgAvgIndex::Second => 1,
        }
    }
}

//==========================================================

/// adc sample moving averages
struct AdcMvgAvg {
    total: [u32; 2],
    count: [u32; 2],
}

impl AdcMvgAvg {
    fn calc(&self, n: AdcMvgAvgIndex) -> Option<u16> {
        let i: usize = n.into();
        if self.count[i] == 0 {
            None
        } else {
            let mut quotient = 0;
            let mut rem = self.total[i];
            while rem > self.count[i] {
                rem -= self.count[i];
                quotient += 1;
            }
            if quotient > u16::MAX {
                None
            } else {
                Some(quotient as u16)
            }
        }
        //        self.total.checked_div(self.count)
    }
    fn reset(&mut self, n: AdcMvgAvgIndex) {
        let i: usize = n.into();
        self.count[i] = 0;
        self.total[i] = 0;
    }
}

//==========================================================

/// the struct passed from isr to main line, containing current adc readings for a channel
#[derive(Copy, Clone, Debug)]
struct Sample {
    channel: AdcChannel,
    total: u32,
    count: u32,
}

impl Sample {
    fn reset(&mut self, ch: AdcChannel) {
        self.channel = ch;
        self.total = 0;
        self.count = 0;
    }
}

//==========================================================

bitflags! {
    /// ADC Channel enabled bitflags
    pub struct AdcChannelsEnabled: u8 {
    const CURRENT = 0b0000_0001;
    const VOLTAGE = 0b0000_0010;
    const CONTROLLERTEMP = 0b0000_0100;
    const MOTORTEMP = 0b0000_1000;
    const RUDDERANGLE = 0b0001_0000;
    }
}

//==========================================================

/// Queue for samples from ADC
static mut SAMPLE_QUEUE: Queue<Sample, 64> = Queue::new();

/// flag for SAMPLE_QUEUE overflow, true if enqueue failed
pub static mut SAMPLE_OVF: AtomicBool = AtomicBool::new(false);

/// the hardware for the ADC
static mut ADC_HARDWARE: Mutex<Cell<Option<atmega328p::ADC>>> = Mutex::new(Cell::new(None));

/// the enabled flags for ADC channels
static mut CHANNELS_ENABLED: AdcChannelsEnabled = AdcChannelsEnabled::empty();

pub static mut LED: Option<Pin<mode::Output, port::PD3>> = None;

/// has the sample queue overflowed
pub fn sample_overflow() -> bool {
    // SAFETY: read of u8 should be safe
    unsafe { SAMPLE_OVF.load(Ordering::Relaxed) }
}

//==========================================================

/// adc sample data for each channel
pub struct AdcResults {
    data: [AdcMvgAvg; 5],
}

impl AdcResults {
    /// create an instance of AdcResults
    pub fn new() -> AdcResults {
        AdcResults {
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
        }
    }

    /// setup adc hardware, set enabled flags
    pub fn init(&mut self, cs: &CriticalSection, adc: atmega328p::ADC) {
        adc.adcsra
            .write(|w| w.aden().set_bit().adie().set_bit().adps().prescaler_128());
        adc.admux.write(|w| w.refs().avcc().mux().adc1());
        // disable digital inputs
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
        // start conversion
        adc.adcsra.modify(|_, w| w.adsc().set_bit());
        // move the adc to the static struct
        // SAFETY: inside of critical section
        unsafe {
            CHANNELS_ENABLED.insert(AdcChannelsEnabled::CURRENT | AdcChannelsEnabled::VOLTAGE);
            #[cfg(feature = "controller_temp")]
            {
                CHANNELS_ENABLED.insert(AdcChannelsEnabled::CONTROLLERTEMP);
            }
            #[cfg(feature = "motor_temp")]
            {
                CHANNELS_ENABLED.insert(AdcChannelsEnabled::MOTORTEMP);
            }
            #[cfg(feature = "rudder_angle")]
            {
                CHANNELS_ENABLED.insert(AdcChannelsEnabled::RUDDERANGLE);
            }
            ADC_HARDWARE.borrow(cs).replace(Some(adc));
        };
    }
    /// process adc samples
    pub fn process(&mut self) -> bool {
        // SAFETY: read of u8 should be safe
        if unsafe { SAMPLE_OVF.load(Ordering::Relaxed) } {
            return true;
        }
        // check for new sample
        while let Some(sample) = unsafe { SAMPLE_QUEUE.split().1.dequeue() } {
            let add_sample = match sample.channel {
                AdcChannel::Current => true,
                AdcChannel::Voltage => true,
                AdcChannel::ControllerTemp => false,
                AdcChannel::MotorTemp => false,
                AdcChannel::Rudder => false,
            };
            if add_sample {
                self.add_sample(sample);
            }
        }
        false
    }

    /// add the sample to a channel array
    #[inline]
    fn add_sample(&mut self, sample: Sample) {
        //        static mut CNT: u16 = 0;
        let channel: usize = sample.channel.into();
        // make sure we don't overflow
        for i in 0..2 {
            let allowed_total = u32::max_value() - self.data[channel].total[i];
            let allowed_count = u32::max_value() - self.data[channel].count[i];
            if allowed_total > sample.total && allowed_count > sample.count {
                self.data[channel].count[i] += sample.count;
                self.data[channel].total[i] += sample.total;
            }
        }
    }

    pub fn get_current(&mut self, count: u32, n: AdcMvgAvgIndex) -> Option<u16> {
        let idx: usize = AdcChannel::Current.into();
        let nidx: usize = n.into();
        if self.data[idx].count[nidx] >= count {
            let avg = self.data[idx].calc(n);
            self.data[idx].reset(n);
            avg
        } else {
            None
        }
    }

    pub fn get_voltage(&mut self, count: u32, n: AdcMvgAvgIndex) -> Option<u16> {
        let idx: usize = AdcChannel::Voltage.into();
        let nidx: usize = n.into();
        if self.data[idx].count[nidx] >= count {
            let avg = self.data[idx].calc(n);
            self.data[idx].reset(n);
            avg
        } else {
            None
        }
    }

    /// convert a raw mvg avg value to thermistor temperature
    #[cfg(any(feature = "controller_temp", feature = "motor_temp"))]
    fn thermistor_conversion(&self, raw: Option<u16>) -> Option<u16> {
        if let Some(v) = raw {
            let r = 100061 * v / (74464 - v);
            Some((30000000 / (r + 2600) + 200) as u16)
        } else {
            None
        }
    }

    /// get controller temperature from sample data
    #[cfg(feature = "controller_temp")]
    pub fn get_controller_temp(&mut self, count: u32, n: AdcMvgAvgIndex) -> Option<u16> {
        let idx: usize = AdcChannel::ControllerTemp.into();
        let nidx: usize = n.into();
        if self.data[idx].count[nidx] >= count {
            let avg = self.data[idx].calc(n);
            self.data[idx].reset(n);
            self.thermistor_conversion(avg)
        } else {
            None
        }
    }

    /// get motor temperature from sample data
    #[cfg(feature = "motor_temp")]
    pub fn get_motor_temp(&mut self, count: u32, n: AdcMvgAvgIndex) -> Option<u16> {
        let idx: usize = AdcChannel::MotorTemp.into();
        let nidx: usize = n.into();
        if self.data[idx].count[nidx] >= count {
            let avg = self.data[idx].calc(n);
            self.data[idx].reset(n);
            self.thermistor_conversion(avg)
        } else {
            None
        }
    }

    /// get rudder angle from sample data
    #[cfg(feature = "rudder_angle")]
    pub fn get_rudder(&mut self, count: u32, n: AdcMvgAvgIndex) -> Option<u16> {
        let idx: usize = AdcChannel::Rudder.into();
        let nidx: usize = n.into();
        if self.data[idx].count[nidx] >= count {
            let avg = self.data[idx].calc(n);
            self.data[idx].reset(n);
            avg * 4
        } else {
            None
        }
    }
}

//==========================================================

#[interrupt(atmega328p)]
fn ADC() {
    static mut ADC: Option<atmega328p::ADC> = None;
    static mut SAMPLE: Sample = Sample {
        channel: AdcChannel::Current,
        total: 0,
        count: 0,
    };
    static mut CNT: u8 = 0;
    static mut LEDCNT: u16 = 0;

    interrupt::free(|cs| {
        unsafe {
            if let Some(adc) = &mut ADC {
                // throw away first 3 samples
                if CNT == 3 {
                    SAMPLE.total += adc.adc.read().bits() as u32;
                    SAMPLE.count += 1;
                    let switch_channel = match SAMPLE.channel {
                        AdcChannel::Current => SAMPLE.count >= 50,
                        AdcChannel::Voltage => SAMPLE.count >= 8,
                        AdcChannel::ControllerTemp | AdcChannel::MotorTemp | AdcChannel::Rudder => {
                            SAMPLE.count >= 1
                        }
                    };
                    if switch_channel {
                        // enqueue current sample, if enabled
                        let enabled = match SAMPLE.channel {
                            AdcChannel::Current => {
                                CHANNELS_ENABLED.contains(AdcChannelsEnabled::CURRENT)
                            }
                            AdcChannel::Voltage => {
                                CHANNELS_ENABLED.contains(AdcChannelsEnabled::VOLTAGE)
                            }
                            AdcChannel::ControllerTemp => {
                                CHANNELS_ENABLED.contains(AdcChannelsEnabled::CONTROLLERTEMP)
                            }
                            AdcChannel::MotorTemp => {
                                CHANNELS_ENABLED.contains(AdcChannelsEnabled::MOTORTEMP)
                            }
                            AdcChannel::Rudder => {
                                CHANNELS_ENABLED.contains(AdcChannelsEnabled::RUDDERANGLE)
                            }
                        };
                        if enabled {
                            match SAMPLE_QUEUE.split().0.enqueue(SAMPLE) {
                                Ok(()) => {}
                                Err(_) => SAMPLE_OVF.store(true, Ordering::Relaxed),
                            }
                        }
                        // get next channel
                        let mut ch = SAMPLE.channel;
                        loop {
                            // loop until next enabled channel
                            ch = match ch {
                                AdcChannel::Current => AdcChannel::Voltage,
                                AdcChannel::Voltage => AdcChannel::ControllerTemp,
                                AdcChannel::ControllerTemp => AdcChannel::MotorTemp,
                                AdcChannel::MotorTemp => AdcChannel::Rudder,
                                AdcChannel::Rudder => AdcChannel::Current,
                            };
                            let enabled = match ch {
                                AdcChannel::Current => {
                                    CHANNELS_ENABLED.contains(AdcChannelsEnabled::CURRENT)
                                }
                                AdcChannel::Voltage => {
                                    CHANNELS_ENABLED.contains(AdcChannelsEnabled::VOLTAGE)
                                }
                                AdcChannel::ControllerTemp => {
                                    CHANNELS_ENABLED.contains(AdcChannelsEnabled::CONTROLLERTEMP)
                                }
                                AdcChannel::MotorTemp => {
                                    CHANNELS_ENABLED.contains(AdcChannelsEnabled::MOTORTEMP)
                                }
                                AdcChannel::Rudder => {
                                    CHANNELS_ENABLED.contains(AdcChannelsEnabled::RUDDERANGLE)
                                }
                            };
                            if enabled {
                                break;
                            }
                        }
                        // change the mux
                        match ch {
                            // Current is on ADC1
                            AdcChannel::Current => {
                                adc.admux.write(|w| w.refs().avcc().mux().adc1())
                            }
                            // Voltage on pin ADC0
                            AdcChannel::Voltage => {
                                adc.admux.write(|w| w.refs().avcc().mux().adc0())
                            }
                            // Controller Temp on pin ADC2
                            AdcChannel::ControllerTemp => {
                                adc.admux.write(|w| w.refs().avcc().mux().adc2())
                            }
                            // Motor Temp on pin ADC3
                            AdcChannel::MotorTemp => {
                                adc.admux.write(|w| w.refs().avcc().mux().adc3())
                            }
                            // Rudder angle on pin ADC4
                            AdcChannel::Rudder => adc.admux.write(|w| w.refs().avcc().mux().adc4()),
                        }
                        if LEDCNT == 100 {
                            if let Some(led) = &mut LED {
                                led.toggle();
                            }
                            LEDCNT = 0;
                        } else {
                            LEDCNT += 1;
                        }
                        // reset sample
                        SAMPLE.reset(ch);
                        CNT = 0;
                    }
                } else {
                    CNT += 1;
                }
                // start a new sample
                adc.adcsra.modify(|_, w| w.adsc().set_bit());
            } else {
                // move hardware into this isr
                ADC.replace(ADC_HARDWARE.borrow(cs).replace(None).unwrap());
                // start a new sample
                if let Some(adc) = &mut ADC {
                    adc.adcsra.modify(|_, w| w.adsc().set_bit());
                }
            }
        }
    })
}

//==========================================================
