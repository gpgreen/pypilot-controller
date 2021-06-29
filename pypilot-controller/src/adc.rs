use avr_device::interrupt;
use core::cell::UnsafeCell;
use core::ptr;

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
struct AdcResultsPriv {
    enabled: u8,
    selected: AdcChannel,
    data: [AdcMvgAvg; 5],
}

pub struct AdcResults(UnsafeCell<AdcResultsPriv>);

impl AdcResults {
    /// setup the array, used for setting the enable flags
    pub fn setup(&self, _cs: &interrupt::CriticalSection) {
        unsafe {
            // setup the enabled flags, if it hasn't yet been done
            if (*self.0.get()).enabled == 0 {
                // Current and Voltage are always on
                let mut enabled_flags = 0x3;
                #[cfg(feature = "controller_temp")]
                enable_controller(&mut enabled_flags);
                #[cfg(feature = "motor_temp")]
                enable_motor(&mut enabled_flags);
                #[cfg(feature = "rudder_angle")]
                enable_rudder_angle(&mut enabled_flags);
                (*self.0.get()).enabled = enabled_flags;
            }
        }
    }

    #[cfg(feature = "controller_temp")]
    fn enable_controller(enabled: &mut u8) {
        enabled |= 0x4;
    }

    #[cfg(feature = "motor_temp")]
    fn enable_motor(enabled: &mut u8) {
        enabled |= 0x8;
    }

    #[cfg(feature = "rudder_angle")]
    fn enable_rudder_angle(enabled: &mut u8) {
        enabled |= 0x10;
    }

    /// get selected channel
    #[inline]
    fn selected(&self, _cs: &interrupt::CriticalSection) -> AdcChannel {
        unsafe { (*self.0.get()).selected }
    }

    /*
    /// convert the selected channel to an index into data array
    #[inline]
    fn selected_channel(&self, _cs: &interrupt::CriticalSection) -> usize {
        // SAFETY: only used in isr
        unsafe { (*self.0.get()).selected.into() }
    }
     */

    /// is a channel enabled
    #[inline]
    fn is_channel_enabled(&self, _cs: &interrupt::CriticalSection, channel: AdcChannel) -> bool {
        // SAFETY: only used in isr
        unsafe {
            match channel {
                AdcChannel::Current => (*self.0.get()).enabled & 0x1 == 0x1,
                AdcChannel::Voltage => (*self.0.get()).enabled & 0x2 == 0x2,
                AdcChannel::ControllerTemp => (*self.0.get()).enabled & 0x4 == 0x4,
                AdcChannel::MotorTemp => (*self.0.get()).enabled & 0x8 == 0x8,
                AdcChannel::Rudder => (*self.0.get()).enabled & 0x10 == 0x10,
            }
        }
    }

    /// get the adc mux register value for selected channel
    #[inline]
    fn adc_mux(&self, _cs: &interrupt::CriticalSection) -> u8 {
        //let mux = _BV(REFS0) | _BV(REFS1);
        // SAFETY: only used in isr
        let mux = match unsafe { (*self.0.get()).selected } {
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
        mux & 0xC0
    }

    /// switch to the next adc channel
    #[inline]
    fn next_channel(&self, cs: &interrupt::CriticalSection) {
        // get the next 'enabled' channel
        // SAFETY: only used in isr
        let mut sel = unsafe { (*self.0.get()).selected };
        loop {
            let nxt = match sel {
                AdcChannel::Current => AdcChannel::Voltage,
                AdcChannel::Voltage => AdcChannel::ControllerTemp,
                AdcChannel::ControllerTemp => AdcChannel::MotorTemp,
                AdcChannel::MotorTemp => AdcChannel::Rudder,
                AdcChannel::Rudder => AdcChannel::Current,
            };
            // SAFETY: only used in isr
            unsafe {
                if self.is_channel_enabled(cs, nxt) {
                    (*self.0.get()).selected = nxt;
                    break;
                } else {
                    sel = nxt;
                }
            }
        }
    }

    /// count the results in a channel array
    pub fn count(
        &self,
        _cs: &interrupt::CriticalSection,
        channel: AdcChannel,
        narray: usize,
    ) -> u16 {
        let i: usize = channel.into();
        unsafe { (*self.0.get()).data[i].count[narray] }
    }

    /// count the results in a channel array
    pub fn add_sample(
        &self,
        _cs: &interrupt::CriticalSection,
        sample: u32,
        channel: AdcChannel,
        narray: usize,
    ) {
        let i: usize = channel.into();
        unsafe {
            (*self.0.get()).data[i].total[narray] += sample;
            (*self.0.get()).data[i].count[narray] += 1;
        }
    }

    /// get result in a channel array
    fn take(&self, _cs: &interrupt::CriticalSection, channel: AdcChannel, narray: usize) -> u16 {
        let i: usize = channel.into();
        unsafe {
            let cnt = (*self.0.get()).data[i].count[narray];
            let val = (*self.0.get()).data[i].total[narray];
            (*self.0.get()).data[i].total[narray] = 0;
            (*self.0.get()).data[i].count[narray] = 0;
            if cnt == 0 {
                0
            } else {
                let long_val: u32 = val.into();
                let long_cnt: u32 = cnt.into();
                let avg = 16 * long_val / long_cnt;
                avg as u16
            }
        }
    }

    /// get current from sample data
    pub fn take_amps(&self, cs: &interrupt::CriticalSection, narray: usize) -> u16 {
        let v = self.take(cs, AdcChannel::Current, narray);
        v * 9 / 34 / 16
    }

    /// get voltage from sample data
    pub fn take_volts(&self, cs: &interrupt::CriticalSection, narray: usize) -> u16 {
        // voltage in 10mV increments 1.1ref, 560 and 10k resistors
        let v = self.take(cs, AdcChannel::Voltage, narray);
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
    pub fn take_controller_temp(&self, narray: usize) -> u16 {
        let v: u32 = self.take(AdcChannel::ControllerTemp, narray).into();
        self.thermistor_conversion(v)
    }

    /// get motor temperature from sample data
    #[cfg(feature = "motor_temp")]
    pub fn take_motor_temp(&self, narray: usize) -> u16 {
        let v: u32 = self.take(AdcChannel::MotorTemp, narray).into();
        self.thermistor_conversion(v)
    }

    /// get rudder angle from sample data
    #[cfg(feature = "rudder_angle")]
    pub fn take_rudder(&self, narray: usize) -> u16 {
        self.take(AdcChannel::Rudder, narray) * 4
    }
}

/// static variable for AdcResults and initializer
pub static ADC_RESULTS: AdcResults = AdcResults(UnsafeCell::new(AdcResultsPriv {
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
}));

unsafe impl Sync for AdcResults {}

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
        interrupt::free(|cs| {
            // magic address
            const ADCW_REG: u16 = 0x78;
            // get the sample value
            let sample = ptr::read_volatile(ADCW_REG as *mut u16);
            // add the sample to the array
            let sel = ADC_RESULTS.selected(&cs);
            SAMPLE_CNT += 1;
            // throw out the first 3 samples
            if SAMPLE_CNT > 3 {
                for i in 0..2 {
                    if ADC_RESULTS.count(&cs, sel, i) < 4000 {
                        ADC_RESULTS.add_sample(&cs, sample as u32, sel, i);
                    }
                }
            }
            // check to see if we need more samples for this channel
            let do_more = match sel {
                AdcChannel::Current => SAMPLE_CNT < 50,
                AdcChannel::Voltage => SAMPLE_CNT < 8,
                AdcChannel::Rudder => SAMPLE_CNT < 16,
                _ => false,
            };
            if !do_more {
                SAMPLE_CNT = 0;
                ADC_RESULTS.next_channel(&cs);
                // magic address
                const ADCMUX_REG: u8 = 0x7C;
                // write the mux
                ptr::write_volatile(ADCMUX_REG as *mut u8, ADC_RESULTS.adc_mux(&cs));
            }
            // do the next conversion
            // magic address
            const ADCSRA_REG: u8 = 0x7A;
            ptr::write_volatile(ADCSRA_REG as *mut u8, 0xC8);
        });
    }
}

//==========================================================
