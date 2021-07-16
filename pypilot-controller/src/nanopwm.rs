pub use arduino_hal::{
    hal::{
        port,
        port::{mode, Pin},
    },
    pac,
};
pub use avr_hal_generic::pwm::*;

pub struct Timer0Pwm {
    timer: pac::TC0,
}

impl Timer0Pwm {
    pub fn new(timer: pac::TC0, prescaler: ::avr_hal_generic::pwm::Prescaler) -> Timer0Pwm {
        let mut t = Timer0Pwm { timer };
        {
            let tim = &mut t.timer;
            let prescaler = prescaler;
            {
                tim.tccr0a.modify(|_, w| w.wgm0().pwm_fast());
                tim.tccr0b.modify(|_, w| match prescaler {
                    Prescaler::Direct => w.cs0().direct(),
                    Prescaler::Prescale8 => w.cs0().prescale_8(),
                    Prescaler::Prescale64 => w.cs0().prescale_64(),
                    Prescaler::Prescale256 => w.cs0().prescale_256(),
                    Prescaler::Prescale1024 => w.cs0().prescale_1024(),
                });
            }
        }
        t
    }
}
pub struct PwmPin<PIN> {
    _pin: PIN,
}
type PwmDuty = u8;

impl PwmPin<Pin<mode::Output, port::PD6>> {
    pub fn into_pwm(
        pin: Pin<mode::Output, port::PD6>,
        _pwm_timer: &mut Timer0Pwm,
    ) -> PwmPin<Pin<mode::Output, port::PD6>> {
        PwmPin::<Pin<mode::Output, port::PD6>> { _pin: pin }
    }
}

impl PwmPin<Pin<mode::Output, port::PD6>> {
    pub fn enable(&mut self) {
        ::avr_hal_generic::avr_device::interrupt::free(|_| {
            let tim = unsafe { &*<pac::TC0>::ptr() };
            {
                tim.tccr0a.modify(|_, w| w.com0a().match_clear());
            }
        })
    }
    pub fn disable(&mut self) {
        ::avr_hal_generic::avr_device::interrupt::free(|_| {
            let tim = unsafe { &*<pac::TC0>::ptr() };
            {
                tim.tccr0a.modify(|_, w| w.com0a().disconnected());
            }
        })
    }
    pub fn get_duty(&self) -> PwmDuty {
        unsafe { (&*<pac::TC0>::ptr()) }.ocr0a.read().bits() as PwmDuty
    }
    pub fn get_max_duty(&self) -> PwmDuty {
        u8::MAX
    }
    pub fn set_duty(&mut self, duty: PwmDuty) {
        unsafe { (&*<pac::TC0>::ptr()).ocr0a.write(|w| w.bits(duty.into())) };
    }
}
/*
impl PwmPin<Pin<mode::Output, port::PD5>> {
    pub fn into_pwm(
        pin: Pin<mode::Output, port::PD5>,
        pwm_timer: &mut Timer0Pwm,
    ) -> PwmPin<Pin<mode::Output, port::PD5>> {
        PwmPin::<Pin<mode::Output, port::PD5>> { pin: pin }
    }
}
impl PwmPin<Pin<mode::Output, port::PD5>> {
    fn enable(&mut self) {
        ::avr_hal_generic::avr_device::interrupt::free(|_| {
            let tim = unsafe { &*<pac::TC0>::ptr() };
            {
                tim.tccr0a.modify(|_, w| w.com0b().match_clear());
            }
        })
    }
    fn disable(&mut self) {
        ::avr_hal_generic::avr_device::interrupt::free(|_| {
            let tim = unsafe { &*<pac::TC0>::ptr() };
            {
                tim.tccr0a.modify(|_, w| w.com0b().disconnected());
            }
        })
    }
    fn get_duty(&self) -> PwmDuty {
        unsafe { (&*<pac::TC0>::ptr()) }.ocr0b.read().bits() as PwmDuty
    }
    fn get_max_duty(&self) -> PwmDuty {
        u8::MAX
    }
    fn set_duty(&mut self, duty: PwmDuty) {
        unsafe { (&*<pac::TC0>::ptr()).ocr0b.write(|w| w.bits(duty.into())) };
    }
}
*/
