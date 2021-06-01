use crate::hal::port::PortExt;

avr_hal_generic::impl_board_pins! {
    #[port_defs]
    use crate::hal::port;

    /// Generic DDR that works for all ports
    pub struct DDR {
        portb: crate::pac::PORTB,
        portc: crate::pac::PORTC,
        portd: crate::pac::PORTD,
    }

    /// Reexport of the Chart Plotter Hat's pins, with the names they have on the board
    pub struct Pins {
        /// `A0`
        ///
        /// * ADC0 (ADC input channel 0)
        /// * PCINT8 (pin change interrupt 8)
        pub a0: portc::pc0::PC0,
        /// `A1`
        ///
        /// * ADC1 (ADC input channel 1)
        /// * PCINT9 (pin change interrupt 9)
        pub a1: portc::pc1::PC1,
        /// `A2`
        ///
        /// * ADC2 (ADC input channel 2)
        /// * PCINT10 (pin change interrupt 10)
        pub a2: portc::pc2::PC2,
        /// `A3`
        ///
        /// * ADC3 (ADC input channel 3)
        /// * PCINT11 (pin change interrupt 11)
        pub a3: portc::pc3::PC3,
        /// `A4`
        ///
        /// * ADC4 (ADC input channel 4)
        /// * SDA (2-wire serial bus data input/output line)
        /// * PCINT12 (pin change interrupt 12)
        pub a4: portc::pc4::PC4,
        /// `A5`
        ///
        /// ADC5 (ADC input channel 5)
        /// SCL (2-wire serial bus clock line)
        /// PCINT13 (pin change interrupt 13)
        pub a5: portc::pc5::PC5,

        /// `RX/D0`
        ///
        pub rx: portd::pd0::PD0,
        /// `TX/D1`
        ///
        pub tx: portd::pd1::PD1,
        /// `D2`
        ///
        pub d2: portd::pd2::PD2,
        /// `D3`
        ///
        pub d3: portd::pd3::PD3,
        /// `D4`
        ///
        pub d4: portd::pd4::PD4,
        /// `D5`
        ///
        pub d5: portd::pd5::PD5,
        /// `D6`
        ///
        pub d6: portd::pd6::PD6,
        /// `D7`
        ///
        pub d7: portd::pd7::PD7,
        /// `D8`
        ///
        pub d8: portb::pb0::PB0,
        /// `D9`
        ///
        pub d9: portb::pb1::PB1,
        /// `D10`
        ///
        pub d10: portb::pb2::PB2,
        /// `D11/MOSI`
        ///
        pub d11: portb::pb3::PB3,
        /// `D12/MISO`
        ///
        pub d12: portb::pb4::PB4,
        /// `D13/SCK/LED`
        ///
        pub d13: portb::pb5::PB5,
    }
}
