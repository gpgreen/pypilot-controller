# PyPilot Motor Controller firmware for PyPilot Controller board

Firmware for ATmega328P that controls an autopilot motor. The
controller is attached to a NMEA2000 network where it receives motor
commands. The ATmega328P is on a Arduino Nano board, the motor
controller is a VNH5019.

Intended for use with Raspberry PI boat computer project. The
corresponding hardware design is at: ![PyPilot Controller](https://github.com/gpgreen/pypilot_controller)

## Code

![state machine for pypilot motor control](StateMachine.png)

Diagram made using [gaphor](https://gaphor.readthedocs.io/en/latest/)

## Build instructions

Install Rust nightly.
```
rustup toolchain install nightly-2021-01-07
```

Then run:

```
cargo +nightly-2021-01-07 build --release
```

The final ELF executable file will then be available at `target/avr-atmega328p/release/pypilot-controller.elf`.

## ATMega328P fuse settings
The fuses are changed from the default and need to be updated
```
avrdude -p atmega328p -c <your programmer here> -U lfuse:w:0xc2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
```

## Firmware Load

The compiled firmware (pypilot-controller.hex) can be programmed into the hardware using
```
avrdude -p atmega328p -c <your programmer here> -U flash:w:pypilot-controller.hex
```

To create the hex file, use avr-objcopy
```
avr-objcopy -O ihex target/avr-atmega328p/release/pypilot-controller.elf pypilot-controller.hex
```

This can also be done with the makefile target 'program', or `make
program`

## PyPilot Controller Hardware Changes

- Rev B (First real hardware)

## PIN LAYOUT
```
            
                   ATmega328P-32A
            +--------------------------+        
            |                          |        
        3.3-|4  VCC              PB0 12|-       
        3.3-|18 AVCC             PB1 13|-INT    
   CAP->GND-|20 AREF             PB2 14|-CS
        GND-|3  GND              PB3 15|-MOSI   
            |                    PB4 16|-MISO   
           -|19 ADC6             PB5 17|-SCK    
           -|22 ADC7             PB6  7|-       
            |                    PB7  8|-       
           -|30 PD0                    |        
           -|31 PD1              PC0 23|-       
        INA-|32 PD2              PC1 24|-A1     
        INB-|1  PD3              PC2 25|-       
        ENA-|2  PD4              PC3 26|-       
        ENB-|9  PD5              PC4 27|-       
        PWM-|10 PD6              PC5 28|-       
           -|11 PD7              PC6 29|-RESET  
            |                          |        
            +--------------------------+        
```

## License
pypilot-controller is licensed under the `GNU General Public License v3.0 or later`. See [LICENSE](LICENSE) for more info.
