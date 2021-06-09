# PyPilot Motor Controller firmware for PyPilot Controller board

Firmware for ATmega328P that controls an autopilot motor. The
controller is attached to a NMEA2000 network where it receives motor
commands. The ATmega328P is on a Arduino Nano board, the motor
controller is a VNH5019.

In debug builds, the board will use the serial port to input/output
packets as in the original motor controller in the Arduino sketch.

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

To create the hex file, use the script 'mkhex.sh' in the
pypilot-controller directory:
```
cd <git working dir>/pypilot-controller
./mkhex.sh --debug pypilot-controller
```
The resulting hex file is in the target directory.

The compiled firmware (pypilot-controller.hex) can be programmed into the hardware using
```
avrdude -p atmega328p -c <your programmer here> -U flash:w:pypilot-controller.hex
```

Or, use the script 'program.sh' in the pypilot-controller directory:
```
./program.sh pypilot-controller
```

## PyPilot Controller Hardware Changes

- Rev B (First real hardware)

## PIN LAYOUT
```
            
                   ATmega328P-32A
            +--------------------------+        
            |                          |        
        3.3-|4  VCC          D8  PB0 12|-       
        3.3-|18 AVCC         D9  PB1 13|-INT    
   CAP->GND-|20 AREF         D10 PB2 14|-CS
        GND-|3  GND          D11 PB3 15|-MOSI   
            |                D12 PB4 16|-MISO   
           -|19 ADC6 A6      D13 PB5 17|-SCK    
           -|22 ADC7 A7          PB6  7|-XTAL1  
            |                    PB7  8|-XTAL2  
        RX0-|30 PD0 D0                 |        
        TX0-|31 PD1 D1       A0  PC0 23|-A0     
        INA-|32 PD2 D2       A1  PC1 24|-A1     
        INB-|1  PD3 D3       A2  PC2 25|-[CT]   
        ENA-|2  PD4 D4       A3  PC3 26|-[MT]   
        ENB-|9  PD5 D5       A4  PC4 27|-[RA]   
        PWM-|10 PD6 D6       A5  PC5 28|-       
           -|11 PD7 D7           PC6 29|-RESET  
            |                          |        
            +--------------------------+        
```

## License
pypilot-controller is licensed under the `GNU General Public License v3.0 or later`. See [LICENSE](LICENSE) for more info.
