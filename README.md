# AvrMidi2CV

Simple MIDI to Control Voltage (CV) converter.
Very much a work-in-progress at the moment,
with no MIDI state machine and just a trace print of MIDI bytes.
SPI interface to MCP4822 DAC is not implemented yet.

Runs on ATmega4809. Code in C, compiled with GCC.

## Connections

| Signal   | MCU Port | ATmega4809 DIP-40 pin | MCP4822 DIP-8 pin |
|----------|----------|-----------------------|-------------------|
| UPDI     | UPDI     | 30                    |                   |
| MIDI_OUT | TxD1     | 1 (PC0)               |                   |
| MIDI_IN  | RxD1     | 2 (PC1)               |                   |
| SQWAVE   | PC2      | 3                     |                   |
| LED      | PC3      | 4                     |                   |
| MOSI     | PA4/MOSI | 37                    | 4 (SDI)           |
| MISO     | PA5/MISO | 38                    | n/c               |
| SCK      | PA6/SCK  | 39                    | 3 (SCK)           |
| SS       | PA7/SS   | 40                    | 2 (/CS)           |
| RxD      | RxD0     | 34 (PA1)              |                   |
| TxD      | TxD0     | 33 (PA0)              |                   |
| TRIGGER  | ?        | ?                     |                   |
| GATE     | ?        | ?                     |                   |

TRIGGER and GATE outputs TBD.

Power and ground pins not shown.

## AVR Toolchain

The program has been compiled, linked and tested using a Linux version
of the 'avr-gcc' toolchain.
This can be installed directly or as part of the Arduino IDE.

The compiler, linker and programmers are invoked from the Makefile in
the usual way.
Various parameters in the Makefile may be altered to suit the development
setup, e.g. the type of programmers used and the ports that they connect to.
The full pathname to the toolchain is also configured in the Makefile.

A special target in the Makefile is provided to invoke the programming
device(s) and write the ELF files into the Flash memory in the chips.
This target is called 'prog4809'.
The 'prog4809' target invokes the UPDI programmer.

There's a Makefile target called 'clean' that deletes the object code files
and the ELF binary files.
It leaves the source code files untouched, of course.

## AVR Programmers

I have tested the code with a 'jtag2updi' implemented on an ATmega328P.

