# AvrMidi2CV

Simple MIDI to Control Voltage (CV) converter.
A work-in-progress at the moment,
with a basic MIDI state machine and CV, Velocity, and Pitch Bend
outputs via AD5676 DAC.
This DAC is an 8-channel 16-bit SPI type,
and should be accurate enough for 1V-per-octave note control voltages.
It's fine for Pitch Bend signals, too, which are 14-bit.
But it's not cheap and only available in a 20-pin TSSOP package
with 0.65mm pitch leads.
Of the eight analog output channels, we're only using three so far.
When we need lots more channels for MIDI messages such as Control Change
we'll add a 12-channel 8-bit DAC, the AD8804 (also SPI).

The microcontroller has plenty of spare GPIO pins which we can use
for on/off controls,
and to trigger analog rhythm sound generators.
This latter facility is not yet implemented,
but will be added later.

Runs on ATmega4809. Code in C, compiled with GCC.

## Connections

| Signal    | MCU Port | ATmega4809 DIP-40 pin | AD5676 TSSOP-20 pin |
|:----------|:---------|:----------------------|:--------------------|
| UPDI      | UPDI     | 30                    |                     |
| MIDI_OUT  | TxD1     | 1 (PC0)               |                     |
| MIDI_IN   | RxD1     | 2 (PC1)               |                     |
| SQWAVE    | PC2      | 3                     |                     |
| LED       | PC3      | 4                     |                     |
| GATE      | PC4      | 7                     |                     |
| TRIGGER   | PC5      | 8                     |                     |
| SUSTAIN   | PD7      | 16                    |                     |
| MOSI      | PA4/MOSI | 37                    | 7 (SDI)             |
| MISO      | PA5/MISO | 38                    | n/c                 |
| SCK       | PA6/SCK  | 39                    | 6 (SCLK)            |
| SS        | PA7/SS   | 40                    | 5 (/SYNC)           |
| RxD       | RxD0     | 34 (PA1)              |                     |
| TxD       | TxD0     | 33 (PA0)              |                     |
| CV        |          |                       | 2 (Vout0)           |
| VELOCITY  |          |                       | 1 (Vout1)           |
| PITCHBEND |          |                       | 20 (Vout2)          |

Power and ground pins not shown.

CV must be amplified by a non-inverting op-amp circuit to achieve
1V per octave over a useful range.
I used an LM324 which gives the capacity for four CV signals.

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

