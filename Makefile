# Makefile for the ATmega4809 MIDI to CV interface

# We'll pick up the GCC toolchain from the Arduino installation
ARDUINO=/home/john/Arduino/arduino-1.8.10

MCU4809=atmega4809

CC=$(ARDUINO)/hardware/tools/avr/bin/avr-gcc
LD=$(ARDUINO)/hardware/tools/avr/bin/avr-gcc
OC=$(ARDUINO)/hardware/tools/avr/bin/avr-objcopy
SZ=$(ARDUINO)/hardware/tools/avr/bin/avr-size
DUDE=$(ARDUINO)/hardware/tools/avr/bin/avrdude

CFLAGS=-c -o $@ -O3
LDFLAGS=-o $@
OCFLAGS=-j .text -j .data -O ihex
SZFLAGS=-B -d
DUDEFLAGS=-C $(ARDUINO)/hardware/tools/avr/etc/avrdude.conf

# Programming port and programming device. Can be overridden from the
# command line, e.g. make UPDIPORT=/dev/ttyUSB1 prog4809
UPDIPORT=/dev/ttyUSB1
UPDIDEV=jtag2updi

OBJS=midi2cv.o
ELFS=$(OBJS:.o=.elf)

# Default target will compile and link all C sources, but not program anything
all: $(ELFS)
.PHONY: all

midi2cv.elf: midi2cv.o
	$(LD) -mmcu=$(MCU4809) $(LDFLAGS) midi2cv.o
	$(SZ) --mcu=$(MCU4809) $(SZFLAGS) midi2cv.elf

midi2cv.o: midi2cv.c
	$(CC) -mmcu=$(MCU4809) $(CFLAGS) midi2cv.c

# Targets to invoke the programmer and program the flash memory of the MCU
prog4809: midi2cv.elf
	$(DUDE) $(DUDEFLAGS) -c $(UPDIDEV) -P $(UPDIPORT) -p $(MCU4809) -e -U flash:w:midi2cv.elf:e

.PHONY: prog4809

# Target 'testupdi' will connect to the
# programmer and read the device ID, but not program it
testupdi:
	$(DUDE) $(DUDEFLAGS) -c $(UPDIDEV) -P $(UPDIPORT) -p $(MCU1616)

.PHONY: testupdi

# Target 'clean' will delete all object files and ELF files
clean:
	-rm -f $(OBJS) $(ELFS)

.PHONY: clean

# USBasp upgrade using real Atmel AVRISP on /dev/ttyS4:
# /home/john/Arduino/arduino-1.8.10/hardware/tools/avr/bin/avrdude -C /home/john/Arduino/arduino-1.8.10/hardware/tools/avr/etc/avrdude.conf -c avrispv2 -P /dev/ttyS4 -p atmega8 -e -U flash:w:usbasp.atmega8.2011-05-28.hex:i
