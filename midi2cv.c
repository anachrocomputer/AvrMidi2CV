/* midi2cv --- ATmega4809 MIDI-to-Control Voltage           2024-03-04 */

#define F_CPU (20000000)

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// UART0 TxD on PA0 (default) pin 33
// UART0 RxD on PA1 (default) pin 34
// UART1 TxD on PC0 (default) pin 1
// UART1 RxD on PC1 (default) pin 2
#define SQWAVE_PIN  PIN2_bm   // 500Hz square wave on PC2 (pin 3)
#define LED_PIN     PIN3_bm   // Blinking LED on PC3 (pin 4)
#define GATE_PIN    PIN4_bm   // GATE signal on PC4 (pin 7)
#define TRIGGER_PIN PIN5_bm   // TRIGGER signal on PC5 (pin 8)
#define DAC_CS_PIN  PIN7_bm   // MCP4822 /CS pin on PA7 (pin 40)


#define MIDI_NOTE_OFF           (0x80)
#define MIDI_NOTE_ON            (0x90)
#define MIDI_POLY_AFTERTOUCH    (0xA0)
#define MIDI_CONTROL_CHANGE     (0xB0)
#define MIDI_PROGRAM_CHANGE     (0xC0)
#define MIDI_CHAN_AFTERTOUCH    (0xD0)
#define MIDI_PITCH_BEND         (0xE0)
#define MIDI_SYSTEM             (0xF0)

#define MIDI_SYS_EX             (0x00)
#define MIDI_TIME_CODE          (0x01)
#define MIDI_SONG_POSITION      (0x02)
#define MIDI_SONG_SELECT        (0x03)
#define MIDI_RESERVED4          (0x04)
#define MIDI_RESERVED5          (0x05)
#define MIDI_TUNE_REQUEST       (0x06)
#define MIDI_SYS_EX_END         (0x07)
#define MIDI_TIMING_CLOCK       (0x08)
#define MIDI_RESERVED9          (0x09)
#define MIDI_START              (0x0A)
#define MIDI_CONTINUE           (0x0B)
#define MIDI_STOP               (0x0C)
#define MIDI_RESERVED13         (0x0D)
#define MIDI_ACTIVE_SENSING     (0x0E)
#define MIDI_SYS_RESET          (0x0F)


#define BAUDRATE (9600UL)
#define MIDIBAUD (31250UL)

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffers
struct UART_BUFFER U0Buf;
struct UART_BUFFER U1Buf;

uint8_t SavedRSTFR = 0;
volatile uint32_t Milliseconds = 0UL;
volatile uint8_t Tick = 0;
uint32_t TriggerOff = 0xffffffff;
const char NoteNames[12][3] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};


/* USART0_RXC_vect --- ISR for USART0 Receive Complete, used for Rx */

ISR(USART0_RXC_vect)
{
   const uint8_t tmphead = (U0Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
   const uint8_t ch = USART0.RXDATAL;  // Read received byte from UART
   
   if (tmphead == U0Buf.rx.tail)   // Is receive buffer full?
   {
       // Buffer is full; discard new byte
   }
   else
   {
      U0Buf.rx.head = tmphead;
      U0Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
   }
}


/* USART0_DRE_vect --- ISR for USART0 Data Register Empty, used for Tx */

ISR(USART0_DRE_vect)
{
   if (U0Buf.tx.head != U0Buf.tx.tail) // Is there anything to send?
   {
      const uint8_t tmptail = (U0Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
      
      U0Buf.tx.tail = tmptail;

      USART0.TXDATAL = U0Buf.tx.buf[tmptail];    // Transmit one byte
   }
   else
   {
      USART0.CTRLA &= ~(USART_DREIE_bm); // Nothing left to send; disable Tx interrupt
   }
}


/* USART1_RXC_vect --- ISR for USART1 Receive Complete, used for Rx */

ISR(USART1_RXC_vect)
{
   const uint8_t tmphead = (U1Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
   const uint8_t ch = USART1.RXDATAL;  // Read received byte from UART
   
   if (tmphead == U1Buf.rx.tail)   // Is receive buffer full?
   {
       // Buffer is full; discard new byte
   }
   else
   {
      U1Buf.rx.head = tmphead;
      U1Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
   }
}


/* USART1_DRE_vect --- ISR for USART1 Data Register Empty, used for Tx */

ISR(USART1_DRE_vect)
{
   if (U1Buf.tx.head != U1Buf.tx.tail) // Is there anything to send?
   {
      const uint8_t tmptail = (U1Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
      
      U1Buf.tx.tail = tmptail;

      USART1.TXDATAL = U1Buf.tx.buf[tmptail];    // Transmit one byte
   }
   else
   {
      USART1.CTRLA &= ~(USART_DREIE_bm); // Nothing left to send; disable Tx interrupt
   }
}


/* TCB0_OVF_vect --- ISR for Timer/Counter 0 overflow, used for 1ms ticker */

ISR(TCB0_INT_vect)
{
   TCB0.INTFLAGS = TCB_CAPT_bm;
   Milliseconds++;
   Tick = 1;
   PORTC.OUTTGL = SQWAVE_PIN;     // DEBUG: 500Hz on PC2 pin
}


/* millis --- return milliseconds since reset */

uint32_t millis(void)
{
   uint32_t ms;
   
   cli();
   ms = Milliseconds;
   sei();
   
   return (ms);
}


/* UART0RxByte --- read one character from UART0 via the circular buffer */

uint8_t UART0RxByte(void)
{
   const uint8_t tmptail = (U0Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U0Buf.rx.head == U0Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U0Buf.rx.tail = tmptail;
   
   return (U0Buf.rx.buf[tmptail]);
}


/* UART0TxByte --- send one character to UART0 via the circular buffer */

void UART0TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U0Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U0Buf.tx.tail)   // Wait, if buffer is full
       ;

   U0Buf.tx.buf[tmphead] = data;
   U0Buf.tx.head = tmphead;

   USART0.CTRLA |= USART_DREIE_bm;   // Enable UART0 Tx interrupt
}


/* USART0_printChar --- helper function to make 'stdio' functions work */

static int USART0_printChar(const char c, FILE *stream)
{
   if (c == '\n')
      UART0TxByte('\r');

   UART0TxByte(c);

   return (0);
}

static FILE USART_stream = FDEV_SETUP_STREAM(USART0_printChar, NULL, _FDEV_SETUP_WRITE);


/* UART0RxAvailable --- return true if a byte is available in UART0 circular buffer */

int UART0RxAvailable(void)
{
   return (U0Buf.rx.head != U0Buf.rx.tail);
}


/* UART1RxAvailable --- return true if a byte is available in UART1 circular buffer */

int UART1RxAvailable(void)
{
   return (U1Buf.rx.head != U1Buf.rx.tail);
}


/* UART1RxByte --- read one character from UART1 via the circular buffer */

uint8_t UART1RxByte(void)
{
   const uint8_t tmptail = (U1Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U1Buf.rx.head == U1Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U1Buf.rx.tail = tmptail;
   
   return (U1Buf.rx.buf[tmptail]);
}


/* UART1TxByte --- send one character to UART1 via the circular buffer */

void UART1TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U1Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U1Buf.tx.tail)   // Wait, if buffer is full
       ;

   U1Buf.tx.buf[tmphead] = data;
   U1Buf.tx.head = tmphead;

   USART1.CTRLA |= USART_DREIE_bm;   // Enable UART1 Tx interrupt
}


/* Spi0TxByte --- transmit a single byte on SPI0 */

uint8_t Spi0TxByte(const uint8_t byte)
{
   SPI0.DATA = byte;
   
   while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
      ;
      
   return (SPI0.DATA);
   
}


/* MidiNoteOn --- handle a MIDI note on message */

void MidiNoteOn(const int channel, const int note, const int velocity)
{
   const uint16_t dacNoteCmd = 0x3000 | (note << 5);
   
   PORTA.OUTCLR = DAC_CS_PIN; // MCP4822 /CS LOW
   
   Spi0TxByte(dacNoteCmd >> 8);
   Spi0TxByte(dacNoteCmd & 0xff);
   
   PORTA.OUTSET = DAC_CS_PIN; // MCP4822 /CS HIGH
   
   const uint16_t dacVelocityCmd = 0xb000 | (velocity << 5);
   
   PORTA.OUTCLR = DAC_CS_PIN; // MCP4822 /CS LOW
   
   Spi0TxByte(dacVelocityCmd >> 8);
   Spi0TxByte(dacVelocityCmd & 0xff);
   
   PORTA.OUTSET = DAC_CS_PIN; // MCP4822 /CS HIGH

   PORTC.OUTSET = GATE_PIN;      // GATE signal HIGH
   PORTC.OUTSET = TRIGGER_PIN;   // TRIGGER signal HIGH
   TriggerOff = millis() + 10;
   
   printf("%d NON %d %s%d %d\n", channel, note, NoteNames[note % 12], (note / 12) - 1, velocity);
}


/* MidiNoteOff --- handle a note off message */

void MidiNoteOff(const int channel, const int note, const int velocity)
{
   PORTC.OUTCLR = GATE_PIN;      // GATE signal LOW
   
   printf("%d NOFF %d %d\n", channel, note, velocity);
}


/* MidiProgramChange --- handle a program change message */

void MidiProgramChange(const int channel, const int program)
{
   printf("%d PC %d\n", channel, program);
}


/* MidiControlChange --- handle a control change message */

void MidiControlChange(const int channel, const int control, const int value)
{
   // TODO: set control DAC here
   
   printf("%d CC %d: %d\n", channel, control, value);
}


/* MidiPitchBend --- handle a pitch bend message */

void MidiPitchBend(const int channel, const int bend)
{
   // TODO: set pitch bend DAC here
   
   printf("%d PB %d\n", channel, bend);
}


/* midiRxByte --- state machine to deal with a single MIDI byte received from the UART */

void MidiRxByte(const uint8_t ch)
{
   static uint8_t midiStatus = 0u;
   static uint8_t midiChannel = 0u;
   static uint8_t midiNoteNumber = 0u;
   uint8_t midiVelocity = 0u;
   uint8_t midiProgram = 0u;
   static uint8_t midiControl = 0u;
   uint8_t midiValue = 0u;
   static uint8_t midiBendLo = 0u;
   uint8_t midiBendHi = 0u;
   static uint8_t midiByte = 0u;
   
   if (ch & 0x80) {              // It's a status byte
      midiStatus = ch & 0xF0;
      midiChannel = (ch & 0x0F) + 1;
      midiByte = 1;
      
      // TODO: check for System Messages here
   }
   else {                        // It's a data byte
      switch (midiByte) {
      case 1:
         if (midiStatus == MIDI_NOTE_ON || midiStatus == MIDI_NOTE_OFF) {
            midiNoteNumber = ch;
            midiByte++;
         }
         else if (midiStatus == MIDI_PROGRAM_CHANGE) {
            midiProgram = ch;
            midiByte = 1;
            
            MidiProgramChange(midiChannel, midiProgram);
         }
         else if (midiStatus == MIDI_CONTROL_CHANGE) {
            midiControl = ch;
            midiByte++;
         }
         else if (midiStatus == MIDI_PITCH_BEND) {
            midiBendLo = ch;
            midiByte++;
         }
         break;
      case 2:
         if (midiStatus == MIDI_NOTE_OFF) {
            midiVelocity = ch;
            midiByte = 1;
            
            MidiNoteOff(midiChannel, midiNoteNumber, midiVelocity);
         }
         else if (midiStatus == MIDI_NOTE_ON) {
            midiVelocity = ch;
            midiByte = 1;
            
            if (midiVelocity == 0)
               MidiNoteOff(midiChannel, midiNoteNumber, midiVelocity);
            else
               MidiNoteOn(midiChannel, midiNoteNumber, midiVelocity);
         }
         else if (midiStatus == MIDI_CONTROL_CHANGE) {
            midiValue = ch;
            midiByte = 1;
            
            MidiControlChange(midiChannel, midiControl, midiValue);
         }
         else if (midiStatus == MIDI_PITCH_BEND) {
            midiBendHi = ch;
            midiByte = 1;
            
            MidiPitchBend(midiChannel, (midiBendHi * 128) + midiBendLo);
         }
         break;
      }
   }
}


/* printDeviceID --- print the Device ID bytes as read from SIGROW */

void printDeviceID(void)
{
   printf("Device ID = %02x %02x %02x\n", SIGROW.DEVICEID0, SIGROW.DEVICEID1, SIGROW.DEVICEID2);
   printf("REVID = %02x\n", SYSCFG.REVID);
}


/* printSerialNumber --- print the chip's unique serial number */

void printSerialNumber(void)
{
   printf("Serial Number = %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                           SIGROW.SERNUM0, SIGROW.SERNUM1, SIGROW.SERNUM2,
                           SIGROW.SERNUM3, SIGROW.SERNUM4, SIGROW.SERNUM5,
                           SIGROW.SERNUM6, SIGROW.SERNUM7, SIGROW.SERNUM8,
                           SIGROW.SERNUM9);
}


/* printFuses --- print the fuse settings */

void printFuses(void)
{
   printf("FUSES.WDTCFG = 0x%02x\n", FUSE.WDTCFG);
   printf("FUSES.BODCFG = 0x%02x\n", FUSE.BODCFG);
   printf("FUSES.OSCCFG = 0x%02x\n", FUSE.OSCCFG);
   printf("FUSES.SYSCFG0 = 0x%02x\n", FUSE.SYSCFG0);
   printf("FUSES.SYSCFG1 = 0x%02x\n", FUSE.SYSCFG1);
   printf("FUSES.APPEND = 0x%02x\n", FUSE.APPEND);
   printf("FUSES.BOOTEND = 0x%02x\n", FUSE.BOOTEND);
}


/* printResetReason --- print the cause of the chip's reset */

void printResetReason(void)
{
   printf("RSTCTRL.RSTFR = 0x%02x\n", SavedRSTFR);
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc); // Select 20MHz RC oscillator
   
   //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm); // Divide-by-six
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc); // No divide-by-six

   SavedRSTFR = RSTCTRL.RSTFR;
   RSTCTRL.RSTFR = RSTCTRL_UPDIRF_bm | RSTCTRL_SWRF_bm | RSTCTRL_WDRF_bm |
                   RSTCTRL_EXTRF_bm | RSTCTRL_BORF_bm | RSTCTRL_PORF_bm;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Disable unused pins on the 40-pin DIP version of the ATmega4809
   // PORTB.PIN0CTRL |= PORT_PULLUPEN_bm;
   PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
   PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
   PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
   PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
   PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
   PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
   PORTC.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
   PORTC.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;

   PORTA.DIR = 0;
   PORTB.DIR = 0;
   PORTC.DIR = LED_PIN | SQWAVE_PIN | GATE_PIN | TRIGGER_PIN; // For LED, GATE, TRIGGER, and 500Hz signal
   PORTD.DIR = 0;
   PORTE.DIR = 0;
   PORTF.DIR = 0;

   PORTA.OUT = 0xFF;
   PORTB.OUT = 0xFF;
   PORTC.OUT = 0xF0;
   PORTD.OUT = 0xFF;
   PORTE.OUT = 0xFF;
   PORTF.OUT = 0xFF;
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   // Switch all UART pins to the default locations
   PORTMUX.USARTROUTEA = PORTMUX_USART0_DEFAULT_gc |
                         PORTMUX_USART1_DEFAULT_gc |
                         PORTMUX_USART2_DEFAULT_gc |
                         PORTMUX_USART3_DEFAULT_gc;

   // Set up UART0 and associated circular buffers
   U0Buf.tx.head = 0;
   U0Buf.tx.tail = 0;
   U0Buf.rx.head = 0;
   U0Buf.rx.tail = 0;

   USART0.BAUD = (F_CPU * 64UL) / (16UL * BAUDRATE);
   USART0.CTRLA = 0;
   USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
   USART0.CTRLA |= USART_RXCIE_bm;   // Enable UART0 Rx interrupt
   USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_RXMODE_NORMAL_gc;
   
   // Enable UART0 TxD pin
   PORTA.DIRSET = PIN0_bm;
   
   // Set up UART1 and associated circular buffers
   U1Buf.tx.head = 0;
   U1Buf.tx.tail = 0;
   U1Buf.rx.head = 0;
   U1Buf.rx.tail = 0;

   USART1.BAUD = (F_CPU * 64UL) / (16UL * MIDIBAUD);
   USART1.CTRLA = 0;
   USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
   USART1.CTRLA |= USART_RXCIE_bm;   // Enable UART1 Rx interrupt
   USART1.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_RXMODE_NORMAL_gc;
   
   // Enable UART1 TxD pin
   PORTC.DIRSET = PIN0_bm;
   
   stdout = &USART_stream;    // Allow use of 'printf' and similar functions
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up TCB0 for regular 1ms interrupt
   TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc;
   TCB0.CTRLB = TCB_CNTMODE_INT_gc;
   TCB0.CCMP = 9999;             // 10000 counts gives 1ms
   TCB0.CNT = 0;
   TCB0.INTCTRL = TCB_CAPT_bm;   // Enable interrupts
   TCB0.CTRLA |= TCB_ENABLE_bm;  // Enable timer
}


/* initSPI --- set up the SPI interface */

void initSPI(void)
{
   SPI0.CTRLA = SPI_MASTER_bm; // SPI prescaler divide-by 4 gives 5MHz
   
   SPI0.CTRLB = SPI_SSD_bm | SPI_MODE1_bm | SPI_MODE0_bm;
   SPI0.CTRLA |= SPI_ENABLE_bm;  // Enable SPI
   
   PORTA.DIRSET = PIN4_bm;    // Make sure PA4/MOSI (pin 37 on DIP-40) is an output
   PORTA.DIRSET = PIN6_bm;    // Make sure PA6/SCK (pin 39 on DIP-40) is an output
   PORTA.DIRSET = PIN7_bm;    // Make sure PA7/SS (pin 40 on DIP-40) is an output
}


int main(void)
{
   uint32_t end;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initSPI();
   initMillisecondTimer();
   
   sei();   // Enable interrupts
   
   printf("\nHello from the %s\n", "ATmega4809");
   printResetReason();
   printFuses();
   printDeviceID();
   printSerialNumber();
   
   PORTC.OUTCLR = GATE_PIN;      // GATE signal LOW initially on PC4
   PORTC.OUTCLR = TRIGGER_PIN;   // TRIGGER signal LOW initially on PC5
   
   end = millis() + 500UL;
   
   while (1) {
      if (Tick) {
         if (millis() >= end) {
            end = millis() + 500UL;
            PORTC.OUTTGL = LED_PIN;        // LED on PC3 toggle

            printf("millis() = %ld\n", millis());
         }
         
         if (millis() >= TriggerOff) {
            PORTC.OUTCLR = TRIGGER_PIN;   // TRIGGER signal LOW
            TriggerOff = 0xffffffff;
         }
         
         // TODO: Nudge watchdog here
         Tick = 0;
      }
      
      if (UART0RxAvailable()) {
         const uint8_t ch = UART0RxByte();
         
         printf("UART0: %02x\n", ch);
         switch (ch) {
         case 'f':
         case 'F':
            printFuses();
            break;
         case 'i':
         case 'I':
            printDeviceID();
            break;
         case 'n':
         case 'N':
            printSerialNumber();
            break;
         case 'r':
         case 'R':
            printResetReason();
            break;
         }
      }
      
      if (UART1RxAvailable()) {
         const uint8_t ch = UART1RxByte();
         
         //printf("UART1: %02x\n", ch);
         MidiRxByte(ch);
      }
   }
}
