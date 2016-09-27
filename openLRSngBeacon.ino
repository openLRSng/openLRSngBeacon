// **********************************************************
// ************************ openLRSngBeacon *****************
// **********************************************************
// ** by Kari Hautio - kha @ AeroQuad/RCGroups/IRC(Freenode)/FPVLAB etc.
//
// Lost plane finder beacon using openLRS RX module
//
// Donations for development tools and utilities (beer) here
// https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=DSWGKGKPRX5CS

// 3 = OpenLRS Rx v2 Board or OrangeRx UHF RX
#define BOARD_TYPE 3

//###### SERIAL PORT SPEED - just debugging atm. #######
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed

// helpper macro for European PMR and US FRS channels
#define EU_PMR_CH(x) (445993750L + 12500L * (x)) // valid for ch 1 - 8
#define US_FRS_CH(x) (462537500L + 25000L * (x)) // valid for ch 1 - 7

#define BEACON_FREQUENCY EU_PMR_CH(1)

#define BEACON_DEADTIME 30 // time to wait until going into beacon mode (s)
#define BEACON_INTERVAL 10 // interval between beacon transmits (s)

#define FULL_POWER_MODE 0  // Set to 1 for 100mW operation, need HAM license

// power levels with RFM22B
// 7 - 100mW
// 6 - 50mW
// 5 - 25mW
// 4 - 13mW
// 3 - 6mW
// 2 - 3mW
// 1 - 1.6mW
// 0 - 1.3mW


#if (FULL_POWER_MODE == 1)
// 100/15/1mW better range
#define BEACON_POWER_HI  0x07  // 100mW
#define BEACON_POWER_MED 0x04  // 13mW
#define BEACON_POWER_LOW 0x00  // 1.3mW
#else
// 25/6/1.3mW mostly legal
#define BEACON_POWER_HI  0x05  // 25mW
#define BEACON_POWER_MED 0x03  // 6mW
#define BEACON_POWER_LOW 0x00  // 1.1mW
#endif

// Servovalues considered 'good' i.e. beacon will not activate when it is fed
// with PWM within these limits (feed via ch4 connector)
#define MINPWM 1000
#define MAXPWM 1500

//####################
//### CODE SECTION ###
//####################

#include <Arduino.h>
#include <EEPROM.h>

#if (BOARD_TYPE == 3)

#define USE_ICP1 // use ICP1 for PPM input for less jitter

#define PPM_IN 8 // ICP1

#define Red_LED A3
#define Green_LED 13

#define Red_LED_ON  PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(3);    // Was originally #define Green_LED_OFF  PORTB |= _BV(5);   E.g turns it ON not OFF

#define Green_LED_ON  PORTB |= _BV(5);
#define Green_LED_OFF  PORTB &= ~_BV(5);

//## RFM22B Pinouts for Public Edition (Rx v2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTC |= (1<<2) //A2
#define  SCK_off PORTC &= 0xFB //A2

#define  SDI_on PORTC |= (1<<1) //A1
#define  SDI_off PORTC &= 0xFD //A1

#define  SDO_1 (PINC & 0x01) == 0x01 //A0
#define  SDO_0 (PINC & 0x01) == 0x00 //A0

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin A2
#define IRQ_pin 2
#define nSel_pin 4

#define IRQ_interrupt 0

#endif

uint8_t spiReadRegister(uint8_t address);
void spiWriteRegister(uint8_t address, uint8_t data);

#define NOP() __asm__ __volatile__("nop")

#define RF22B_PWRSTATE_POWERDOWN    0x00
#define RF22B_PWRSTATE_READY        0x01
#define RF22B_PACKET_SENT_INTERRUPT 0x04
#define RF22B_PWRSTATE_RX           0x05
#define RF22B_PWRSTATE_TX           0x09

#define RF22B_Rx_packet_received_interrupt   0x02

uint8_t ItStatus1, ItStatus2;

void spiWriteBit(uint8_t b);

void spiSendCommand(uint8_t command);
void spiSendAddress(uint8_t i);
uint8_t spiReadData(void);
void spiWriteData(uint8_t i);


// **** SPI bit banging functions

void spiWriteBit(uint8_t b)
{
  if (b) {
    SCK_off;
    NOP();
    SDI_on;
    NOP();
    SCK_on;
    NOP();
  } else {
    SCK_off;
    NOP();
    SDI_off;
    NOP();
    SCK_on;
    NOP();
  }
}

uint8_t spiReadBit(void)
{
  uint8_t r = 0;
  SCK_on;
  NOP();

  if (SDO_1) {
    r = 1;
  }

  SCK_off;
  NOP();
  return r;
}

void spiSendCommand(uint8_t command)
{
  nSEL_on;
  SCK_off;
  nSEL_off;

  for (uint8_t n = 0; n < 8 ; n++) {
    spiWriteBit(command & 0x80);
    command = command << 1;
  }

  SCK_off;
}

void spiSendAddress(uint8_t i)
{
  spiSendCommand(i & 0x7f);
}

void spiWriteData(uint8_t i)
{
  for (uint8_t n = 0; n < 8; n++) {
    spiWriteBit(i & 0x80);
    i = i << 1;
  }

  SCK_off;
}

uint8_t spiReadData(void)
{
  uint8_t Result = 0;
  SCK_off;

  for (uint8_t i = 0; i < 8; i++) {   //read fifo data byte
    Result = (Result << 1) + spiReadBit();
  }

  return(Result);
}

uint8_t spiReadRegister(uint8_t address)
{
  uint8_t result;
  spiSendAddress(address);
  result = spiReadData();
  nSEL_on;
  return(result);
}

void spiWriteRegister(uint8_t address, uint8_t data)
{
  address |= 0x80; //
  spiSendCommand(address);
  spiWriteData(data);
  nSEL_on;
}

void rfmSetCarrierFrequency(uint32_t f)
{
  uint16_t fb, fc, hbsel;
  if (f < 480000000) {
    hbsel = 0;
    fb = f / 10000000 - 24;
    fc = (f - (fb + 24) * 10000000) * 4 / 625;
  } else {
    hbsel = 1;
    fb = f / 20000000 - 24;
    fc = (f - (fb + 24) * 20000000) * 2 / 625;
  }
  spiWriteRegister(0x75, 0x40 + (hbsel?0x20:0) + (fb & 0x1f));
  spiWriteRegister(0x76, (fc >> 8));
  spiWriteRegister(0x77, (fc & 0xff));
}

void beacon_tone(int16_t hz, int16_t len)
{
  int16_t d = 500 / hz; // somewhat limited resolution ;)

  if (d < 1) {
    d = 1;
  }

  int16_t cycles = (len * 1000 / d);

  for (int16_t i = 0; i < cycles; i++) {
    SDI_on;
    delay(d);
    SDI_off;
    delay(d);
  }
}

void beaconSend(void)
{
  Green_LED_ON
  ItStatus1 = spiReadRegister(0x03);   // read status, clear interrupt
  ItStatus2 = spiReadRegister(0x04);
  spiWriteRegister(0x06, 0x00);    // no wakeup up, lbd,
  spiWriteRegister(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  spiWriteRegister(0x09, 0x7f);  // (default) c = 12.5p
  spiWriteRegister(0x0a, 0x05);
  spiWriteRegister(0x0b, 0x12);    // gpio0 TX State
  spiWriteRegister(0x0c, 0x15);    // gpio1 RX State
  spiWriteRegister(0x0d, 0xfd);    // gpio 2 micro-controller clk output
  spiWriteRegister(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  spiWriteRegister(0x70, 0x2C);    // disable manchest

  spiWriteRegister(0x30, 0x00);    //disable packet handling

  spiWriteRegister(0x79, 0);    // start channel

  spiWriteRegister(0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

  spiWriteRegister(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  spiWriteRegister(0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  spiWriteRegister(0x73, 0x00);
  spiWriteRegister(0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(BEACON_FREQUENCY);

  spiWriteRegister(0x6d, BEACON_POWER_HI);

  delay(10);
  spiWriteRegister(0x07, RF22B_PWRSTATE_TX);    // to tx mode
  delay(10);
  beacon_tone(500, 1);

  spiWriteRegister(0x6d, BEACON_POWER_MED);
  delay(10);
  beacon_tone(250, 1);

  spiWriteRegister(0x6d, BEACON_POWER_LOW);
  delay(10);
  beacon_tone(160, 1);

  spiWriteRegister(0x07, RF22B_PWRSTATE_POWERDOWN);
  Green_LED_OFF
}


uint32_t beaconDelay = BEACON_DEADTIME;

volatile uint16_t startPulse = 0;

#define TIMER1_PRESCALER    8

/****************************************************
 * Interrupt Vector
 ****************************************************/
ISR(TIMER1_CAPT_vect)
{
  if (TCCR1B & (1 << ICES1)) {
    startPulse = ICR1;
  } else {
    uint16_t pulseWidth = ICR1 - startPulse;
    if ((pulseWidth >= MINPWM*2) && (pulseWidth <= MAXPWM * 2)) {
      beaconDelay = BEACON_DEADTIME;
    }
  }
  TCCR1B ^= (1 << ICES1); // change trigger edge
}

void setupPPMinput()
{
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << ICES1));
  OCR1A = 0xffff;
  TIMSK1 |= (1 << ICIE1);   // Enable timer1 input capture interrupt
}

void setup(void)
{

  //RF module pins
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL

  //LED and other interfaces
  pinMode(Red_LED, OUTPUT);   //RED LED
  pinMode(Green_LED, OUTPUT);   //GREEN LED

  pinMode(PPM_IN, INPUT);   //PPM from TX
  digitalWrite(PPM_IN, HIGH); // enable pullup for TX:s with open collector output

  Serial.begin(SERIAL_BAUD_RATE);

  setupPPMinput();

  sei();

  beaconDelay = BEACON_DEADTIME;
}

void loop(void)
{
  if (0 == beaconDelay) {
    Red_LED_ON;
    beaconSend();
    Red_LED_OFF;
    beaconDelay = BEACON_INTERVAL;
  } else {
    Green_LED_ON;
    delay(2);
    Green_LED_OFF;
    beaconDelay--;
  }
  delay(1000);
}

