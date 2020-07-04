#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// Signal IN
#define SIG_IN PB3
#define SIG_IN_INTER PCINT3
#define SIG_IN_PORT PORTB
#define SIG_IN_PIN PINB
#define SIG_IN_DDR DDRB

// LED
#define LED PB2
#define LED_PORT PORTB
#define LED_DDR DDRB

// Signal OUT
#define SIG_OUT PB4
#define SIG_OUT_PORT PORTB
#define SIG_OUT_DDR DDRB

// Code to switch the amp on / off

// lenght of each bit in microseconds
#define BIT_LENGTH 1040

// length of a pulse in us when sending a 1 (38kHz modulated)
#define PULSE_LENGTH 310

#define CODE1_PACKET_GAP 48
#define CODE2_PACKET_GAP 42

// the length of a full 37 kHz swing
#define KHZ_37 (1000000 / 37000)
// the length of each half-swing @ 37kHz
#define KHZ_37_PULSE ((1000000 / 37000) / 2)
// the error due to integer rounding in us
#define KHZ_37_ERR KHZ_37 - (KHZ_37_PULSE * 2)

// number of times the ir packet is repeated
#define PACKET_REPITITIONS 3

// the ir code is divided into 2 different packets
uint8_t code1[] = {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1};
uint8_t code2[] = {1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1,
                   0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1};
#define CODE1_LENGTH 19
#define CODE2_LENGTH 25

// sencods to wait for another audio sample before switching the receiver off
#define RECEIVER_ON_TIME_IN_SILENCE 5 * 60

volatile uint8_t signalPresent = 0;
volatile uint16_t receiverOnTime = RECEIVER_ON_TIME_IN_SILENCE;

void setupIO() {
  SIG_IN_DDR &= ~(1 << SIG_IN_PIN);

  LED_DDR |= (1 << LED);

  SIG_OUT_DDR |= (1 << SIG_OUT);
}

void setupInterrupts(void)
{
  // pin change interrupt enable
	GIMSK |= (1 << PCIE);
  // pin change interrupt enabled for PCINT3
	PCMSK |= (1 << SIG_IN_INTER);


  // clear on compare match, 8192 prescaler
  TCCR1 = (1 << CTC1) | (1 << CS13) | (1 << CS12) | (1 << CS11);
  // set compare for ~10 interrupts per  second (8MHz / pescaler)
  OCR1A = 122;
  OCR1C = 122;

  // enable interrupts
	sei();
}

void enableTimer(){
  // setup for compare interrupts
  TIMSK |= 1 << OCIE1A;
}

void disableTimer(){
  // setup for compare interrupts
  TIMSK &= ~(1 << OCIE1A);
}

// ISR for the pin change interrupt
ISR(PCINT0_vect)
{
  signalPresent = SIG_IN_PIN & (1 << SIG_IN);
}

// ISR for the 1 second timer
ISR(TIMER1_COMPA_vect) {
  receiverOnTime--;
}

void sendMark() {
  // on time
  for (uint8_t i = 0; i < PULSE_LENGTH / (KHZ_37); i++) {
    SIG_OUT_PORT |= (1 << SIG_OUT);
    _delay_us(KHZ_37 / 2);
    SIG_OUT_PORT &= ~(1 << SIG_OUT);
    _delay_us((KHZ_37 / 2) + KHZ_37_ERR);
  }

  // off time
  _delay_us(BIT_LENGTH - PULSE_LENGTH);
}

void sendSpace() {
  // space
  _delay_us(BIT_LENGTH);
}

void switchReceiver() {
  for (uint8_t j = 0; j < PACKET_REPITITIONS; j++) {
    for (uint8_t i = 0; i < CODE1_LENGTH; i++) {
      if (code1[i]) {
        sendMark();
      } else {
        sendSpace();
      }
    }
    // wait between packets
    _delay_ms(CODE1_PACKET_GAP);

    for (uint8_t i = 0; i < CODE2_LENGTH; i++) {
      if (code2[i]) {
        sendMark();
      } else {
        sendSpace();
      }
    }

    // wait between packets
    _delay_ms(CODE2_PACKET_GAP);
  }
}

int main() {
  setupIO();
  setupInterrupts();

  // wait one second until the power suppy is stabilized
  // and the audio detection circuit has settled
  _delay_ms(1000);


  bool receiverWasOn = false;
  signalPresent = SIG_IN_PIN & (1 << SIG_IN);

  while (true) {

    cli();

    if(signalPresent && !receiverWasOn){
      switchReceiver();
      enableTimer();
      receiverWasOn = true;
      //switch on the indicator LED
      LED_PORT |= (1 << LED);
    }

    if(signalPresent){
      receiverOnTime = RECEIVER_ON_TIME_IN_SILENCE; 
    }

    set_sleep_mode(SLEEP_MODE_IDLE);

    if(receiverOnTime == 0){
      switchReceiver();
      receiverWasOn = false;
      disableTimer();

      //switch off the indicator LED
      LED_PORT &= ~(1 << LED);

      //sleep a minimum number of 5 seconds after switching the receiver
      // to avoid switching accidently while ist shuttong down / booting
      _delay_ms(5000);

      //set to sleep mode power down because we can now stop the timer
      // and only wake up due to a PCINT event
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    }

    sei();

    // go to sleep
    sleep_mode();

  }
}
