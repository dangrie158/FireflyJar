#include <stdint.h>
#include <stdlib.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

typedef uint8_t bright_t;
typedef uint8_t time_t;

const bright_t maxBrightness = 250;
const time_t maxPause = 100;

const uint8_t numChannels = 8;
static volatile uint8_t *ledPORTs[] = {&PORTB, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTB, &PORTB};
static volatile uint8_t *ledDDRs[] = {&DDRB, &DDRA, &DDRA, &DDRA, &DDRA, &DDRA, &DDRB, &DDRB};
const uint8_t ledPINs[] = {PB2, PA7, PA3, PA2, PA1, PA0, PB0, PB1};

// the current brightness the channel should fade to
bright_t targetBrightness[numChannels];
volatile bright_t currentBrightness[numChannels];
bright_t pauseDuration[numChannels];

int main()
{
  for (uint8_t channel = 0; channel < numChannels; channel++)
  {
    // set all led channel as outputs
    *(ledDDRs[channel]) |= (1 << ledPINs[channel]);
  }

  // Setup the Timer0
  cli();
  // F_CPU / (127 * 1)/255 ~= 247 Hz
  OCR0A = F_CPU / ( 500* 1 ) /255;
  // CTC
  TCCR0A |= (1 << WGM01);
  // no prescaler
  TCCR0B |= (1 << CS00);
  // Output Compare Match A Interrupt Enable
  TIMSK0 |= (1 << OCIE0A);
  // enable interrupts
  sei();

  while (1)
  {
    //it's time to update the led state
    for (uint8_t channel = 0; channel < numChannels; channel++)
    {
      if (pauseDuration[channel] == 0)
      {
        if (currentBrightness[channel] < targetBrightness[channel])
        {
          // fade up
          currentBrightness[channel] += 1;
        }
        if (currentBrightness[channel] == targetBrightness[channel])
        {
          // fade up ended
          targetBrightness[channel] = 0;
        }
        if (currentBrightness[channel] > targetBrightness[channel])
        {
          // fade down
          currentBrightness[channel] -= 1;
        }
        if (currentBrightness[channel] == 0 && targetBrightness[channel] == 0)
        {
          //fade down ended
          pauseDuration[channel] = rand() % maxPause;
          targetBrightness[channel] = rand() % maxBrightness;
        }
      }
      else
      {
        pauseDuration[channel] -= 1;
      }
    }
  _delay_ms(5);
  }

}

ISR(TIM0_COMPA_vect)
{
  static bright_t pwmStep = 0;

  // soft PWM routine 
  for (uint8_t channel = 0; channel < numChannels; channel++)
  {
    if (pwmStep < currentBrightness[channel])
    {
      *(ledPORTs[channel]) |= (1 << ledPINs[channel]);
    }
    else
    {
      *(ledPORTs[channel]) &= ~(1 << ledPINs[channel]);
    }
  }

  ++pwmStep;
}