/*

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define LED_BITS ((1U << PB1) | (1U << PB2) | (1U << PB3) | (1U << PB4))

ISR(PCINT2_vect) {
  static unsigned char flip = 0;
  if (!flip) {
    flip = 1;
    PORTB = (PORTB & ~LED_BITS) | (((PORTB & LED_BITS) << 1) & LED_BITS);
    if (!(PORTB & LED_BITS)) {
      PORTB = (PORTB & ~LED_BITS) | (1U << PB1);
    }
  } else {
    flip = 0;
  }
}

int main(void) {

  cli(); // Disable all interrupts (temporarily)

  // Set P9-12 to output
  DDRB |= ((1U << DDB1) | (1U << DDB2) | (1U << DDB3) | (1U << DDB4));

  // Set P7 to input
  DDRD &= ~(1U << DDD7);
  PORTD |= (1U << PD7);

  // Turn P9 on
  PORTB |= (1U << PB1);

  // Enable interrupts (Pin 7 := PCINT23)
  PCICR |= (1U << PCIE2); // PCIE2 enables PCINT[16..23]
  PCMSK2 |= (1U << PCINT23);

  sei(); // Re-enable interrupts

  do {
  } while (1);
}

*/
