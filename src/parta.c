/*

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define LED_BITS ((1U << PB1) | (1U << PB2) | (1U << PB3) | (1U << PB4))

int main(void) {

  unsigned char btn_enabled_last_clk = 0;

  cli(); // Disable all interrupts (temporarily)

  // Set P9-12 to output
  DDRB |= ((1U << DDB1) | (1U << DDB2) | (1U << DDB3) | (1U << DDB4));

  // Set P7 to input
  DDRD &= ~(1U << DDD7);
  PORTD |= (1U << PD7);

  // Turn P9 on
  PORTB |= (1U << PB1);

  sei(); // Re-enable interrupts

  do {
    if (PIND & (1U << PD7)) {
      if (!btn_enabled_last_clk) {
        btn_enabled_last_clk = 1;
        PORTB = (PORTB & ~LED_BITS) | (((PORTB & LED_BITS) << 1) & LED_BITS);
        if (!(PORTB & LED_BITS)) {
          PORTB = (PORTB & ~LED_BITS) | (1U << PB1);
        }
      }
      _delay_ms(100);
    } else if (btn_enabled_last_clk) {
      btn_enabled_last_clk = 0;
    }
  } while (1);
}

*/
