#include "uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>

// Millisecond-based convenience macros
#define ms400 ((uint16_t)25000U) // 400ms = 0.4s = 62,500clk/s * 0.4s = 25000clk
#define ms200 ((uint16_t)12500U)
#define ms50 ((uint16_t)3125U)
#define ms30 ((uint16_t)1875U)
#define msINF (~(uint16_t)0) // more than CTC reset & never met, effectively inf
#define LED_BLINK_TIME ms50

//%%%%%%%%%%%%%%%% Debug printing: enable to see all the shit I went through <3
#define DEBUG_send(value)      // UART_send(value)
#define DEBUG_putstring(value) // UART_putstring(value)

//%%%%%%%%%%%%%%%% Globals
static unsigned char decode_state; // <30ms => 1; >30 => 2; >200 => 4
static unsigned char morse_seq;    // interpreted as 4x(2 bits): all codes fit
#define SET_DECODE_STATE(value)                                                \
  do {                                                                         \
    DEBUG_putstring("[decode_state set from ");                                \
    DEBUG_send('0' + decode_state);                                            \
    decode_state = (value);                                                    \
    PORTB &= ~((1U << PB1) | (1U << PB2)); /* Turn off both LEDs */            \
    DEBUG_putstring(" to ");                                                   \
    DEBUG_send('0' + decode_state);                                            \
    DEBUG_putstring(" in ");                                                   \
    DEBUG_putstring(__func__);                                                 \
    DEBUG_send(']');                                                           \
  } while (0) // while (0) allows a semicolon
#define PRINT_MORSE_SEQ()                                                      \
  do {                                                                         \
    DEBUG_send((1 & (morse_seq >> 7)) ? '1' : '0');                            \
    DEBUG_send((1 & (morse_seq >> 6)) ? '1' : '0');                            \
    DEBUG_send((1 & (morse_seq >> 5)) ? '1' : '0');                            \
    DEBUG_send((1 & (morse_seq >> 4)) ? '1' : '0');                            \
    DEBUG_send((1 & (morse_seq >> 3)) ? '1' : '0');                            \
    DEBUG_send((1 & (morse_seq >> 2)) ? '1' : '0');                            \
    DEBUG_send((1 & (morse_seq >> 1)) ? '1' : '0');                            \
    DEBUG_send((1 & (morse_seq >> 0)) ? '1' : '0');                            \
  } while (0)
#define SET_MORSE_SEQ(value)                                                   \
  do {                                                                         \
    DEBUG_putstring("[morse_seq set from ");                                   \
    PRINT_MORSE_SEQ();                                                         \
    morse_seq = (value);                                                       \
    DEBUG_putstring(" to ");                                                   \
    PRINT_MORSE_SEQ();                                                         \
    DEBUG_putstring(" in ");                                                   \
    DEBUG_putstring(__func__);                                                 \
    DEBUG_send(']');                                                           \
  } while (0) // while (0) allows a semicolon

//%%%%%%%%%%%%%%%% Utility functions
#define SET_INTERRUPT_TIME(value)                                              \
  do {                                                                         \
    OCR1B = (value);                                                           \
  } while (0) // while (0) allows a semicolon

#define DECODE_AND_SEND()                                                      \
  do {                                                                         \
    if (decode_state) {                                                        \
      DEBUG_putstring("[ERROR: Decoding with nonzero state]");                 \
    }                                                                          \
    DEBUG_send('[');                                                           \
    PRINT_MORSE_SEQ();                                                         \
    DEBUG_send(' ');                                                           \
    switch (morse_seq) {                                                       \
    case 0b01:                                                                 \
      UART_send('E');                                                          \
      break;                                                                   \
    case 0b0101:                                                               \
      UART_send('I');                                                          \
      break;                                                                   \
    case 0b010101:                                                             \
      UART_send('S');                                                          \
      break;                                                                   \
    case 0b01010101:                                                           \
      UART_send('H');                                                          \
      break;                                                                   \
    case 0b01010110:                                                           \
      UART_send('V');                                                          \
      break;                                                                   \
    case 0b010110:                                                             \
      UART_send('U');                                                          \
      break;                                                                   \
    case 0b01011001:                                                           \
      UART_send('F');                                                          \
      break;                                                                   \
    case 0b0110:                                                               \
      UART_send('A');                                                          \
      break;                                                                   \
    case 0b011001:                                                             \
      UART_send('R');                                                          \
      break;                                                                   \
    case 0b01100101:                                                           \
      UART_send('L');                                                          \
      break;                                                                   \
    case 0b011010:                                                             \
      UART_send('W');                                                          \
      break;                                                                   \
    case 0b01101001:                                                           \
      UART_send('P');                                                          \
      break;                                                                   \
    case 0b01101010:                                                           \
      UART_send('J');                                                          \
      break;                                                                   \
    case 0b10:                                                                 \
      UART_send('T');                                                          \
      break;                                                                   \
    case 0b1001:                                                               \
      UART_send('N');                                                          \
      break;                                                                   \
    case 0b100101:                                                             \
      UART_send('D');                                                          \
      break;                                                                   \
    case 0b10010101:                                                           \
      UART_send('B');                                                          \
      break;                                                                   \
    case 0b10010110:                                                           \
      UART_send('X');                                                          \
      break;                                                                   \
    case 0b100110:                                                             \
      UART_send('K');                                                          \
      break;                                                                   \
    case 0b10011001:                                                           \
      UART_send('C');                                                          \
      break;                                                                   \
    case 0b10011010:                                                           \
      UART_send('Y');                                                          \
      break;                                                                   \
    case 0b1010:                                                               \
      UART_send('M');                                                          \
      break;                                                                   \
    case 0b101001:                                                             \
      UART_send('G');                                                          \
      break;                                                                   \
    case 0b10100101:                                                           \
      UART_send('Z');                                                          \
      break;                                                                   \
    case 0b10100110:                                                           \
      UART_send('Q');                                                          \
      break;                                                                   \
    case 0b101010:                                                             \
      UART_send('O');                                                          \
      break;                                                                   \
    default:                                                                   \
      UART_putstring("[unrecognized]");                                        \
      break;                                                                   \
    }                                                                          \
    DEBUG_putstring("]\r\n\r\n");                                              \
    morse_seq = 0;                                                             \
  } while (0) // while (0) allows a semicolon

//%%%%%%%%%%%%%%%% Interrupts
ISR(TIMER1_COMPB_vect) {
  // Meaningful expiration interrupt
  switch (decode_state) {
  case 0:                      // 400ms after RELEASE: space
    SET_INTERRUPT_TIME(msINF); // Disable next cycle
    DEBUG_putstring("[Clock interrupt state 0]");
    DECODE_AND_SEND();
    break;
  case 1: // 30ms: we now have a valid button-press
    SET_INTERRUPT_TIME((TCNT1 + (ms200 - ms30)) % ms400);
    DEBUG_putstring("[Clock interrupt state 1]");
    SET_DECODE_STATE(2);
    break;
  case 2:
    SET_INTERRUPT_TIME(msINF);
    DEBUG_putstring("[Clock interrupt state 2]");
    SET_DECODE_STATE(4); // dash
    break;
  case 4:
    SET_INTERRUPT_TIME(msINF); // Disable next cycle
    DEBUG_putstring("[ERROR: Clock interrupt state 4]");
    SET_DECODE_STATE(0);
    break;
  default:
    SET_INTERRUPT_TIME(msINF); // Disable next cycle
    DEBUG_putstring("[ERROR: Unrecognized decode_state ");
    DEBUG_send('0' + (decode_state / 100));
    DEBUG_send('0' + ((decode_state % 100) / 10));
    DEBUG_send('0' + (decode_state % 10));
    DEBUG_putstring(" in clock interrupt]");
    SET_DECODE_STATE(0);
    break;
  }
}

ISR(PCINT2_vect) {
  // Button-pressed-OR-let-go interrupt
  if (PIND & (1U << PD7)) {
    // Button pressed
    if (!decode_state) {
      SET_INTERRUPT_TIME((TCNT1 + ms30) % ms400);
      DEBUG_putstring("[down]");
      SET_DECODE_STATE(1); // Reset timer: <30ms, might not be valid yet
    }
  } else {
    // Button up
    if (decode_state) {
      if (decode_state & 9) {
        SET_INTERRUPT_TIME(msINF);
        DEBUG_putstring("[up with state ");
        DEBUG_send('0' + decode_state);
        DEBUG_putstring("]");
        SET_DECODE_STATE(0);
      } else {
        SET_INTERRUPT_TIME((uint16_t)(TCNT1 - 1U) % ms400);
        // SET_INTERRUPT_TIME((TCNT1 + LED_BLINK_TIME) % ms400);
        DEBUG_putstring("[up]");
        switch (decode_state) {
        case 2:                                 // 30-200ms: dot
          SET_MORSE_SEQ((morse_seq << 2) | 1U); // ...[xx][xx][01]
          SET_DECODE_STATE(0);
          PORTB |= (1U << PB1);
          break;
        case 4:                                 // 200+ms: dash
          SET_MORSE_SEQ((morse_seq << 2) | 2U); // ...[xx][xx][10]
          SET_DECODE_STATE(0);
          PORTB |= (1U << PB2);
          break;
        default:
          SET_INTERRUPT_TIME(msINF); // Disable next cycle
          DEBUG_putstring("[ERROR: Unrecognized decode_state ");
          DEBUG_send('0' + (decode_state / 100));
          DEBUG_send('0' + ((decode_state % 100) / 10));
          DEBUG_send('0' + (decode_state % 10));
          DEBUG_putstring(" in button interrupt]");
          SET_DECODE_STATE(0);
          break;
        }
      }
    }
  }
}

//%%%%%%%%%%%%%%%% Entry point
int main(void) {

  // Temporarily disable interrupts
  cli();

  // Prescale the 16MHz timer (#1) over 256 -> 62,500 clk/s
  TCCR1B = ((TCCR1B & ~((1U << CS10) | (1U << CS11))) | (1U << CS12));

  // Enable Clear-Timer-on-Compare (CTC) mode
  TCCR1A &= ~((1U << WGM10) | (1U << WGM11));
  TCCR1B = ((TCCR1B | (1U << WGM12)) & ~(1U << WGM13));

  // Clear @ 400ms
  OCR1A = ms400;
  SET_INTERRUPT_TIME(msINF); // INF never called b/c 0xffff=65535 > 25000=400ms

  // Enable timer-overflow interrupts
  TIMSK1 |= (1U << OCIE1B);

  // Set P9&10 to output
  DDRB |= ((1U << DDB1) | (1U << DDB2));

  // Set P7 to input
  DDRD &= ~(1U << DDD7);
  PORTD |= (1U << PD7);

  // Enable interrupts (Pin 7 := PCINT23)
  PCICR |= (1U << PCIE2); // PCIE2 enables PCINT[16..23]
  PCMSK2 |= (1U << PCINT23);

  // Initialize UART
  UART_init(((F_CPU / (9600UL * 16UL))) - 1U);
  UART_putstring("\rMorse translation:\r\n");

  // Initialize globals
  SET_DECODE_STATE(0);
  SET_MORSE_SEQ(0);

  // Re-enable interrupts
  sei();

  // Sit around & wait for interrupts
  do {
  } while (1);
}
