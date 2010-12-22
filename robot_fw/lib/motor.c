#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

void pwm_init(void)
{
  // Enable output
  DDRB |= 0x02 | 0x04;
  // Set output compare
  OCR1A = 0;
  OCR1B = 0;
  // Set TOP
  ICR1 = 100;
  // Set WGM[13:10] to 1000 (phase and frequency correct PWM mode, TOP = ICR1)
  TCCR1A &= ~0x03;
  TCCR1B &= ~0x08;
  TCCR1B |=  0x10;
  // Set COM1A[1:0] and COM1B[1:0] to 10 (set/clear OC1A/OC1B on compare match)
  TCCR1A |=  0x80 | 0x20;
  TCCR1A &= ~(0x40 | 0x10);
  // Set clock source to CLKIO, no prescaler (CS1[2:0] = 001)
  TCCR1B &= ~0x06;
  TCCR1B |= 0x01;
}

void motor_init(void)
{
  DDRB |= 0xF0;  // Init PB7..PB4 as outputs
  PORTB &= ~0xF0;
  
  pwm_init();
}

void set_speed(int8_t left, int8_t right)
{
  uint8_t dir = 0;
  
  // extract direction
  if(left >= 0) {
    dir |= 0x20;
  } else {
    dir |= 0x10;
    left = -left;
  }
  
  if(right >= 0) {
    dir |= 0x40;
  } else {
    dir |= 0x80;
    right = -right;
  }
  
  // clip values
  if(left > 100) left = 100;
  if(right > 100) right = 100;
  
  // Avoid shoot-through (allow MOSFET gates to discharge)
  if((PORTB & 0xF0) != dir) {
    OCR1A = OCR1B = 0;
    _delay_ms(2);
    PORTB = (PORTB & 0x0F) | dir;
  }
  
  OCR1A = left;
  OCR1B = right;
}
