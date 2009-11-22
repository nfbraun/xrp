#include "global.h"

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

void SetPWM(uint16_t val);

uint8_t cmdbuf[8];
volatile int8_t cmd_ready;
uint8_t nread;

ISR(USART_RXC_vect)
{
    uint8_t c;

    c = UDR;
    if(bit_is_clear(UCSRA, FE)) {
        cmdbuf[nread++] = c;
        if(nread > (cmdbuf[0] & 0x07)) {
            cmd_ready = 1;
            nread = 0;
        }
    }
}

void InitSerial(void)
{
    UCSRB = _UV(TXEN) | _UV(RXEN) | _UV(RXCIE);
    UCSRC = _UV(URSEL) | _UV(UCSZ1) | _UV(UCSZ0); // 8 bit, no parity, 1 stop
    UBRRL = 12;  // 38.4 kbps
}

void InitPWM(void)
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

void InitMotor(void)
{
  DDRC = 0x0F;  // Init PC3..PC0 as outputs
  PORTC = 0x00;
}

void SetSpeed(int8_t left, int8_t right)
{
  uint8_t dir = 0;
  
  // extract direction
  if(left >= 0) {
    dir |= 0x02;
  } else {
    dir |= 0x01;
    left = -left;
  }
  
  if(right >= 0) {
    dir |= 0x04;
  } else {
    dir |= 0x08;
    right = -right;
  }
  
  // clip values
  if(left > 100) left = 100;
  if(right > 100) right = 100;
  
  // Avoid shoot-through (allow MOSFET gates to discharge)
  if((PORTC & 0x0F) != dir) {
    OCR1A = OCR1B = 0;
    _delay_ms(2);
    PORTC = (PORTC & 0xF0) | dir;
  }
  
  OCR1A = left;
  OCR1B = right;
}

// NOTE: The CPU clock frequency in slave mode must be at least 16 times
// higher than the SCL frequency! (see datasheet section 20.2.2)

int main()
{  
  InitMotor();
  InitPWM();
  InitSerial();
  sei(); // enable interrupts

  while(1) {
    if(cmd_ready) {
      switch(cmdbuf[0]) {
        case 0x0A:
          SetSpeed(cmdbuf[1], cmdbuf[2]);
          break;
        default:
          break;
      }
      cmd_ready = 0;
    }
    sleep_mode();
  }
}
