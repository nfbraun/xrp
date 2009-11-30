#include "global.h"

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "twi-slave.h"

#define PWM_ADDRESS 42

void InitPWM()
{
  // Enable output
  DDRB |= 0x02;
  // Set output compare
  OCR1A = 0;
  // Set TOP
  ICR1 = 100;
  // Set WGM[13:10] to 1000 (phase and frequency correct PWM mode, TOP = ICR1)
  TCCR1A &= ~0x03;
  TCCR1B &= ~0x08;
  TCCR1B |=  0x10;
  // Set COM1A[1:0] to 10 (set/clear OC1A on compare match)
  TCCR1A |=  0x80;
  TCCR1A &= ~0x40;
  // Set clock source to CLKIO, no prescaler (CS1[2:0] = 001)
  TCCR1B &= ~0x06;
  TCCR1B |= 0x01;
}

void SetPWM(uint16_t val)
{
  if(val > 100) val = 100; // clip value
  OCR1A = val;
}

// NOTE: The CPU clock frequency in slave mode must be at least 16 times
// higher than the SCL frequency! (see datasheet section 20.2.2)

int main()
{
  uint8_t messageBuf[TWI_BUFFER_SIZE];
  
  InitPWM();

  TWI_Slave_Initialise((PWM_ADDRESS << TWI_ADR_BITS) | (0 << TWI_GEN_BIT));
  TWI_Start_Transceiver();
  sei(); // enable interrupts
  
  while(1) {
    TWI_Get_Data_From_Transceiver(messageBuf, 1);
    SetPWM(messageBuf[0]);
    TWI_Start_Transceiver();
  }
}
