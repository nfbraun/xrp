#include "bitset.h"
#include "serial.h"
#include "motor.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>

int16_t get_angle(void)
{
  uint32_t dat;
  int16_t angle;
  
  twi_read(0x28, &dat, 4);
  
  angle = (int16_t) ((dat & 0x000000FF) << 4) | ((dat & 0x0000C000) >> 12) | ((dat & 0xC0000000) >> 30);
  
  return angle;
}

volatile int8_t holdoff = 1;
ISR(TIMER2_COMP_vect)
{
    holdoff = 0;
}

void timer2_init(void)
{
    OCR2 = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
}


int main()
{
    int16_t t = 0;
    int16_t phi;
    
    serial_init();
    twi_init();
    scons_init();
    timer2_init();
    sei(); // enable interrupts

    while(1) {
        while(holdoff);
        holdoff = 1;
    
        phi = get_angle();
        printf("%d %d\n", t++, phi);
    }
}
