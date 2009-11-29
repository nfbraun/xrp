#define F_CPU 1000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

#define DELAY 400 // us

int main()
{
    DDRD = 0x07;
    uint8_t dir;

    while(1) {
        if(PINB & 0x06) {
            dir = (PINB & 0x02) ? 0 : 1;
            _delay_us(DELAY);
            PORTD = 0x02 | dir;
            _delay_us(DELAY);
            PORTD = 0x00 | dir;
            _delay_us(DELAY);
        } else {
            PORTD = 0x04;
        }
    }
}
