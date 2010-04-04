#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

#define DELAY 400 // us

int main()
{
    DDRC = 0x07;
    uint8_t dir;

    while(1) {
        if(PINB & 0x06) {
            dir = (PINB & 0x02) ? 0 : 2;
            _delay_us(DELAY);
            PORTC = 0x05 | dir;
            _delay_us(DELAY);
            PORTC = 0x04 | dir;
            _delay_us(DELAY);
        } else {
            PORTC = 0x00;
        }
    }
}
