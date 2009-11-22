#include "global.h"

#include <util/delay.h>
#include <avr/io.h>

void InitSerial(void)
{
    UCSRB = _UV(TXEN) | _UV(RXEN) | _UV(RXCIE);
    UCSRC = _UV(URSEL) | _UV(UCSZ1) | _UV(UCSZ0); // 8 bit, no parity, 1 stop
    UBRRL = 12;  // 38.4 kbps
}

void putchr(char c)
{
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
}

void putchrs(const char* c, int n)
{
    int i;
    for(i=0; i<n; ++i)
        putchr(*c++);
}

int main()
{
    uint8_t tmp[4];
    DDRB = 0x01;
    
    twi_init();
    InitSerial();

    while(1) {
        _delay_ms(10);
        twi_read(0x28, tmp, 4);
        putchrs(tmp, 4);
    }
}
