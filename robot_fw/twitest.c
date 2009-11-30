#include "global.h"

#include <util/delay.h>
#include <avr/io.h>

#define SSI_CS0n 2
#define SSI_CS1n 3
#define SSI_DI   6
#define SSI_CLK  7

void InitSerial(void)
{
    UCSRB = _UV(TXEN) | _UV(RXEN) | _UV(RXCIE);
    UCSRC = _UV(URSEL) | _UV(UCSZ1) | _UV(UCSZ0); // 8 bit, no parity, 1 stop
    UBRRL = 12;  // 38.4 kbps
}

void InitSSI(void)
{
    DDRD |= (_UV(SSI_CS0n) | _UV(SSI_CS1n) | _UV(SSI_CLK));
    DDRD &= ~_UV(SSI_DI);
    PORTD |= (_UV(SSI_CS0n) | _UV(SSI_CS1n) | _UV(SSI_CLK));
}

uint16_t ReadSSI(void)
{
    uint8_t i;
    uint16_t data;
    
    _delay_us(10);
    
    PORTD &= ~_UV(SSI_CLK);
    _delay_us(10);
    PORTD |= _UV(SSI_CLK);
    _delay_us(10);
    
    for(i=0; i<16; ++i) {
        PORTD &= ~_UV(SSI_CLK);
        _delay_us(10);
        data <<= 1;
        data |= ((PIND & _UV(SSI_DI)) >> SSI_DI);
        PORTD |= _UV(SSI_CLK);
        _delay_us(10);
    }
    
    return data;
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
    /* uint8_t tmp[4];
    DDRB = 0x01;
    
    twi_init();
    InitSerial();

    while(1) {
        _delay_ms(10);
        twi_read(0x28, tmp, 4);
        putchrs(tmp, 4);
    } */
    
    uint16_t tmp;
    InitSSI();
    InitSerial();
    
    tmp = 0;
    while(1) {
        _delay_ms(10);
        PORTD &= ~_UV(SSI_CS0n);
        tmp = ReadSSI();
        PORTD |= _UV(SSI_CS0n);
        putchrs(&tmp, 2);
    }
}
