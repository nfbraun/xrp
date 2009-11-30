#include "global.h"
#include <avr/io.h>
#include <util/delay.h>

#define SSI_DI   6
#define SSI_CLK  7

void ssi_init(void)
{
    DDRD |= _UV(SSI_CLK);
    DDRD &= ~_UV(SSI_DI);
    PORTD |= _UV(SSI_CLK);
}

uint16_t ssi_read(void)
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
