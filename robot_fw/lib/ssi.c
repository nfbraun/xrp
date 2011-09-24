#include "bitset.h"
#include <avr/io.h>
#include <util/delay.h>

#define SSI_PORT  PORTD
#define SSI_PIN   PIND
#define SSI_DDR   DDRD

#define SSI_DI    6
#define SSI_CLK   7

void ssi_init(void)
{
    _SETBIT(SSI_DDR, SSI_CLK);
    _CLRBIT(SSI_DDR, SSI_DI);
    _SETBIT(SSI_PORT, SSI_CLK);
}

uint16_t ssi_read(void)
{
    uint8_t i;
    uint16_t data;
    
    _delay_us(10);
    
    _CLRBIT(SSI_PORT, SSI_CLK);
    _delay_us(10);
    _SETBIT(SSI_PORT, SSI_CLK);
    _delay_us(10);
    
    for(i=0; i<16; ++i) {
        _CLRBIT(SSI_PORT, SSI_CLK);
        _delay_us(10);
        data <<= 1;
        data |= ((SSI_PIN & _UV(SSI_DI)) >> SSI_DI);
        _SETBIT(SSI_PORT, SSI_CLK);
        _delay_us(10);
    }
    
    return data;
}
