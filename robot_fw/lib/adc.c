#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "bitset.h"
#include <inttypes.h>

volatile int16_t adc_value;

ISR(ADC_vect)
{
    _CLRBIT(ADCSRA, ADIE);
    adc_value = ADCW;
}

void adc_start_single(uint8_t c)
{
    ADMUX = _UV(REFS0) | (c & 0x07);
    _SETBIT(ADCSRA, ADSC);
    _SETBIT(ADCSRA, ADIE);
}

void adc_init(void)
{
    ADCSRA = _UV(ADEN) | _UV(ADPS2) | _UV(ADPS1);
    ADMUX = _UV(REFS0);
}

int16_t adc_read(uint8_t c)
{
    adc_value = -1;
    
    adc_start_single(c);
    
    while(adc_value < 0)
        sleep_mode();
    
    return adc_value;
}
