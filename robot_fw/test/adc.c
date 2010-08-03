#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "bitset.h"
#include <inttypes.h>
#include "serencode.h"

volatile uint8_t ch;

void adc_start_single(uint8_t c);

ISR(ADC_vect)
{
    _CLRBIT(ADCSRA, ADIE);
    
    uint16_t val = ADCW;
    
    if(ch == 0)
        se_start_frame(6);
    
    se_puti16(val);
    
    ch++;
    if(ch < 3) {
        adc_start_single(ch);
    }
}

ISR(TIMER2_COMP_vect)
{
    ch = 0;
    adc_start_single(0);
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

void timer2_init(void)
{
    OCR2 = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
}

int main()
{
    serial_init();
    timer2_init();
    adc_init();
    sei();
    
    while(1) {
        sleep_mode();
    }
}
