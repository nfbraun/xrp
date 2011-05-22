#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "bitset.h"
#include <inttypes.h>
#include <serencode.h>

#define N_CH 5

volatile uint8_t ch;

void adc_start_single(uint8_t c);

ISR(ADC_vect)
{
    _CLRBIT(ADCSRA, ADIE);
    
    uint16_t val = ADCW;
    
    if(ch == 0)
        se_start_frame(2 * N_CH);
    
    se_puti16(val);
    
    ch++;
    if(ch < N_CH) {
        adc_start_single(ch);
    }
}

#if defined(__AVR_ATmega8__)
ISR(TIMER2_COMP_vect)
#elif defined(__AVR_ATmega328P__)
ISR(TIMER2_COMPA_vect)
#endif
{
    _SETBIT(PORTB, 0);
    ch = 0;
    adc_start_single(0);
    _CLRBIT(PORTB, 0);
}

void adc_start_single(uint8_t c)
{
    // ADC reference voltage is AVcc
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
#if defined(__AVR_ATmega8__)
    OCR2 = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
#elif defined(__AVR_ATmega328P__)
    OCR2A = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2A = _UV(WGM21);
    TCCR2B = _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK2, OCIE2A);
#endif
}

int main()
{
    serial_init();
    timer2_init();
    adc_init();
    sei();
    
    DDRB = 0x01;
    
    while(1) {
        sleep_mode();
    }
}
