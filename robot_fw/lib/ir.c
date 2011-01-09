#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include "bitset.h"
#include <ir.h>

volatile uint8_t ir_state;
volatile uint8_t ir_data[IR_DATA_SIZE];

int8_t check_checksum(volatile uint8_t *buf, uint8_t ch)
{
    return (((buf[0] + buf[1] + buf[2] + 0x0F) & 0x3F) | ch) == buf[3];
}

ISR(INT0_vect)
{
    static uint8_t mask;
    
    // Falling edge
    if(!(PIND & 0x04)) {
        if(ir_state == IR_STATE_ARMED) {
            // Start bit
            if(TCNT0 >= 24) {
                ir_state = 0;
                mask = 0;
            }
        } else if(ir_state >= 0 && ir_state < IR_DATA_SIZE) {
            if(!mask) {
                mask = 0x80;
                ir_data[ir_state] = 0;
            }
            // logical one
            if(TCNT0 < 4)
                ir_data[ir_state] |= mask;
            
            mask >>= 1;
            
            if(!mask)
                ir_state++;
        }
    }
    
    // Reset timer
    TCNT0 = 0;
}

void ir_init(void)
{
#if defined(__AVR_ATmega8__)
    // Setup INT0
    MCUCR |= _UV(ISC00);
    MCUCR &= ~_UV(ISC01);
    GICR |= _UV(INT0);
    
    // Pull-up for interrupt port
    PORTD |= _UV(PD2);
    
    // Setup timer0
    TCCR0 = _UV(CS02) | _UV(CS00);
#elif defined(__AVR_ATmega328P__)
    // Setup INT0
    EICRA = _UV(ISC00);
    EIMSK |= _UV(INT0);
    
    // Setup timer2
    TCCR2B = _UV(CS22) | _UV(CS21) | _UV(CS20);
#endif
    
    ir_state = IR_STATE_ARMED;
}

