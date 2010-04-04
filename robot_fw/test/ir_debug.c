#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "bitset.h"

int16_t recv_bit(int16_t timeout)
{
    int8_t shift = 0;
    int16_t count = 0;

    while(shift == 0 && count < timeout) {
        shift = (shift << 1) | (PINB & 0x01);
        ++count;
        _delay_us(5);
    }
    
    if(count >= timeout) return -1;
    
    count = 0;
    while(shift != 0) {
        shift = (shift << 1) | (PINB & 0x01);
        _delay_us(5);
        ++count;
    }
    
    return count;
}

int recv_sequence(int8_t *buf)
{
    int16_t count = 0;
    int8_t bitcount = 0;

    // Wait for start
    while(count < 400) {
        count = recv_bit(10000);
    }
        
    // Receive bits    
    while(1) {
        count = recv_bit(400);
        if(count < 0) {
            break;
        } else if(count > 100) {
            *buf++ = 0;
            bitcount++;
        } else  {
            *buf++ = 1;
            bitcount++;
        }
    }
    
    return bitcount;
}

int main()
{
    int8_t buf[64];
    int8_t i, n;

    DDRC = 0x03;
    serial_init();
    sei();
    
    while(1) {
        n = recv_sequence(buf);
        for(i=0; i<n; ++i) {
            if((i % 8) == 0)
                s_putchr(' ');
            
            if(buf[i] == 0)
                s_putchr('0');
            else
                s_putchr('1');
        }
        s_putchr('\n');
    }
}


