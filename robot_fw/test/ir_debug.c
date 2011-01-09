#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "bitset.h"

int16_t recv_bit(int16_t timeout)
{
    int16_t count = 0;
    
    while((PINC & 0x01) && count < timeout) {
        ++count;
        _delay_us(5);
    }
    
    if(count >= timeout) return -1;
    
    count = 0;
    while(!(PINC & 0x01)) {
        _delay_us(5);
        ++count;
    }
    
    return count;
}

int recv_sequence(uint8_t *buf)
{
    int16_t count = 0;
    uint8_t mask = 0;
    int8_t bitcount = 0;
    
    buf--;

    // Wait for start
    while(count < 400) {
        count = recv_bit(10000);
    }
    
    // Receive bits
    while(1) {
        count = recv_bit(400);
        if(count < 0) {
            break;
        }
        
        if(mask == 0) {
            buf++;
            *buf = 0x00;
            mask = 0x80;
        }
        
        if(count < 100) {
            *buf |= mask;
        }
        
        mask >>= 1;
        bitcount ++;
    }
    
    return bitcount;
}

int8_t check_checksum(uint8_t *buf, uint8_t ch)
{
    return (((buf[0] + buf[1] + buf[2] + 0x0F) & 0x3F) | ch) == buf[3];
}

int main()
{
    uint8_t buf[64];
    int8_t i, n;

    serial_init();
    scons_init();
    sei();
    
    while(1) {
        n = recv_sequence(buf);
        /* for(i=0; i<n; ++i) {
            if((i % 8) == 0)
                s_putchr(' ');
            
            if(buf[i] == 0)
                s_putchr('0');
            else
                s_putchr('1');
        }
        s_putchr('\r');
        s_putchr('\n'); */
        printf("%02hhX %02hhX %02hhX %02hhX", buf[0], buf[1], buf[2], buf[3]);
        if(check_checksum(buf, 0xC0))
            printf(" [OK]\r\n");
        else
            printf(" [FAIL]\r\n");
    }
}


