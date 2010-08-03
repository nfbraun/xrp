#include "bitset.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define RESET_ON_ESC

volatile uint8_t cmdbuf[8];
volatile int8_t cmd_ready;
uint8_t nread;

ISR(USART_RXC_vect)
{
    uint8_t c;

    c = UDR;
    if(bit_is_clear(UCSRA, FE)) {
        cmdbuf[nread++] = c;
        
        #ifdef RESET_ON_ESC
        if(nread == 1 && cmdbuf[0] == 0x1B) {
            cli(); // Disable interrupts
            wdt_enable(WDTO_15MS); // Setup watchdog
            while(1);  // Enter endless loop waiting for watchdog
        }
        #endif
        
        if(nread > (cmdbuf[0] & 0x07)) {
            cmd_ready = 1;
            nread = 0;
        }
    }
}

void serial_init(void)
{
    UCSRB = _UV(TXEN) | _UV(RXEN) | _UV(RXCIE);
    UCSRC = _UV(URSEL) | _UV(UCSZ1) | _UV(UCSZ0); // 8 bit, no parity, 1 stop
    UBRRL = 12;  // 38.4 kbps
}

void s_putchr(char c)
{
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
}

void s_puti16(uint16_t x)
{
    s_putchr(x & 0x00FF);
    s_putchr((x & 0xFF00) >> 8);
}

void s_puti32(uint32_t x)
{
    s_putchr((x & 0x000000FF));
    s_putchr((x & 0x0000FF00) >>  8);
    s_putchr((x & 0x00FF0000) >> 16);
    s_putchr((x & 0xFF000000) >> 24);
}

void s_putdata(const char* c, int n)
{
    int i;
    for(i=0; i<n; ++i)
        s_putchr(*c++);
}
