#include "bitset.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define RESET_ON_ESC

volatile uint8_t cmdbuf[8];
volatile int8_t cmd_ready;
uint8_t nread;

#if defined(__AVR_ATmega8__)
ISR(USART_RXC_vect)
#elif defined(__AVR_ATmega328P__)
ISR(USART_RX_vect)
#endif
{
    uint8_t c;
    
    #if defined(__AVR_ATmega8__)
    c = UDR;
    if(bit_is_clear(UCSRA, FE)) {
    #elif defined(__AVR_ATmega328P__)
    c = UDR0;
    if(bit_is_clear(UCSR0A, FE0)) {
    #endif
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
#if defined(__AVR_ATmega8__)
    UCSRB = _UV(TXEN) | _UV(RXEN) | _UV(RXCIE);
    UCSRC = _UV(URSEL) | _UV(UCSZ1) | _UV(UCSZ0); // 8 bit, no parity, 1 stop
    UBRRL = 12;  // 38.4 kbps
#elif defined(__AVR_ATmega328P__)
    UCSR0B = _UV(TXEN0) | _UV(RXEN0) | _UV(RXCIE0);
    UCSR0C = _UV(UCSZ01) | _UV(UCSZ00); // 8 bit, no parity, 1 stop
    UBRR0 = 12;  // 38.4 kbps
#endif
}

void s_putchr(char c)
{
#if defined(__AVR_ATmega8__)
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
#elif defined(__AVR_ATmega328P__)
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
#endif
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
