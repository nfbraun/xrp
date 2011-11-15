#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef _UV
#define _UV(x) (1 << (x))
#endif

#define _setbit(x, y) ((x) |= _UV(y))
#define _clrbit(x, y) ((x) &= ~_UV(y))

#define DRVPORT PORTB
#define DRVEN  0

#define XACT 0
#define YACT 1
#define ZACT 2

#define XPORT PORTB
#define XDIR 1
#define XCLK 2
#define XSTDBY 3

#define YPORT PORTC
#define YDIR 0
#define YCLK 1
#define YSTDBY 2

#define ZPORT PORTC
#define ZDIR 3
#define ZCLK 4
#define ZSTDBY 5

volatile uint8_t cmdbuf;
volatile int8_t cmd_ready;

ISR(USART_RXC_vect)
{
    uint8_t c;

    c = UDR;
    if(bit_is_clear(UCSRA, FE)) {
        cmdbuf = c;
        cmd_ready = 1;
    }
}

void InitSerial(void)
{
    UCSRB = _UV(TXEN) | _UV(RXEN) | _UV(RXCIE);
    UCSRC = _UV(URSEL) | _UV(UCSZ1) | _UV(UCSZ0); // 8 bit, no parity, 1 stop
    UBRRL = 12;  // 38.4 kbps
}

void InitIO(void)
{
    DDRB = 0x0F;
    DDRC = 0x3F;
}

int main()
{
    uint8_t active = 0;

    InitIO();
    InitSerial();
    sei();  // Enable interrupts

    while(1) {
        if(cmd_ready) {
            switch(cmdbuf) {
                case 'd':
                    _clrbit(DRVPORT, DRVEN);
                    break;
                case 'D':
                    _setbit(DRVPORT, DRVEN);
                    break;
                
                case 'x':
                    _clrbit(XPORT, XDIR);
                    _clrbit(XPORT, XSTDBY);
                    _setbit(active, XACT);
                    break;
                case 'X':
                    _setbit(XPORT, XDIR);
                    _clrbit(XPORT, XSTDBY);
                    _setbit(active, XACT);
                    break;
                    
                case 'y':
                    _clrbit(YPORT, YDIR);
                    _clrbit(YPORT, YSTDBY);
                    _setbit(active, YACT);
                    break;
                case 'Y':
                    _setbit(YPORT, YDIR);
                    _clrbit(YPORT, YSTDBY);
                    _setbit(active, YACT);
                    break;
                    
                case 'z':
                    _clrbit(ZPORT, ZDIR);
                    _clrbit(ZPORT, ZSTDBY);
                    _setbit(active, ZACT);
                    break;
                case 'Z':
                    _setbit(ZPORT, ZDIR);
                    _clrbit(ZPORT, ZSTDBY);
                    _setbit(active, ZACT);
                    break;
                    
                case '0':
                    active = 0;
                    _setbit(XPORT, XSTDBY);
                    _setbit(YPORT, YSTDBY);
                    _setbit(ZPORT, ZSTDBY);
                    break;
            }
            cmd_ready = 0;
        }
        
        if(active & _UV(XACT))  _setbit(XPORT, XCLK);
        if(active & _UV(YACT))  _setbit(YPORT, YCLK);
        if(active & _UV(ZACT))  _setbit(ZPORT, ZCLK);
        _delay_us(500);
        
        if(active & _UV(XACT))  _clrbit(XPORT, XCLK);
        if(active & _UV(YACT))  _clrbit(YPORT, YCLK);
        if(active & _UV(ZACT))  _clrbit(ZPORT, ZCLK);
        _delay_us(500);
    }
}
