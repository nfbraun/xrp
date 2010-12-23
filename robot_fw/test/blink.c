#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

int main()
{
    /* cli();
    wdt_reset();
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0x00; */
    
    serial_init();
    sei();
    
    DDRB = 0x03;
    
    PORTB |= 0x02;
    _delay_ms(500);
    PORTB &= ~0x02;
    
    while(1);
}
