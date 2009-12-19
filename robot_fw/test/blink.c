#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

int main()
{
    /* serial_init();
    sei(); */
    
    DDRB = 0x01;
    PORTB |= 0x01;
    _delay_ms(1000);
    PORTB &= ~(0x01);
}
