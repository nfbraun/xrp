#include "bitset.h"
#include "serial.h"
#include "motor.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define SPEED_CLOSED_LOOP
#ifdef SPEED_CLOSED_LOOP

#include "speedctrl.h"

#define SSI_CS0n 2
#define SSI_CS1n 3

volatile int8_t speed_0 = 0, speed_1 = 0;
volatile wheelpos_t wheel_pos_0, wheel_pos_1;
int16_t error_int_0 = 0, error_int_1 = 0;

ISR(TIMER2_COMP_vect)
{
    int8_t motor_0, motor_1;
    
    motor_0 = sp_ctrl_step(speed_0, get_speed(&wheel_pos_0), &error_int_0);
    motor_1 = sp_ctrl_step(speed_1, get_speed(&wheel_pos_1), &error_int_1);
            
    set_speed(motor_0, motor_1);
}

void timer2_init(void)
{
    OCR2 = F_CPU / (1024UL * 50UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
}

#else
int8_t speed_0, speed_1;
#endif

int main()
{
    motor_init();
    serial_init();
    //twi_init();
    ssi_init();
    //scons_init();

    sei(); // enable interrupts
    
    #ifdef SPEED_CLOSED_LOOP
    _SETBIT(DDRD, SSI_CS0n);
    _SETBIT(DDRD, SSI_CS1n);
    _SETBIT(PORTD, SSI_CS0n);
    _SETBIT(PORTD, SSI_CS1n);

    _SETBIT(DDRD, 4);
    
    init_wheel_pos(&wheel_pos_0, SSI_CS0n);
    init_wheel_pos(&wheel_pos_1, SSI_CS1n);

    timer2_init();
    #endif

    speed_0 = 0;
    speed_1 = 0;

    while(1) {
        if(cmd_ready) {
            switch(cmdbuf[0]) {
            case 0x0A:
                speed_0 = ((int8_t) cmdbuf[1]);
                speed_1 = ((int8_t) cmdbuf[2]);
                set_speed(speed_0, speed_1);
                break;
            default:
                break;
            }
            cmd_ready = 0;
        }
        sleep_mode();
    }
}
