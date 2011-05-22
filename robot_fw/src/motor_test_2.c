#include "bitset.h"
#include "serial.h"
#include "motor.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "speedctrl.h"

#define SWITCH_PIN (PINB & _UV(3))

#define SSI_CS0n 3
#define SSI_CS1n 4

int8_t sin_lut[] = { 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 16, 17, 18, 18, 19, 20, 21, 21, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30 };

int8_t sin_speed(int16_t t)
{
    if(t <= 0) return 0;
    else if(t <= 50) return sin_lut[t];
    else if(t <= 100) return sin_lut[100-t];
    else if(t <= 150) return -sin_lut[t-100];
    else if(t <= 200) return -sin_lut[200-t];
    else return 0;
}

int8_t square_speed(int16_t t)
{
    if(t < 0) return 0;
    else if(t < 100) return 10;
    else if(t < 200) return -10;
    else return 0;
}

volatile int8_t holdoff = 1;

ISR(TIMER2_COMP_vect)
{
    holdoff = 0;
}

void timer2_init(void)
{
    OCR2 = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
}

int8_t set_speed_0 = 0, set_speed_1 = 0;
wheelpos_t wheel_pos_0, wheel_pos_1;
int16_t error_int_0 = 0, error_int_1 = 0;
int16_t cur_t = 0;

void reset_controller(void)
{
    init_wheel_pos(&wheel_pos_0, SSI_CS0n);
    init_wheel_pos(&wheel_pos_1, SSI_CS1n);
    
    error_int_0 = 0;
    error_int_1 = 0;
    
    set_speed_0 = 0;
    set_speed_1 = 0;
    cur_t = -10;
}

int main()
{
    int8_t enable = 0;
    int8_t enable_count = 0;
    
    int8_t power_0, power_1;
    int16_t speed_0, speed_1;
    
    motor_init();
    serial_init();
    //twi_init();
    ssi_init();
    //scons_init();

    sei(); // enable interrupts
    
    _SETBIT(DDRD, SSI_CS0n);
    _SETBIT(DDRD, SSI_CS1n);
    _SETBIT(PORTD, SSI_CS0n);
    _SETBIT(PORTD, SSI_CS1n);

    _SETBIT(DDRD, 4);
    
    init_wheel_pos(&wheel_pos_0, SSI_CS0n);
    init_wheel_pos(&wheel_pos_1, SSI_CS1n);

    timer2_init();

    set_speed_0 = 0;
    set_speed_1 = 0;

    while(1) {
        while(holdoff);
        holdoff = 1;
        
        if(enable) {
            speed_0 = get_speed(&wheel_pos_0);
            speed_1 = get_speed(&wheel_pos_1);
            
            power_0 = sp_ctrl_step(set_speed_0, speed_0, &error_int_0);
            power_1 = sp_ctrl_step(set_speed_1, speed_1, &error_int_1);
            
            set_speed(power_0, power_1);
            
            set_speed_0 = set_speed_1 = sin_speed(2*cur_t);
            
            cur_t++;
            if(cur_t >= 100) {
                cur_t = 0;
            }
            
        } else {
            set_speed(0,0);
        }
        
        se_start_frame(20);
        se_puti16(set_speed_0);
        se_puti32(error_int_0);
        se_puti16(power_0);
        se_puti16(speed_0);
        se_puti16(set_speed_1);
        se_puti32(error_int_1);
        se_puti16(power_1);
        se_puti16(speed_1);
        
        if(SWITCH_PIN) {
            enable_count++;
        } else {
            enable = 0;
            enable_count = 0;
        }
        
        if(!enable && enable_count >= 100) {
            enable = 1;
            reset_controller();
        }
    }
}
