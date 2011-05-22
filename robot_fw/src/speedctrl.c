#include "speedctrl.h"
#include "bitset.h"
#include <avr/io.h>

void init_wheel_pos(wheelpos_t* wp, int8_t ch)
{
    int16_t dat;
    
    _CLRBIT(PORTD, ch);
    dat = ssi_read();
    _SETBIT(PORTD, ch);
    
    wp->old_wheel_pos = (dat >> 6);
    // Note different sign convention for left and right encoder
    if(!(ch & 0x01))
        wp->old_wheel_pos = 1023 - wp->old_wheel_pos;
    
    wp->wheel_pos_0 = wp->old_wheel_pos;
    wp->wheel_pos_offset = 0;
    wp->ch = ch;
}

#define MOTOR_CLAMP 100
#define ERROR_INT_CLAMP 30000L

int16_t get_speed(wheelpos_t* wp)
{
    uint16_t dat;
    int16_t wheel_pos, speed;

    _CLRBIT(PORTD, wp->ch);
    dat = ssi_read();
    _SETBIT(PORTD, wp->ch);
    
    wheel_pos = (dat >> 6);
    // Note different sign convention for left and right encoder
    if(!(wp->ch & 0x01))
        wheel_pos = 1023 - wheel_pos;
    
    speed = wheel_pos - wp->old_wheel_pos;
    
    if(speed > 800) {
        speed -= 1024;
        wp->wheel_pos_offset--;
    } else if(speed < -800)  {
        speed += 1024;
        wp->wheel_pos_offset++;
    }
    
    wp->old_wheel_pos = wheel_pos;
    
    return speed;
}

int8_t sp_ctrl_step(int16_t set_speed, int16_t speed, int16_t* error_int)
{
    int16_t error = set_speed - speed;
    *error_int += error;
    
    if(*error_int > ERROR_INT_CLAMP) *error_int = ERROR_INT_CLAMP;
    else if(*error_int < -ERROR_INT_CLAMP) *error_int = -ERROR_INT_CLAMP;
    
    error += *error_int / 4;
    
    if(error > MOTOR_CLAMP) error = MOTOR_CLAMP;
    else if(error < -MOTOR_CLAMP) error = -MOTOR_CLAMP;
    
    return (int8_t) error;
}
