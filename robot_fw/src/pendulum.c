#include "bitset.h"
#include "motor.h"
#include "serial.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SSI_CS0n 2
#define SSI_CS1n 3

int16_t get_wheel_pos(int8_t ch)
{
    uint16_t dat;

    _CLRBIT(PORTD, ch);
    dat = ssi_read();
    _SETBIT(PORTD, ch);
       
    return (dat >> 6);
}

int16_t get_angle(void)
{
  uint32_t dat;
  int16_t angle;
  
  twi_read(0x28, &dat, 4);
  
  angle = (int16_t) ((dat & 0x000000FF) << 4) | ((dat & 0x0000C000) >> 12) | ((dat & 0xC0000000) >> 30);
  
  return angle;
}

#define MOTOR_CLAMP 80
#define ERROR_INT_CLAMP 5000000L

int32_t error_int;

int8_t motor_control(int16_t phi, int16_t phi_dot, int16_t pos, int16_t v)
{
    int32_t error;
    int32_t motor;

    error = -500L * phi - 35000L * phi_dot + 2L * pos + 400L * v;
    error_int += error;
    
    if(error_int > ERROR_INT_CLAMP) error_int = ERROR_INT_CLAMP;
    else if(error_int < -ERROR_INT_CLAMP) error_int = -ERROR_INT_CLAMP;

    #if 0
    printf("%d %d %d %d %ld %ld\n", phi, phi_dot, pos, v, error, error_int);
    #endif

    motor = (error + 3 * error_int) / 100000L;
    
    if(motor > MOTOR_CLAMP) motor = MOTOR_CLAMP;
    if(motor < -MOTOR_CLAMP) motor = -MOTOR_CLAMP;
    
    return (int8_t) motor;
}

int16_t old_wheel_pos, wheel_pos_0, wheel_pos_offset;
int16_t old_phi;

volatile int8_t holdoff = 1;

void reset_controller(void)
{
    old_wheel_pos = get_wheel_pos(SSI_CS0n);
    wheel_pos_0 = old_wheel_pos;
    wheel_pos_offset = 0;
    
    old_phi = get_angle();
    
    error_int = 0;
}

// Phi range for safety shutdown
#define PHI_MIN 1900 //2100
#define PHI_MAX 2700 //2450

// Angle for upright pendulum
// #define PHI_0 2270
#define PHI_0 2240

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

int main()
{
    int8_t enable = 0;
    int8_t enable_count = 0;

    int16_t phi, wheel_pos;
    int8_t motor;
    
    twi_init();
    ssi_init();
    motor_init();
    serial_init();
    scons_init();
    timer2_init();
    sei(); // enable interrupts
    
    // chip select for SSI
    _SETBIT(DDRD, SSI_CS0n);
    _SETBIT(DDRD, SSI_CS1n);
    _SETBIT(PORTD, SSI_CS0n);
    _SETBIT(PORTD, SSI_CS1n);
    
    // safety shutdown indicator LED
    _SETBIT(DDRB, 6);
    
    while(1) {
        // Rate limit
        while(holdoff);
        holdoff = 1;
    
        phi = get_angle();
    
        if(enable) {
            // Read out wheel position
            wheel_pos = get_wheel_pos(SSI_CS0n);
            if((wheel_pos - old_wheel_pos + wheel_pos_offset) > 800) {
                wheel_pos_offset -= 1024;
            } else if((wheel_pos - old_wheel_pos + wheel_pos_offset) < -800)  {
                wheel_pos_offset += 1024;
            }
            wheel_pos += wheel_pos_offset;
            
            // Note different sign convention for left and right encoder
            motor = motor_control(phi - PHI_0,
                                  phi - old_phi,
                                  wheel_pos - wheel_pos_0,
                                  wheel_pos - old_wheel_pos);
                                    
            set_speed(motor, motor);
            
            old_wheel_pos = wheel_pos;
            old_phi = phi;
        } else {
            set_speed(0, 0);
        }
        
        if(phi > PHI_MIN && phi < PHI_MAX) {
            enable_count++;
        } else {
            enable = 0;
            enable_count = 0;
            _CLRBIT(PORTB, 6);
        }
        
        if(!enable && enable_count >= 100) {
            enable = 1;
            reset_controller();
            _SETBIT(PORTB, 6);
        }
    }
}
