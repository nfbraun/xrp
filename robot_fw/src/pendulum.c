#include "bitset.h"
#include "motor.h"
#include "serial.h"
#include "speedctrl.h"
#include "serencode.h"
#include "adc.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SSI_CS0n 3
#define SSI_CS1n 4

int16_t poly(int16_t _x)
{
    if(_x < 156 || _x > 623)
        return 30000;
    int32_t x = _x;
    return (-78355 + 508*x - x*x + (x*x)/1416*x);
}

int16_t get_angle(void)
{
  return poly(adc_read(0));
}

#if 0
#define A1 4202
#define A2 90   // was: 176
#define A3 850
#define A4 0
#endif

// These values work somewhat, but I do not know why (or how to improve them).
#define A1 4570
#define A2 90   // was: 192
#define A3 5190
#define A4 10

int32_t get_accel(int32_t x_dot, int32_t phi_dot, int32_t x, int32_t phi)
{
    return (A1*phi_dot + A2*phi + A3*x_dot + A4*x);
}

int16_t old_phi;

volatile int8_t holdoff = 1;

wheelpos_t wheel_pos_0, wheel_pos_1;
int16_t error_int_0, error_int_1;
int32_t target_speed;

void reset_controller(void)
{
    init_wheel_pos(&wheel_pos_0, SSI_CS0n);
    init_wheel_pos(&wheel_pos_1, SSI_CS1n);
    
    old_phi = get_angle();
    
    error_int_0 = 0;
    error_int_1 = 0;
    
    target_speed = 0;
}

// Angle for upright pendulum
#define PHI_0 2055

// Phi range for safety shutdown
#define PHI_MIN (PHI_0 - 20550)
#define PHI_MAX (PHI_0 + 20550)

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

    int16_t phi, omega;
    int16_t speed_0, speed_1;
    int8_t motor_0, motor_1;
    
    twi_init();
    ssi_init();
    motor_init();
    serial_init();
    // scons_init();
    timer2_init();
    adc_init();
    sei(); // enable interrupts
    
    // chip select for SSI
    _SETBIT(DDRD, SSI_CS0n);
    _SETBIT(DDRD, SSI_CS1n);
    _SETBIT(PORTD, SSI_CS0n);
    _SETBIT(PORTD, SSI_CS1n);
    
    // safety shutdown indicator LED
    _SETBIT(DDRB, 0);
    
    while(1) {
        // Rate limit
        int16_t idle_cnt = 0;
        while(holdoff) { idle_cnt++; }
        holdoff = 1;
        
        phi = get_angle();
        omega = adc_read(1);
        
        if(enable) {
            // Read out wheel position
            speed_0 = get_speed(&wheel_pos_0);
            speed_1 = get_speed(&wheel_pos_1);
            
            target_speed += get_accel(target_speed >> 20,
                                      //325 - omega,
                                      phi - old_phi,
                                      GET_WHEEL_POS(wheel_pos_0),
                                      phi - PHI_0);
            
            motor_0 = sp_ctrl_step(target_speed >> 20,
                                   speed_0,
                                   &error_int_0);
            motor_1 = sp_ctrl_step(target_speed >> 20,
                                   speed_1,
                                   &error_int_1);
            
            set_speed(motor_0, motor_1);
            
            // WARNING: The amount of data transmitted here is on the border
            // of what is acceptable.
            se_start_frame(24);
            
            se_puti32(target_speed);
            se_puti16(phi - old_phi);
            se_puti32(GET_WHEEL_POS(wheel_pos_0));
            se_puti16(phi - PHI_0);
            
            se_puti16(speed_0);
            se_puti16(speed_1);
            
            se_puti16(325 - omega);
            
            se_puti16(idle_cnt);
            
            se_puti16(error_int_0);
            se_puti16(error_int_1);
            
            old_phi = phi;
        } else {
            set_speed(0, 0);
        }
        
        if(phi > PHI_MIN && phi < PHI_MAX) {
            enable_count++;
        } else {
            enable = 0;
            enable_count = 0;
            _CLRBIT(PORTB, 0);
        }
        
        if(!enable && enable_count >= 100) {
            enable = 1;
            reset_controller();
            _SETBIT(PORTB, 0);
        }
    }
}
