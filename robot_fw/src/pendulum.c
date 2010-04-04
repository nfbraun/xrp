#include "bitset.h"
#include "motor.h"
#include "serial.h"
#include "speedctrl.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SSI_CS0n 2
#define SSI_CS1n 3

int16_t get_angle(void)
{
  uint32_t dat;
  int16_t angle;
  
  twi_read(0x28, &dat, 4);
  
  angle = (int16_t) ((dat & 0x000000FF) << 4) | ((dat & 0x0000C000) >> 12) | ((dat & 0xC0000000) >> 30);
  
  return angle;
}

#if 0
#define A1 2137
#define A2 138462
#define A3 5
#define A4 8455
#endif

#define A1 3694
#define A2 169700
#define A3 11
#define A4 9342

int32_t get_accel(int32_t x_dot, int32_t phi_dot, int32_t x, int32_t phi)
{
    return A1*x_dot + A2*phi_dot + A3*x + A4*phi;
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

// Phi range for safety shutdown
#define PHI_MIN 1900 //2100
#define PHI_MAX 2700 //2450

// Angle for upright pendulum
// #define PHI_0 2270
#define PHI_0 2275

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

    int16_t phi;
    int16_t speed_0, speed_1;
    int8_t motor_0, motor_1;
    
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
        printf("%d\n", phi);
    
        if(enable) {
            // Read out wheel position
            speed_0 = get_speed(&wheel_pos_0);
            speed_1 = get_speed(&wheel_pos_1);
            
            target_speed += get_accel(target_speed >> 20,
                                      phi - old_phi,
                                      GET_WHEEL_POS(wheel_pos_0),
                                      phi - PHI_0);
                                      
            old_phi = phi;
            
            motor_0 = sp_ctrl_step(target_speed >> 20,
                                   speed_0,
                                   &error_int_0);
            motor_1 = sp_ctrl_step(target_speed >> 20,
                                   speed_1,
                                   &error_int_1);
            
            set_speed(motor_0, motor_1);
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
