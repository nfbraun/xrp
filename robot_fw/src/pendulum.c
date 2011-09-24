#include "bitset.h"
#include "motor.h"
#include "serial.h"
#include "speedctrl.h"
#include "serencode.h"
#include "adc.h"
#include "ir.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SSI_CS0n 3
#define SSI_CS1n 4

#define SWITCH_PIN (PINB & _UV(3))

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
// scale = 2**20
//#define A1 4570
//#define A2 90   // was: 192
//#define A3 5190
//#define A4 10

// scale = 2**16
//#define A1 285
//#define A2   5
//#define A3   0
//#define A4   1
//#define A5 324

// *** CURRENT ***
//// #define A1 299
//#define A1 3151 //4000
////#define A2 13
//#define A2 137
//#define A3 6097
//#define A4 1
//#define A5 -5772

#define A1 5048
#define A2 211
#define A3 9807
#define A4 21
#define A5 -7126


int32_t get_accel(int32_t phi_dot, int32_t phi, int32_t x_dot, int32_t x, int32_t x_dot_s)
{
    return (A1*phi_dot + A2*phi + A3*x_dot + A4*x + A5*x_dot_s);
}

int16_t old_phi;

volatile int8_t holdoff = 1;

wheelpos_t wheel_pos_0, wheel_pos_1;
int16_t error_int_0, error_int_1;
int32_t target_speed;
int32_t user_pos;

void reset_controller(void)
{
    init_wheel_pos(&wheel_pos_0, SSI_CS0n);
    init_wheel_pos(&wheel_pos_1, SSI_CS1n);
    
    old_phi = get_angle();
    
    error_int_0 = 0;
    error_int_1 = 0;
    
    target_speed = 0;
    
    user_pos = 0;
}

// Angle for upright pendulum
#define PHI_0 1500

// Phi range for safety shutdown
#define PHI_MIN (PHI_0 - 20550)
#define PHI_MAX (PHI_0 + 20550)

// Approx. zero level for angular rate sensor
#define OMEGA_0 324

// Zero level for accelerometer
#define GX_0 354

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
    
    int8_t user_speed = 0;
    int8_t bad_cnt = 0;
    
    int32_t omega_int = 0;
    
    twi_init();
    ssi_init();
    motor_init();
    serial_init();
    // scons_init();
    timer2_init();
    adc_init();
    ir_init();
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
        
        if(ir_state == IR_STATE_READY) {
            if(check_checksum(ir_data, 0xC0)) {
                bad_cnt = 0;
                user_speed = (ir_data[1] & 0x0F);
                if(ir_data[2] & 0x40)
                    user_speed = -user_speed;
            } else {
                if(bad_cnt < 10)
                    bad_cnt++;
            }
            if(bad_cnt > 8)
                target_speed = 0;
            ir_state = IR_STATE_ARMED;
        }
        
        phi = get_angle();
        omega = adc_read(1);
        
        if(enable) {
            // Read out wheel position
            speed_0 = get_speed(&wheel_pos_0);
            speed_1 = get_speed(&wheel_pos_1);
            
            target_speed += get_accel(//phi - old_phi,
                                      omega - OMEGA_0,
                                      //phi - PHI_0,
                                      omega_int >> 8,
                                      speed_0 - user_speed,
                                      GET_WHEEL_POS(wheel_pos_0) - user_pos,
                                      target_speed >> 16);
            
            user_pos += user_speed;
            
            motor_0 = sp_ctrl_step(target_speed >> 16,
                                   speed_0,
                                   &error_int_0);
            motor_1 = sp_ctrl_step(target_speed >> 16,
                                   speed_1,
                                   &error_int_1);
            
            set_speed(motor_0, motor_1);
            
            // se_puti32(target_speed);
            /* se_puti16(phi - old_phi);
            se_puti32(GET_WHEEL_POS(wheel_pos_0));
            se_puti16(phi - PHI_0); */
            
            // se_puti16(speed_0);
            // se_puti16(speed_1);
            
            /* se_puti16(325 - omega);
            
            se_puti16(idle_cnt); */
            
            // se_puti16(error_int_0);
            // se_puti16(error_int_1);
            
            // se_puti16(motor_0);
            // se_puti16(motor_1);
        } else {
            set_speed(0, 0);
        }
        
        se_start_frame(12);
        //se_puti32(user_pos);
        //se_puti16((int16_t)user_speed);
        //se_puti16(idle_cnt);
        // se_puti16(user_speed);
        //se_puti32(target_speed);
        //se_puti16(phi - old_phi);
        //se_puti32(GET_WHEEL_POS(wheel_pos_0));
        //se_puti16(phi - PHI_0);
        //se_puti16(omega - OMEGA_0);
        se_puti16(speed_0);
        se_puti16(user_speed);
        se_puti32(GET_WHEEL_POS(wheel_pos_0));
        se_puti32(user_pos);
        
        //se_puti16(adc_read(2) - GX_0);
        //se_puti16(adc_read(3));
        //se_puti16(adc_read(4));
        //se_puti32(omega_int);
        
        old_phi = phi;
        omega_int += (omega - OMEGA_0) << 8;
        omega_int += (7209*((int32_t)adc_read(2) - GX_0) - omega_int) >> 8;
        //omega_int = adc_read(2) - GX_0;
        
        if(phi > PHI_MIN && phi < PHI_MAX && SWITCH_PIN) {
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
