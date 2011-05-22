#include "bitset.h"
#include "serial.h"
#include "motor.h"
#include <serencode.h>
#include <ir.h>
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "speedctrl.h"

#define abs(x) ((x) < 0 ? (-(x)) : (x))

#define SSI_CS0n 3
#define SSI_CS1n 4

int16_t get_angle(int8_t ch)
{
    uint16_t dat;

    _CLRBIT(PORTD, ch);
    dat = ssi_read();
    _SETBIT(PORTD, ch);
       
    return (dat >> 6);
}

volatile int8_t holdoff = 1;

ISR(TIMER2_COMP_vect)
{
    holdoff = 0;
}

void cmd_multi_angle(uint8_t n)
{
    uint8_t i;
    // uint8_t val[4];
    uint16_t val;
  
    for(i=0; i<n; ++i) {
        _CLRBIT(PORTD, SSI_CS0n);
        val = ssi_read();
        _SETBIT(PORTD, SSI_CS0n);
        //twi_read(0x28, val, 4);
        
        _delay_ms(10);
        s_puti16(val);
        
        //s_putdata(val, sizeof(val));
    }
}

void timer2_init(void)
{
    OCR2 = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
}

/* int main()
{
    serial_init();
    scons_init();
    ir_init();
    sei();
    
    while(1) {
        sleep_mode();
        if(ir_state == IR_STATE_READY) {
            printf("%02hhX %02hhX %02hhX %02hhX", ir_data[0], ir_data[1], ir_data[2], ir_data[3]);
            if(check_checksum(ir_data, 0xC0))
                printf(" [OK]\r\n");
            else
                printf(" [FAIL]\r\n");
            ir_state = IR_STATE_ARMED;
        }
    }
} */

wheelpos_t wheel_pos_0, wheel_pos_1;
int16_t error_int_0, error_int_1;
int16_t target_speed;

void reset_controller(void)
{
    init_wheel_pos(&wheel_pos_0, SSI_CS0n);
    init_wheel_pos(&wheel_pos_1, SSI_CS1n);
    
    error_int_0 = 0;
    error_int_1 = 0;
    
    target_speed = 0;
}

int main()
{
    int16_t speed_0, speed_1;
    int8_t motor_0, motor_1;
    
    int8_t bad_cnt = 0;
    
    motor_init();
    serial_init();
    //twi_init();
    ssi_init();
    ir_init();
    scons_init();
    
    _SETBIT(DDRD, SSI_CS0n);
    _SETBIT(DDRD, SSI_CS1n);
    _SETBIT(PORTD, SSI_CS0n);
    _SETBIT(PORTD, SSI_CS1n);
    
    timer2_init();
    
    sei(); // enable interrupts
    
    reset_controller();
    
    while(1) {
        sleep_mode();
        
        if(ir_state == IR_STATE_READY) {
            printf("%02hhX %02hhX %02hhX %02hhX\r\n", ir_data[0], ir_data[1], ir_data[2], ir_data[3]);
            if(check_checksum(ir_data, 0xC0)) {
                bad_cnt = 0;
                target_speed = 2 * (ir_data[1] & 0x0F);
                if(ir_data[2] & 0x40)
                    target_speed = -target_speed;
            } else {
                if(bad_cnt < 10)
                    bad_cnt++;
            }
            if(bad_cnt > 8)
                target_speed = 0;
            ir_state = IR_STATE_ARMED;
        }
        
        if(!holdoff) {
            // Read out wheel position
            speed_0 = get_speed(&wheel_pos_0);
            speed_1 = get_speed(&wheel_pos_1);
            
            motor_0 = sp_ctrl_step(target_speed,
                                   speed_0,
                                   &error_int_0);
            motor_1 = sp_ctrl_step(target_speed,
                                   speed_1,
                                   &error_int_1);
            
            set_speed(motor_0, motor_1);
            
            holdoff = 1;
            
            /* se_start_frame(8);
            se_puti16(target_speed);
            se_puti16(speed_0);
            se_puti16(error_int_0);
            se_puti16(motor_0); */
        }
    }
    
  /* while(1) {
    if(cmd_ready) {
      switch(cmdbuf[0]) {
        case 0x0A:
          desired_speed_0 = ((int8_t) cmdbuf[1]);
          desired_speed_1 = ((int8_t) cmdbuf[2]);
          break;
        case 0x11:
          cmd_multi_angle(cmdbuf[1]);
          break;
        default:
          break;
      }
      cmd_ready = 0;
    }
    sleep_mode();
  } */
}
