#include "bitset.h"
#include "serial.h"
#include "motor.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define abs(x) ((x) < 0 ? (-(x)) : (x))

#define SSI_CS0n 2
#define SSI_CS1n 3

int16_t get_angle(int8_t ch)
{
    uint16_t dat;

    _CLRBIT(PORTD, ch);
    dat = ssi_read();
    _SETBIT(PORTD, ch);
       
    return (dat >> 6);
}

volatile int16_t old_angle_0, old_angle_1;
volatile int16_t desired_speed_0 = 0, desired_speed_1 = 0;
int16_t error_int_0 = 0, error_int_1 = 0;

ISR(TIMER2_COMP_vect)
{
    int16_t angle, speed, error, out_0, out_1;

    // Motor 0
    angle = get_angle(SSI_CS0n);
    
    speed = angle - old_angle_0;
    old_angle_0 = angle;
    if(speed > 800) speed -= 1024;
    if(speed < -800) speed += 1024;
    
    error = desired_speed_0 - speed;
    error_int_0 += error;
    
    out_0 = error + (error_int_0 >> 1);
    
    if(out_0 > 100) out_0 = 100;
    if(abs(out_0) < 15) out_0 = 0;
    if(out_0 < -100) out_0 = -100;
    
    // Motor 1
    angle = get_angle(SSI_CS1n);
        
    speed = old_angle_1 - angle;
    old_angle_1 = angle;
    if(speed > 800) speed -= 1024;
    if(speed < -800) speed += 1024;
    
    error = desired_speed_1 - speed;
    error_int_1 += error;
    
    out_1 = error + (error_int_1 >> 1);
    
    if(out_1 > 100) out_1 = 100;
    if(abs(out_1) < 15) out_1 = 0;
    if(out_1 < -100) out_1 = -100;
    
    printf("%d %d\n", out_0, out_1);
    
    set_speed(out_0, out_1);
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
    OCR2 = F_CPU / (1024UL * 50UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
}

int main()
{
  motor_init();
  serial_init();
  //twi_init();
  ssi_init();
  scons_init();
  
  _SETBIT(DDRD, SSI_CS0n);
  _SETBIT(DDRD, SSI_CS1n);
  _SETBIT(PORTD, SSI_CS0n);
  _SETBIT(PORTD, SSI_CS1n);
  
  _SETBIT(DDRD, 4);
  
  old_angle_0 = get_angle(SSI_CS0n);
  old_angle_1 = get_angle(SSI_CS1n);
  timer2_init();

  sei(); // enable interrupts

  while(1) {
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
  }
}
