#include "global.h"
#include "serial.h"
#include "motor.h"
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

void cmd_multi_angle(uint8_t n)
{
  uint8_t i;
  uint8_t val[4];
  
  for(i=0; i<n; ++i) {
    twi_read(0x28, val, 4);
    _delay_ms(10);
    s_putdata(val, sizeof(val));
  }
}

int main()
{
  motor_init();
  serial_init();
  twi_init();

  sei(); // enable interrupts

  while(1) {
    if(cmd_ready) {
      switch(cmdbuf[0]) {
        case 0x0A:
          set_speed(cmdbuf[1], cmdbuf[2]);
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
