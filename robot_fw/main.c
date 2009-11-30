#include "global.h"
#include "serial.h"
#include "motor.h"
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define SSI_CS0n 2

void cmd_multi_angle(uint8_t n)
{
  uint8_t i;
  uint16_t val;
  
  for(i=0; i<n; ++i) {
    _CLRBIT(PORTD, SSI_CS0n);
    val = ssi_read();
    _SETBIT(PORTD, SSI_CS0n);
    _delay_ms(10);
    s_puti16(val);
  }
}

int main()
{
  motor_init();
  serial_init();
  ssi_init();
  _SETBIT(DDRD, SSI_CS0n);
  _SETBIT(PORTD, SSI_CS0n);
  
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
