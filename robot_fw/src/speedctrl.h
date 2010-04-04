#ifndef __SPEEDCTRL_H__
#define __SPEEDCTRL_H__

#include <inttypes.h>

int8_t sp_ctrl_step(int16_t set_speed, int16_t speed, int16_t* error_int);

typedef struct {
  int16_t wheel_pos_0;      // wheel pos at time of call to init_wheel_pos()
  int16_t old_wheel_pos;    // wheel pos at time of last call to get_speed()
  int16_t wheel_pos_offset; // wheel position offset in complete encoder revolutions (1024 steps)
  int8_t ch;
} wheelpos_t;

#define GET_WHEEL_POS(wp) ((wp).old_wheel_pos - (wp).wheel_pos_0 + 1024 * (wp).wheel_pos_offset)

#endif
