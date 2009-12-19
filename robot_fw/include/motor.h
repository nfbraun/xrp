#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <inttypes.h>

void motor_init(void);
void set_speed(int8_t left, int8_t right);

#endif
