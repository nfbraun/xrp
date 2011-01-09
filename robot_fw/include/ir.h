#ifndef __IR_H__
#define __IR_H__

#include <inttypes.h>

#define IR_DATA_SIZE 4

extern volatile uint8_t ir_state;
extern volatile uint8_t ir_data[IR_DATA_SIZE];

#define IR_STATE_ARMED 255
#define IR_STATE_READY IR_DATA_SIZE

int8_t check_checksum(volatile uint8_t *buf, uint8_t ch);
void ir_init(void);


#endif
