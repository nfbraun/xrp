#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <inttypes.h>

extern volatile uint8_t cmdbuf[8];
extern volatile int8_t cmd_ready;

void serial_init(void);
void s_putchr(char c);
void s_puti16(uint16_t x);
void s_putstr(const char* c, int n);

#endif
