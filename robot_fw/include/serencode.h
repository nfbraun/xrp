#ifndef __SERENCODE_H__
#define __SERENCODE_H__

#include <inttypes.h>

void se_start_frame(uint8_t n);
void se_putchr(char c);
void se_puti16(uint16_t x);
void se_puti32(uint32_t x);
void se_putdata(const char* c, int n);

#endif
