#include "serial.h"

const uint8_t DATA_PREFIX = 0x40;
const uint8_t SHORT_FRAME_PREFIX = 0x50;
const uint8_t LONG_FRAME_PREFIX = 0x60;

void se_start_frame(uint8_t n)
{
    if(n < 0x10) {
        s_putchr(SHORT_FRAME_PREFIX | n);
    } else {
        s_putchr(LONG_FRAME_PREFIX);
        s_putchr(n);
    }
}

void se_putchr(uint8_t c)
{
    s_putchr(DATA_PREFIX | (c & 0x0F));
    s_putchr(DATA_PREFIX | (c >> 4));
}

void se_puti16(uint16_t x)
{
    se_putchr(x & 0x00FF);
    se_putchr((x & 0xFF00) >> 8);
}

void se_puti32(uint32_t x)
{
    se_putchr((x & 0x000000FF));
    se_putchr((x & 0x0000FF00) >>  8);
    se_putchr((x & 0x00FF0000) >> 16);
    se_putchr((x & 0xFF000000) >> 24);
}

void se_putdata(const char* c, int n)
{
    int i;
    for(i=0; i<n; ++i)
        se_putchr(*c++);
}
