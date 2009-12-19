#include <stdio.h>
#include "serial.h"

static int __s_putchr(char c, FILE* stream)
{
    s_putchr(c);
    return 0;
}

static FILE s_stdout = FDEV_SETUP_STREAM(__s_putchr, NULL,
                                         _FDEV_SETUP_WRITE);
                                         
void scons_init(void)
{
    stdout = &s_stdout;
}
