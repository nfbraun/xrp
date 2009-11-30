#ifndef GLOBAL_H
#define GLOBAL_H

#define F_CPU 8000000UL

#ifndef _UV
#define _UV(x) (1 << (x))
#endif

#ifndef _SETBIT
#define _SETBIT(x, y) ((x) |= _UV(y))
#endif

#ifndef _CLRBIT
#define _CLRBIT(x, y) ((x) &= ~_UV(y))
#endif

#endif
