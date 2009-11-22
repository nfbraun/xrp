#include <inttypes.h>

void twi_init(void);
int twi_read(uint8_t addr, uint8_t* buf, int len);
