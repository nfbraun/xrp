#ifndef __ADC_H__
#define __ADC_H__

#include <inttypes.h>

void adc_start_single(uint8_t c);
void adc_init(void);
int16_t adc_read(uint8_t c);

#endif
