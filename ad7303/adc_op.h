#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 4000000

void adc_set(void);

uint16_t adc_read(void);

void adc_ch(int n);