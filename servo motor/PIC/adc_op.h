#include <xc.h>
#include <stdint.h>
#include "clock_config.h"

void adc_set(void);

uint16_t adc_read(void);

void adc_ch(int n);
