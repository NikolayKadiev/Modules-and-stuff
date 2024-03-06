#include "clock_config.h"
#include <xc.h>
#include <stdint.h>

void spi_master(uint8_t mode);

void spi_slave(uint8_t mode);

void spi_wait(void);

void spi_write(uint8_t dat);

uint8_t spi_read(void);

uint8_t spi_transfer(uint8_t out);
