#include "clock_config.h"
#include <xc.h>
#include <stdint.h>

void i2c_master(const unsigned long I2C_c);

void i2c_wait(void);

void i2c_start(void);

void i2c_repstart(void);

void i2c_stop(void);

void i2c_write(uint8_t I2C_d);

void i2c_sendaddr(uint8_t adr);

uint8_t i2c_read(uint8_t I2C_a);

