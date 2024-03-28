#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"

#define MMA8452_ADDR            0x1C<<1

//hardware registers

#define REG_X_MSB       0x01
#define REG_X_LSB       0x02
#define REG_Y_MSB       0x03
#define REG_Y_LSB       0x04
#define REG_Z_MSB       0x05
#define REG_Z_LSB       0x06
#define REG_DATA_CFG    0x0E
#define REG_CTRL_REG1   0x2A
#define REG_CTRL_REG2   0x2B
#define REG_CTRL_REG3   0x2C
#define REG_CTRL_REG4   0x2D
#define REG_CTRL_REG5   0x2E

// Set the range and precision for the data 
#define range_config    0x00 // 0x00 for ±2g, 0x01 for ±4g, 0x02 for ±8g
#define count           4096 // 4096 for ±2g, 2048 for ±4g, 1024 for ±8g

float mma8452_convert_accel(uint16_t raw_acc);
void mma8452_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t mma8452_get_status(void);
uint16_t mma8452_get_result(uint8_t reg_addr);
void mma8452_simple_init(void);
