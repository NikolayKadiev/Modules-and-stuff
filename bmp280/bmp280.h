#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"


#define BMP280_ADDR             0x76
#define BMP280_ID               0xd0
#define BMP280_RESET            0xe0
#define BMP280_STATUS           0xf3
#define BMP280_CTL_MEAS         0xf4
#define BMP280_CONFIG           0xf5
#define BMP280_PRESS_MSB        0xf7
#define BMP280_PRESS_LSB        0xf8
#define BMP280_PRESS_XLSB       0xf9
#define BMP280_TEMP_MSB         0xfa
#define BMP280_TEMP_LSB         0xfb
#define BMP280_TEMP_XLSB        0xfc

#define BMP280_CALIBRATION_DIG_T1       0x88
#define BMP280_CALIBRATION_DIG_T2       0x8a
#define BMP280_CALIBRATION_DIG_T3       0x8c
#define BMP280_CALIBRATION_DIG_P1       0x8e
#define BMP280_CALIBRATION_DIG_P2       0x90
#define BMP280_CALIBRATION_DIG_P3       0x92
#define BMP280_CALIBRATION_DIG_P4       0x94
#define BMP280_CALIBRATION_DIG_P5       0x96
#define BMP280_CALIBRATION_DIG_P6       0x98
#define BMP280_CALIBRATION_DIG_P7       0x9a
#define BMP280_CALIBRATION_DIG_P8       0x9c
#define BMP280_CALIBRATION_DIG_P9       0x9e

void bmp280_write_reg_8_bit(uint8_t w_reg, uint8_t w_data);

uint8_t bmp280_read_reg_8_bit(uint8_t r_reg);

uint32_t bmp280_read_reg_24_bit(uint8_t r_reg);

uint16_t bmp280_read_const(uint8_t addr);

uint8_t bmp280_get_status(void);

uint8_t bmp280_conv_status(void);

void bmp280_default_cfg(void);

uint8_t bmp280_get_temp_press(float* temper, float* pressure);