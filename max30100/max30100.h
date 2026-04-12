#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"



/*-------------------DEVICE REGISTERS--------------------------*/
#define REG_INT_STATUS          0x00
#define REG_INT_ENABLE          0x01
#define REG_FIFO_WR_PTR         0x02
#define REG_FIFO_OVF_PTR        0x03
#define REG_FIFO_RD_PTR         0x04
#define REG_FIFO_DATA           0x05
#define REG_MODE_CTRL           0x06
#define REG_SPO2_CONF           0x07
#define REG_LED_CONF            0x09
#define REG_TEMP_INT            0x16
#define REG_TEMP_FRA            0x17
#define REG_REV_ID              0xFE
#define REG_PART_ID             0xFF

/*-------------------REGISTERS BITS--------------------------*/
#define BIT_A_FULL             0x80
#define BIT_TEMP_RDY           0x40
#define BIT_HR_RDY             0x20
#define BIT_SPO2_RDY           0x10
#define BIT_PWR_RDY            0x01

#define BIT_A_FULL_EN             0x80
#define BIT_TEMP_RDY_EN           0x40
#define BIT_HR_RDY_EN             0x20
#define BIT_SPO2_RDY_EN           0x10

#define BIT_SHDN             0x80
#define BIT_RESET            0x40
#define BIT_TEMP_EN          0x08
#define BIT_MODE_HR          0x02
#define BIT_MODE_SPO2        0x03

#define BIT_SPO2_HI_RES          0x40

#define BIT_SPO2_SR_50           0x00
#define BIT_SPO2_SR_100          0x04
#define BIT_SPO2_SR_167          0x08
#define BIT_SPO2_SR_200          0x0C
#define BIT_SPO2_SR_400          0x10
#define BIT_SPO2_SR_600          0x14
#define BIT_SPO2_SR_800          0x18
#define BIT_SPO2_SR_1000         0x1C

#define BIT_LED_PW_200           0x00
#define BIT_LED_PW_400           0x01
#define BIT_LED_PW_800           0x02
#define BIT_LED_PW_1600          0x03

#define BIT_RED_0             0x00
#define BIT_RED_1             0x10
#define BIT_RED_2             0x20
#define BIT_RED_3             0x30
#define BIT_RED_4             0x40
#define BIT_RED_5             0x50
#define BIT_RED_6             0x60
#define BIT_RED_7             0x70
#define BIT_RED_8             0x80
#define BIT_RED_9             0x90
#define BIT_RED_10            0xA0
#define BIT_RED_11            0xB0
#define BIT_RED_12            0xC0
#define BIT_RED_13            0xD0
#define BIT_RED_14            0xE0
#define BIT_RED_15            0xF0

#define BIT_IR_0              0x00
#define BIT_IR_1              0x01
#define BIT_IR_2              0x02
#define BIT_IR_3              0x03
#define BIT_IR_4              0x04
#define BIT_IR_5              0x05
#define BIT_IR_6              0x06
#define BIT_IR_7              0x07
#define BIT_IR_8              0x08
#define BIT_IR_9              0x09
#define BIT_IR_10             0x0A
#define BIT_IR_11             0x0B
#define BIT_IR_12             0x0C
#define BIT_IR_13             0x0D
#define BIT_IR_14             0x0E
#define BIT_IR_15             0x0F

/*-------------------DEVICE FUNCTIONS--------------------------*/
#define I2C_PIN_SCL             22
#define I2C_PIN_SDA             21
#define I2C_IN_USE              I2C_NUM_0
#define I2C_FRQZ_HZ             100*1000
#define I2C_MS_TIMEOUT          100

#define MAX301000_ADDR          0x57

esp_err_t max30100_write_reg (uint8_t reg, uint8_t data);
esp_err_t max30100_read_reg (uint8_t reg, uint8_t *data, uint8_t len);
void max30100_init (void);