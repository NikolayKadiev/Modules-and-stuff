#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "driver/i2c.h"

#define LCD_ADDR 0x4E

// RS           ->  P0
// R/W          ->  P1
// E            ->  P2
// DB0          ->  N.C.
// DB1          ->  N.C.
// DB2          ->  N.C.
// DB3          ->  N.C.
// DB4          ->  P4
// DB5          ->  P5
// DB6          ->  P6
// DB7          ->  P7
// Back Light   ->  P3

void lcd_control (uint8_t comand, uint8_t baclight_use);

void lcd_init(uint8_t pin_sda, uint8_t pin_scl);

void lcd_send(uint8_t data, uint8_t baclight_use);

void lcd_shift(char x, uint8_t baclight_use);

void lcd_send_msg(uint8_t *msg, uint8_t msg_len, uint8_t baclight_use);

void lcd_adr(uint8_t row, uint8_t col, uint8_t baclight_use);

void lcd_clear (uint8_t baclight_use);
