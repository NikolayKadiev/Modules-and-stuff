#include <xc.h>
#include <stdint.h>
#include "clock_config.h"

// Use a spare PORT of your pic
#define EN_LCD   RB3
#define RS_LCD   RB2
#define PT_OUT   PORTB
#define TR_OUT   TRISB

void lcd_control (uint8_t comand);

void lcd_set(void);

void lcd_send(char data);

void lcd_shift(char x);

void lcd_sendstr(char str[]);

void lcd_adr(uint8_t row, uint8_t col);
