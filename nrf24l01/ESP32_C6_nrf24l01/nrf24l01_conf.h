#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"


#define PIN_SCK     1
#define PIN_MOSI    22
#define PIN_MISO    2
#define PIN_CSN     21
#define PIN_CE      0
#define PIN_IRQ     23

void set_pins(void);

void nRF24_CE_L(void);

void nRF24_CE_H(void);

void nRF24_CSN_L(void);

void nRF24_CSN_H(void);

uint8_t nRF24_LL_RW(uint8_t data);

void Delay_ms(uint32_t ms);
