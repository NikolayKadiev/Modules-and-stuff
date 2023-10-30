
#define UC_ESP32 0
#define UC_PICO 1
#define UC_PIC 2

#define UC_IN_USE UC_ESP32


#if UC_IN_USE == UC_ESP32

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"


#define PIN_SCK     18
#define PIN_MOSI    23
#define PIN_MISO    19
#define PIN_CSN     5
#define PIN_CE      17

spi_device_handle_t spi;

static inline void set_pins(void){
    gpio_set_direction(PIN_CSN, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CSN, 1);
    gpio_set_direction(PIN_CE, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CE, 1);

    spi_bus_config_t buscfg={
            .miso_io_num=PIN_MISO,
            .mosi_io_num=PIN_MOSI,
            .sclk_io_num=PIN_SCK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz=100,
    };

    spi_device_interface_config_t devcfg={
            .clock_speed_hz= 500*1000,           
            .mode=0,                          
            .spics_io_num=-1,               
            .queue_size=150,
    };	

    printf("bus init %d\n", spi_bus_initialize(VSPI_HOST, &buscfg, 0));
    printf("device add %d\n",spi_bus_add_device(VSPI_HOST, &devcfg, &spi));
}

static inline void nRF24_CE_L(void) {
    gpio_set_level(PIN_CE, 0);
}

static inline void nRF24_CE_H(void) {
    gpio_set_level(PIN_CE, 1);
}

static inline void nRF24_CSN_L(void) {
    gpio_set_level(PIN_CSN, 0);
}

static inline void nRF24_CSN_H(void) {
    gpio_set_level(PIN_CSN, 1);
}

static inline uint8_t nRF24_LL_RW(uint8_t data) {
    spi_transaction_t txrx;
    memset(&txrx, 0, sizeof(txrx));

    txrx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    txrx.tx_buffer = NULL;
    txrx.length = 1*8;
    txrx.tx_data[0] = data;
    txrx.rxlength = 1*8;

	spi_device_polling_transmit(spi, &txrx);
	return txrx.rx_data[0];
}

static inline void Delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

#endif

#if UC_IN_USE == UC_PICO

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"


#define PIN_SCK     2
#define PIN_MOSI    3
#define PIN_MISO    4
#define PIN_CSN     5
#define PIN_CE      6

static inline void set_pins(void){

    spi_init(SPI_PORT, 1*1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CSN);
    gpio_set_dir(PIN_CSN, GPIO_OUT);
    gpio_put(PIN_CSN, 1);
    gpio_init(PIN_CE);
    gpio_set_dir(PIN_CE, GPIO_OUT);
    gpio_put(PIN_CE, 1);
}

static inline void nRF24_CE_L(void) {
    gpio_put(PIN_CE, 0);
}

static inline void nRF24_CE_H(void) {
    gpio_put(PIN_CE, 0);
}

static inline void nRF24_CSN_L(void) {
    gpio_put(PIN_CSN, 0);
}

static inline void nRF24_CSN_H(void) {
    gpio_put(PIN_CSN, 1);
}

static inline uint8_t nRF24_LL_RW(uint8_t data) {
    uint8_t value = 0;
	spi_write_blocking(SPI_PORT, &data, 1);
    spi_read_blocking(SPI_PORT, 0, &value, 1);
	return value;
}

static inline void Delay_ms(uint32_t ms) {
    sleep_ms(ms);
}
#endif

#if UC_IN_USE == UC_PIC

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "clock_config.h"
#include "spi.h"

#define PIN_CSN     RC2
#define PIN_CE      RC1

static inline void set_pins(void){
    TRISC2 = 0;
    TRISC1 = 0;
    PIN_CE = 1;
    PIN_CSN = 1;

    spi_master(0);
}

static inline void nRF24_CE_L(void) {
    PIN_CE = 0;
}

static inline void nRF24_CE_H(void) {
    PIN_CE = 1;
}

static inline void nRF24_CSN_L(void) {
    PIN_CSN = 0;
}

static inline void nRF24_CSN_H(void) {
    PIN_CSN = 1;
}

static inline uint8_t nRF24_LL_RW(uint8_t data) {
	return = spi_transfer(data);
}

static inline void Delay_ms(uint32_t ms) {
    __delay_ms(ms);
}

#endif
