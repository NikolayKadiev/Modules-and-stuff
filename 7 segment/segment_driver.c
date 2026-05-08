#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "string.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   22
#define PIN_NUM_CLR  21

#define LED_NUM_A    0x7f
#define LED_NUM_B    0xfe
#define LED_NUM_C    0xfb
#define LED_NUM_D    0xef
#define LED_NUM_E    0xdf
#define LED_NUM_F    0xbf
#define LED_NUM_G    0xfd
#define LED_NUM_DP   0xf7
#define LED_NUM_NO   0xff

#define SEG_NUM_0    0x0a
#define SEG_NUM_1    0xfa
#define SEG_NUM_2    0x4c
#define SEG_NUM_3    0x68
#define SEG_NUM_4    0xb8
#define SEG_NUM_5    0x29
#define SEG_NUM_6    0x09
#define SEG_NUM_7    0x7a
#define SEG_NUM_8    0x08
#define SEG_NUM_9    0x28

#define DIG_NUM_1    0x07
#define DIG_NUM_2    0x0b
#define DIG_NUM_3    0x0d
#define DIG_NUM_4    0x0e

uint8_t show_dig[] = {
    DIG_NUM_1,
    DIG_NUM_2,
    DIG_NUM_3,
    DIG_NUM_4
};

uint8_t show_char[] = {
    LED_NUM_NO,
    LED_NUM_A,
    LED_NUM_B,
    LED_NUM_C,
    LED_NUM_D,
    LED_NUM_E,
    LED_NUM_F,
    LED_NUM_G,
    LED_NUM_DP,
    SEG_NUM_0,
    SEG_NUM_1,
    SEG_NUM_2,
    SEG_NUM_3,
    SEG_NUM_4,
    SEG_NUM_5,
    SEG_NUM_6,
    SEG_NUM_7,
    SEG_NUM_8,
    SEG_NUM_9
};

spi_device_handle_t spi;

spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
 };

 spi_device_interface_config_t devcfg={
        .clock_speed_hz=1*1000*1000,           //Clock out at 100 kHz
        .mode=0,                                //SPI mode
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=10,                          //We want to be able to queue 10 transactions at a time
  };


void app_main(void)
{
    gpio_set_direction(PIN_NUM_CLR,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CLR, 0);

    spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    gpio_set_level(PIN_NUM_CLR, 1);

    spi_transaction_t trans;

    while(1){
        for(uint8_t j = 0; j < 4; j ++){
            for(uint8_t i = 0; i < 19; i ++){
                    memset(&trans, 0, sizeof(trans));
                    trans.length = 2*8;
                    trans.tx_data[0] = show_char[i];
                    trans.tx_data[1] = show_dig[j];
                    trans.flags = SPI_TRANS_USE_TXDATA;
                    spi_device_transmit(spi, &trans);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
            }
        }
    }

}