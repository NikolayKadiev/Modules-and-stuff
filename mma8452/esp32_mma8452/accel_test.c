#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#include "mma8452.h"

#define PIN_DRDY    17

QueueHandle_t gpio_evt_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg){
    uint32_t section = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &section, NULL);
}

static void set_up_pins(void){
    gpio_evt_queue = xQueueCreate(5, sizeof(uint8_t));
    configASSERT(gpio_evt_queue);

    // This example will use I2C
	i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100*1000;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_NUM_0 , &conf);
    i2c_driver_install(I2C_NUM_0 , conf.mode, 0, 0, 0);

    // Set up our UART with the required speed.
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 20,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL,0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_POSEDGE,
    .pin_bit_mask = (1ULL<<PIN_DRDY),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_DRDY, gpio_isr_handler, (void*) PIN_DRDY);
}

void app_main(void){
    uint16_t res_x = 0, res_y = 0, res_z = 0; 
    float x_acc = 0.0, y_acc = 0.0, z_acc = 0.0;
    uint8_t state = 0;
    char buff[100];

    set_up_pins();
    mma8452_simple_init();

    while(1){

        if(xQueueReceive(gpio_evt_queue, &state, portMAX_DELAY)){
            res_x = mma8452_get_result(REG_X_MSB);
            res_y = mma8452_get_result(REG_Y_MSB);
            res_z = mma8452_get_result(REG_Z_MSB);
            
            x_acc = mma8452_convert_accel(res_x);
            y_acc = mma8452_convert_accel(res_y);
            z_acc = mma8452_convert_accel(res_z);

            sprintf(buff, "%.3f,%.3f,%.3f\n", x_acc, y_acc, z_acc);
            // sprintf(buff, "x= %d\t y= %d\tz= %d\n", x_acc, y_acc, z_acc);
            // sprintf(buff, "x= %d\t y= %d\tz= %d\n", res_x, res_y, res_z);
            uart_write_bytes(UART_NUM_0, buff, strlen(buff));
        }

    }
}