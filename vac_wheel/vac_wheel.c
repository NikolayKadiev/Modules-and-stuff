#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"


QueueHandle_t uart_queue;
uint16_t speed_motor = 3900;

void set_motor(void){
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 23,
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = 22;
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = 21;
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_channel.channel = LEDC_CHANNEL_3;
    ledc_channel.gpio_num = 19;
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void app_main(void){
    uart_event_t event;
    uint8_t * temp = NULL;
    uint8_t position = 0;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 20,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 1024, 1024, 20, &uart_queue,0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    set_motor();

    printf("Motor start\n");

    while(1){
        if (xQueueReceive(uart_queue, (void * )&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    memset(temp,0x0,event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                break;
                default:
                break;
            }

            if (((temp[0] == 1) || (temp[0] == '1')) && (speed_motor <= 16300)){
                speed_motor += 100;
            }
                
            if (((temp[0] == 2) || (temp[0] == '2')) && (speed_motor > 0)){
                speed_motor -= 100;
            }
                
            if ((temp[0] == 3) || (temp[0] == '3')){
                if(!position){
                    position = 1;
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                }
                else{
                    position = 0;
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                }

            }

            printf("%d\n", speed_motor);

            if(position){
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed_motor);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, speed_motor);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
            }
            else{
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed_motor);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, speed_motor);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            }

            free(temp);
        }
    }
}
