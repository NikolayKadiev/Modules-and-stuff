#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "max30100.h"

/* With interrups*/

QueueHandle_t gpio_evt_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg){
    uint32_t section = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &section, NULL);
}

void app_main (void){
    uint8_t int_status;
    uint8_t fifo_data[4];
    uint8_t pin_status = 0;
    uint16_t ir_raw = 0, red_raw = 0;

    gpio_evt_queue = xQueueCreate(5, sizeof(int));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_23);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_23, gpio_isr_handler, (void*) GPIO_NUM_23);
    gpio_intr_enable(GPIO_NUM_23);

    max30100_init();

    while(1){
        if (xQueueReceive(gpio_evt_queue, &pin_status, portMAX_DELAY)){
            max30100_read_reg(REG_INT_STATUS, &int_status, 1);

            if(int_status & BIT_SPO2_RDY){
                if(max30100_read_reg(REG_FIFO_DATA, fifo_data, 4) == ESP_OK){
                    ir_raw = (fifo_data[0] << 8) | fifo_data[1];
                    red_raw = (fifo_data[2] << 8) | fifo_data[3];
                    // printf("%u, %u\n", ir_raw, red_raw);
                    // printf("%u\n", red_raw);
                    printf("%u\n", ir_raw);
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// /* Without interrups*/
// void app_main (void){
//     uint8_t int_status;
//     uint8_t fifo_data[4];
//     uint16_t ir_raw = 0, red_raw = 0;

//     max30100_init();

//     while(1){
//         max30100_read_reg(REG_INT_STATUS, &int_status, 1);

//         if(int_status & BIT_SPO2_RDY){
//             if(max30100_read_reg(REG_FIFO_DATA, fifo_data, 4) == ESP_OK){
//                 ir_raw = (fifo_data[0] << 8) | fifo_data[1];
//                 red_raw = (fifo_data[2] << 8) | fifo_data[3];
//                 printf("%u, %u\n", ir_raw, red_raw);
//                 // printf("%u\n", red_raw);
//                 // printf("%u\n", ir_raw);
//             }
//         }

//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }