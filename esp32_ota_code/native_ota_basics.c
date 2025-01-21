
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "errno.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#define PRES_APP_VER 1

static const char *TAG = "native_ota_example";

QueueHandle_t uart_queue;

static void running_task_on(void *pvParameter){
    while(1){
        gpio_set_level(GPIO_NUM_33, 1);
        vTaskDelay(500 /portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_33, 0);
        vTaskDelay(500 /portTICK_PERIOD_MS);
        if(gpio_get_level(GPIO_NUM_32) == 0){
            printf("Started with version No: %d\n", PRES_APP_VER);
        }
    }
}

void app_main(void)
{
    uart_event_t event;
    uint8_t * temp = NULL;

    uint8_t start_upd_state = 0;
    size_t new_app_len = 0;
    uint16_t in_buff_size = 0;
    esp_ota_handle_t update_handle = 0 ;
    bool image_header_was_checked = false;
    const esp_partition_t *update_partition = NULL;
    char go_msg[] = {"GO\n"};
    char ok_msg[] = {"OK\n"};
    char end_msg[] = {"END\n"};
    TaskHandle_t xHandle;

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 20,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, 1024, 1024, 20, &uart_queue,0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 19, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_33, 0);
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_NUM_32);
    
    xTaskCreate(&running_task_on, "running_task_on", 3*1024, NULL, 5, &xHandle);

    while(1){
        if (xQueueReceive(uart_queue, (void * )&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    memset(temp,0x0,event.size);
                    uart_read_bytes(UART_NUM_1,temp,event.size,portMAX_DELAY);
                    in_buff_size = event.size;
                    break;
                default:
                    break;
            }

            if(start_upd_state == 0){
                if((temp[0] == 'N') && (temp[1] == 'A') && (temp[2] == 'V')){
                    new_app_len = (temp[3]  << 24) | (temp[4]  << 16) | (temp[5]  << 8) | temp[6];
                    start_upd_state = 1;
                    printf("GO\n");
                    uart_write_bytes(UART_NUM_1, go_msg, strlen(go_msg));
                    vTaskSuspend(xHandle);
                    free(temp);
                    in_buff_size = 1000;
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*1000);
                    while(1){
                        uart_read_bytes(UART_NUM_1,temp,in_buff_size,portMAX_DELAY);
                        if(image_header_was_checked == false){
                            update_partition = esp_ota_get_next_update_partition(NULL);
                            if(esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle) != ESP_OK){
                                printf("Problem begin\n");
                            }
                            image_header_was_checked = true;
                        }
                        if(esp_ota_write(update_handle, (const void *)temp, in_buff_size) != ESP_OK){
                                esp_ota_abort(update_handle);
                                printf("Problem write\n");
                            }
                        new_app_len -= in_buff_size;
                        if((new_app_len < in_buff_size) &&  (new_app_len > 0)){
                            in_buff_size = new_app_len;
                        }
                        if(new_app_len == 0){
                            if(esp_ota_end(update_handle) != ESP_OK){
                                printf("Problem end\n");
                            }
                            esp_ota_set_boot_partition(update_partition);
                            printf("END\n");
                            uart_write_bytes(UART_NUM_1, end_msg, strlen(end_msg));
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            esp_restart();
                        }
                        else{
                            printf("OK\n");
                            uart_write_bytes(UART_NUM_1, ok_msg, strlen(ok_msg));
                        }
                    }
                    
                }
            }
        }
        else{
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}
