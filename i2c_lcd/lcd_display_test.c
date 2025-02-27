#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include <sys/unistd.h>
#include <sys/stat.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "i2c_lcd_ctrl.h"


void app_main(void){
    char msg_arr[100];
    uint8_t count = 0, back_light_en = 0;

    lcd_init(19, 18);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    lcd_adr(0, 0, back_light_en);
    sprintf(msg_arr, "Hello!");
    lcd_send_msg((uint8_t *) msg_arr, strlen(msg_arr), back_light_en);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    lcd_clear(back_light_en);
    // back_light_en = 1;

    while(1){ 
        lcd_adr(1, 0, back_light_en);
        sprintf(msg_arr, "Count = %d", count);
        lcd_send_msg((uint8_t *) msg_arr, strlen(msg_arr), back_light_en);
        count ++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}