#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "bmp280.h"

void app_main (void){
    float temper = 0.0, press = 0.0;

    bmp280_default_cfg();

    while(1){
        bmp280_get_temp_press(&temper, &press);
        printf("t = %.2f C\t press = %.2f\n\r", temper, press);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}