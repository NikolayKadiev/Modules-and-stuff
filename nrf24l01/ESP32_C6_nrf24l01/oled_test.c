#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <sys/param.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "oled_i2c.h"


void app_main(void)
{       
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 20;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 19;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100*1000;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_NUM_0 , &conf);
    i2c_driver_install(I2C_NUM_0 , conf.mode, 0, 0, 0);

    OLED_Init();

    while(1){
        OLED_SetCursor(0,1);
        OLED_DisplayString("Hello!");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

}
