#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "oled_i2c.h"

#define MLX90614_ADDR 0xB4

uint16_t get_temp(void){
    uint8_t data[3];
	i2c_cmd_handle_t cmd_read;
	cmd_read = i2c_cmd_link_create();
	i2c_master_start(cmd_read);
	i2c_master_write_byte(cmd_read, MLX90614_ADDR | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd_read, 0x07 , 1);
	i2c_master_start(cmd_read);
	i2c_master_write_byte(cmd_read, MLX90614_ADDR | I2C_MASTER_READ, 1);
	i2c_master_read(cmd_read, data, 3, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd_read);
	i2c_master_cmd_begin(I2C_NUM_1 , cmd_read, 5 / portTICK_RATE_MS); //portMAX_DELAY
	i2c_cmd_link_delete(cmd_read);
    return ((data[1] & 0x7F) << 8) | data[0];
}

void app_main(void){

    float celc = 0.0;
    uint16_t temp = 0;
    char msg[20];

	i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100*1000;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_NUM_0 , &conf);
    i2c_driver_install(I2C_NUM_0 , conf.mode, 0, 0, 0);

    OLED_Init();


    while(1){          
        temp = get_temp(); 
		celc = ((temp*0.02)-0.01) - 273.15;  
        printf("t0 = %.2f C\n", celc);
		OLED_SetCursor(5,1);
        sprintf(msg, "Temp = %.2f C ", celc);
		OLED_DisplayString(msg);
        vTaskDelay(500 / portTICK_RATE_MS);
    }

}
