#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "oled_i2c.h"
#include <driver/ledc.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO

void i2c_master_init(uint8_t sda, uint8_t scl, uint32_t clk){
	i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_io_num = sda;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_io_num = scl;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = clk;
		i2c_param_config(I2C_NUM_1, &conf);
		i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);
}

static xQueueHandle gpio_evt_queue = NULL;
volatile static xQueueHandle rotary_que = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg){
    uint32_t section = 0;
    if(gpio_get_level(GPIO_NUM_18)){
    	section = 0xA0;
    }
    else{
    	section = 0x0A;
    }
    xQueueSendFromISR(gpio_evt_queue, &section, NULL);
}

static void gpio_task_example(void* arg){
	uint16_t count = 0;
	uint8_t state = 0;
	while(1){
		xQueueReceive(gpio_evt_queue, &state, portMAX_DELAY);
		if(state == 0xA0 && count < 1024){
			count++;
		}
		if(state == 0x0A && count > 0){
			count--;
		}
		ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, count);
		ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
		xQueueSend(rotary_que, &count, 0);
	}
}

static void oled_task(void* arg){
	uint16_t count=0;
	char msg[20];

	while(1){
		xQueueReceive(rotary_que, &count, portMAX_DELAY);
		sprintf(msg, "count = %d   ", count);
		OLED_SetCursor(4,1);
		OLED_DisplayString(msg);
	}
}

void app_main(void){
//	uint16_t count=0;
//	char msg[20];
	gpio_config_t io_conf;

	ledc_timer_config_t ledc_timer = {
			.bit_num = LEDC_TIMER_10_BIT,
		    .freq_hz = 50000,
		    .speed_mode = LEDC_HIGH_SPEED_MODE,
			.clk_cfg = LEDC_APB_CLK,
		    .timer_num = LEDC_TIMER_1
	};
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel = {
			.channel = LEDC_CHANNEL_0,
			.duty = 0,
			.gpio_num = 23,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.timer_sel = LEDC_TIMER_1,
	};
	ledc_channel_config(&ledc_channel);

	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.pin_bit_mask = GPIO_NUM_19;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.pin_bit_mask = GPIO_NUM_18;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_set_intr_type(GPIO_NUM_19, GPIO_INTR_NEGEDGE);
	gpio_evt_queue = xQueueCreate(5, sizeof(uint16_t));
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_19, gpio_isr_handler, (void*) GPIO_NUM_19);
	gpio_intr_enable(GPIO_NUM_19);

	rotary_que = xQueueCreate(5, sizeof(uint16_t));
	xTaskCreate(gpio_task_example, "gpio_task_example", 2 * 1024, NULL, 15, NULL);
	xTaskCreate(oled_task, "oled_task", 4 * 1024, NULL, 14, NULL);

	i2c_master_init(21, 22, 100000);

	OLED_Init();

//	OLED_SetCursor(0,0);
//	OLED_DisplayString("Hello there!");
//	OLED_SetCursor(2,0);
//	OLED_DisplayString("My name is ESP32");
//	OLED_SetCursor(4,0);
//	OLED_DisplayString("And this is OLED \ntesting!");
//	vTaskDelay(3000 / portTICK_PERIOD_MS);
//	OLED_Clear();
//
//	sprintf(msg, "count = %d   ", count);
	OLED_SetCursor(4,1);
	OLED_DisplayString("count = 0");
}
