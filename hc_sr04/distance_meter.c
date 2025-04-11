
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include <sys/time.h>
#include "driver/gpio.h"

#define TRIG_PIN    22
#define ECHO_PIN    23

QueueHandle_t queue;
gptimer_handle_t gptimer = NULL;
uint64_t golb_glass = 0, start_time = 0, end_time = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg){
    if(gpio_get_level(ECHO_PIN) == 1){
        gptimer_get_raw_count(gptimer, &start_time);
    }
    else{
        gptimer_get_raw_count(gptimer, &end_time);
        gptimer_stop(gptimer);
        golb_glass = end_time - start_time;
        xQueueSendFromISR(queue, &golb_glass, NULL);
    }
}

void app_main(void){
    uint64_t state = 0;
    float distance = 0;

   queue = xQueueCreate(5, sizeof(uint64_t));
   configASSERT(queue);

   gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
   gpio_set_level(TRIG_PIN, 0);

   gpio_config_t io_conf = {
   .intr_type = GPIO_INTR_ANYEDGE,
   .pin_bit_mask = (1ULL<<ECHO_PIN),
   .mode = GPIO_MODE_INPUT,
   };
   gpio_config(&io_conf);
   gpio_install_isr_service(0);
   gpio_isr_handler_add(ECHO_PIN, gpio_isr_handler, (void*) ECHO_PIN);
   gpio_intr_enable(ECHO_PIN);
   
   gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
   };
   gptimer_new_timer(&timer_config, &gptimer);
   gptimer_enable(gptimer);
   gptimer_alarm_config_t alarm_config1 = {
      .alarm_count = 2*1000000, // period = 2s
   };

   printf("HC-SR04 test\n");

   while(1){
    gptimer_set_raw_count(gptimer, 0);
    gptimer_set_alarm_action(gptimer, &alarm_config1);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);
    gptimer_start(gptimer);
    xQueueReceive(queue, &state, portMAX_DELAY);
    distance = (state * 340.0) / 2000000.0;
    printf("time = %lld us <=> distance = %.2f m\n", state, distance);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}