
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include <sys/time.h>
#include "driver/gpio.h"

#define TRIG_PIN    22
#define FRQ_IN_PIN    23

QueueHandle_t queue;
gptimer_handle_t gptimer = NULL;
gptimer_alarm_config_t alarm_config1 = {
   .alarm_count = 2*1000000, // period = 2s
};
uint8_t signal_start = 0;
uint64_t end_time = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg){
    if(signal_start == 0){
        gptimer_set_raw_count(gptimer, 0);
        gptimer_set_alarm_action(gptimer, &alarm_config1);
        gptimer_start(gptimer);
        signal_start = 1;
    }
    else{
        signal_start = 0;
        gptimer_get_raw_count(gptimer, &end_time);
        gptimer_stop(gptimer);
        xQueueSendFromISR(queue, &end_time, NULL);
    }
}

void app_main(void){
    uint64_t state = 0;
    float frq_in = 0.0;

   queue = xQueueCreate(5, sizeof(uint64_t));
   configASSERT(queue);

   gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
   gpio_set_level(TRIG_PIN, 0);

   gpio_config_t io_conf = {
   .intr_type = GPIO_INTR_POSEDGE,
   .pin_bit_mask = (1ULL<<FRQ_IN_PIN),
   .mode = GPIO_MODE_INPUT,
   };
   gpio_config(&io_conf);
   gpio_install_isr_service(0);
   gpio_isr_handler_add(FRQ_IN_PIN, gpio_isr_handler, (void*) FRQ_IN_PIN);
   gpio_intr_enable(FRQ_IN_PIN);
   
   gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
   };
   gptimer_new_timer(&timer_config, &gptimer);
   gptimer_enable(gptimer);

   printf("Frequency Meter\n");

   while(1){
    xQueueReceive(queue, &state, portMAX_DELAY);
    gpio_intr_disable(FRQ_IN_PIN);
    frq_in = 1000000.0 / (float) state;
    printf("Period = %lld us => Frequency = %.2f Hz\n", state, frq_in);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_intr_enable(FRQ_IN_PIN);
   }
}