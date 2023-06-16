#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/pwm.h"

uint8_t data_in = 0;

void tud_cdc_rx_cb(uint8_t itf)
{
    data_in = 0xAA;
}

int main() {
    uint16_t duty = 0;
    uint8_t per = 0, step = 10; // min = 1637  max = 4758
    uint8_t to_go = 0, to_set = 0, count = 0, i = 0;
    uint8_t databuf[10], servobuf[10];


    sleep_ms(250);

    board_init();
    tusb_init(); 

    gpio_set_function(0, GPIO_FUNC_PWM);
    gpio_set_function(1, GPIO_FUNC_PWM);
    gpio_set_function(3, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(0);
    pwm_set_clkdiv(slice_num, 61);
    pwm_set_wrap(slice_num, 40984);
    pwm_set_chan_level(slice_num, 0, 1637);
    pwm_set_enabled(slice_num, true);

    uint slice_num2 = pwm_gpio_to_slice_num(1);
    pwm_set_clkdiv(slice_num2, 61);
    pwm_set_wrap(slice_num2, 40984);
    pwm_set_chan_level(slice_num2, 1, 1637);
    pwm_set_enabled(slice_num2, true);

    uint slice_num3 = pwm_gpio_to_slice_num(3);
    pwm_set_clkdiv(slice_num3, 61);
    pwm_set_wrap(slice_num3, 40984);
    pwm_set_chan_level(slice_num3, 3, 1637);
    pwm_set_enabled(slice_num3, true);

    while(1){
        tud_task();
        if(data_in > 0){
            count = tud_cdc_n_read(0, databuf, sizeof(databuf));
            if(to_set == 0){
                switch(databuf[0]){
                    case 's':
                        to_set = 0xAA;
                        i = 0;
                    break;
                    default:
                    break;
                }
            }
            else{
                servobuf[i] = databuf[0];
                i++;
                if(i == 6){
                    pwm_set_chan_level(slice_num, 0, (servobuf[0] << 8) + servobuf[1]);
                    pwm_set_chan_level(slice_num2, 1, (servobuf[2] << 8) + servobuf[3]);
                    pwm_set_chan_level(slice_num3, 3, (servobuf[4] << 8) + servobuf[5]);
                    to_set = 0;
                }
            }
            data_in = 0;
        }
    }
}
