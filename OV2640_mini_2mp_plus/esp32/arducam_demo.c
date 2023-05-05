#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "arducam.h"


void app_main(void)
{
    
    uart_event_t event;
    uint8_t * temp = NULL;

    arducam.systemInit();
    arducam.busDetect();
    arducam.cameraProbe();
    arducam.cameraInit();

    while(1){
        if (xQueueReceive(spp_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    memset(temp,0x0,event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                break;
                default:
                break;
            }
            switch(temp[0]){
                    case '1':
                        arducam.takePicture();
                        break;
                    case '2':
                        arducam.setJpegSize(res_160x120);
                        break;
                    case '3':
                        arducam.setJpegSize(res_176x144);
                        break;
                    case '4':
                        arducam.setJpegSize(res_320x240);
                        break;
                    case '5':
                        arducam.setJpegSize(res_352x288);
                        break;
                    case '6':
                        arducam.setJpegSize(res_640x480);
                        break;
                    case '7':
                        arducam.setJpegSize(res_800x600);
                        break;
                    case '8':
                        arducam.setJpegSize(res_1024x768);
                        break;
                    case '9':
                        arducam.setJpegSize(res_1280x1024);
                        break;
                    default:
                        break;
                }
            free(temp);
        }
    }
}

