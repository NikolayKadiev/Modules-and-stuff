#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "arducam.h"

int main() {
    
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

    arducam.systemInit();
    if(arducam.busDetect()){
        return 1;
    }
    if(arducam.cameraProbe()){
        return 1;
    }
    arducam.cameraInit();
    
    gpio_put(25, 1);

    while (true) {
        if(cameraCommand != 0){
            switch(cameraCommand){
                case '1':
                    singleCapture();
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
            cameraCommand = 0;
        }
    }
    return 0;
}
