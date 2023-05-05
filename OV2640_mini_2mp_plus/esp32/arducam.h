
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


spi_device_handle_t spi;
QueueHandle_t spp_uart_queue;

#ifndef __ARDUCAM_H
#define __ARDUCAM_H

/*spi pin source*/
#define PIN_SCK  18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_CS   5

/*i2c pin source */
#define PIN_SDA  21
#define PIN_SCL  22
#define WRITE_BIT 0x80

#define BAUD_RATE 921600
#define UART_TX_PIN 1
#define UART_RX_PIN 3

#ifndef _SENSOR_
#define _SENSOR_
    struct sensor_reg {
        unsigned int reg;
        unsigned int val;
    };
#endif

struct sensor_info{
    uint8_t sensor_slave_address;
    uint8_t address_size;
    uint8_t data_size; 
    uint16_t sensor_id;
};
struct camera_operate{
    uint8_t slave_address;
    void (*systemInit)(void);
    uint8_t (*busDetect) (void);
    uint8_t (*cameraProbe) (void);
    void  (*cameraInit) (void);
    void (*setJpegSize)(uint8_t size);
    void (*takePicture) (void);
};
#define res_160x120 		0	//160x120
#define res_176x144 		1	//176x144
#define res_320x240 		2	//320x240
#define res_352x288 		3	//352x288
#define res_640x480		    4	//640x480
#define res_800x600 		5	//800x600
#define res_1024x768		6	//1024x768
#define res_1280x1024	7	//1280x1024
#define res_1600x1200	8	//1600x1200
#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20
#define ARDUCHIP_GPIO			0x06  //GPIO Write Register
#define GPIO_RESET_MASK			0x01  //0 = Sensor reset,							1 =  Sensor normal operation
#define GPIO_PWDN_MASK			0x02  //0 = Sensor normal operation, 	1 = Sensor standby
#define GPIO_PWREN_MASK			0x04	//0 = Sensor LDO disable, 			1 = sensor LDO enable

#define BURST_FIFO_READ			0x3C  //Burst FIFO read operation
#define SINGLE_FIFO_READ		0x3D  //Single FIFO read operation

#define ARDUCHIP_REV       		0x40  //ArduCHIP revision
#define VER_LOW_MASK       		0x3F
#define VER_HIGH_MASK      		0xC0

#define ARDUCHIP_TRIG      		0x41  //Trigger source
#define VSYNC_MASK         		0x01
#define SHUTTER_MASK       		0x02
#define CAP_DONE_MASK      		0x08

#define FIFO_SIZE1				0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44  //Camera write FIFO size[18:16]

extern uint8_t cameraCommand;
extern struct camera_operate arducam;
extern uint8_t slave_addr;
void rdSensorReg8_8(uint8_t regID, uint8_t* regDat );
void wrSensorReg8_8(uint8_t regID, uint8_t regDat );
void wrSensorRegs8_8(const struct sensor_reg reglist[]);
void write_reg(uint8_t address, uint8_t value);
uint8_t read_reg(uint8_t address);
void singleCapture(void);
#endif