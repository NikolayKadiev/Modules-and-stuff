#include "main.h"

const uint8_t ADDRESS = 0x1C<<1;

//hardware registers

const uint8_t REG_X_MSB = 0x01;
const uint8_t REG_X_LSB = 0x02;
const uint8_t REG_Y_MSB = 0x03;
const uint8_t REG_Y_LSB = 0x04;
const uint8_t REG_Z_MSB = 0x05;
const uint8_t REG_Z_LSB = 0x06;
const uint8_t REG_DATA_CFG = 0x0E;
const uint8_t REG_CTRL_REG1 = 0x2A;

// Set the range and precision for the data 
const uint8_t range_config = 0x00; // 0x00 for ±2g, 0x01 for ±4g, 0x02 for ±8g
const float count = 4096; // 4096 for ±2g, 2048 for ±4g, 1024 for ±8g

float mma8452_convert_accel(uint16_t raw_acc) {
    float acceleration;
    uint16_t raw_accel = raw_acc;
    // Acceleration is read as a multiple of g (gravitational acceleration on the Earth's surface)
    // Check if acceleration < 0 and convert to decimal accordingly
    if ((raw_accel & 0x800) == 0x800) {
        raw_accel &= 0x7FF;
        acceleration = (-2046 + (float) raw_accel) / count;
    }
    else {
        acceleration = (float) raw_accel / count;
    }
    acceleration *= 9.81f;
    return acceleration;
}


void mma8452_write_reg(uint8_t reg_addr, uint8_t data) {
    i2c_start();
    i2c_sendaddr(ADDRESS + 0);
    i2c_write(reg_addr);
    i2c_write(data);                      
    i2c_stop();
}

uint8_t mma8452_get_status(void) {
    uint8_t read = 0;
    i2c_start();
    i2c_sendaddr(ADDRESS + 0);
    i2c_write(0x00);
    i2c_repstart();
    i2c_sendaddr(ADDRESS + 1);                   
	read = i2c_read(0);                        
    i2c_stop();
    return read;
}

uint16_t mma8452_get_result(uint8_t reg_addr) {
    uint8_t read1 = 0, read2 = 0;
    uint16_t ressult = 0;
    i2c_start();
    i2c_sendaddr(ADDRESS + 0);
    i2c_write(reg_addr);
    i2c_repstart();
    i2c_sendaddr(ADDRESS + 1);                   
	read1 = i2c_read(1);
    read2 = i2c_read(0);
    i2c_stop();
    ressult = (read1 << 4) | (read2 >> 4);
    return ressult;
}

void main(void) {
    uint16_t res_x = 0, res_y = 0, res_z = 0; 
    float x_acc = 0.0, y_acc = 0.0, z_acc = 0.0;
    uint8_t buff[100];
    
    TRISA=0x00;
    TRISB=0x00;
    TRISC=0x00;
    
    uart_set();
    i2c_master(100000);
    __delay_ms(100);
    
    mma8452_write_reg(REG_CTRL_REG1, 0);
    mma8452_write_reg(REG_DATA_CFG, range_config);
    mma8452_write_reg(REG_CTRL_REG1, 1);
    
    mma8452_write_reg(REG_CTRL_REG1, 0);
    // mma8452_write_reg(REG_CTRL_REG3, 2);
    // mma8452_write_reg(REG_CTRL_REG4, 1);
    // mma8452_write_reg(REG_CTRL_REG5, 1);
    mma8452_write_reg(REG_CTRL_REG1, 0b00011001);
    
    while(1){
        if(mma8452_get_status() & 0x07){
            res_x = mma8452_get_result(REG_X_MSB);
            res_y = mma8452_get_result(REG_Y_MSB);
            res_z = mma8452_get_result(REG_Z_MSB);
            
            x_acc = mma8452_convert_accel(res_x);
            y_acc = mma8452_convert_accel(res_y);
            z_acc = mma8452_convert_accel(res_z);

            sprintf(buff, "x= %.3f\t y= %.3f\tz= %.3f\n", x_acc, y_acc, z_acc);
//            sprintf(buff, "x= %d\t y= %d\tz= %d\n", res_x, res_y, res_z);

            uart_sendstr(buff);
        }
    }
    return;
}
