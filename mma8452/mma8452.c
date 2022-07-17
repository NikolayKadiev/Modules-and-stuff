#include "main.h"

const uint8_t ADDRESS = 0x1C;

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
const uint8_t range_config = 0x01; // 0x00 for �2g, 0x01 for �4g, 0x02 for �8g
const float count = 2048; // 4096 for �2g, 2048 for �4g, 1024 for �8g

float mma8452_convert_accel(uint16_t raw_accel) {
    float acceleration;
    // Acceleration is read as a multiple of g (gravitational acceleration on the Earth's surface)
    // Check if acceleration < 0 and convert to decimal accordingly
    if ((raw_accel & 0x2000) == 0x2000) {
        raw_accel &= 0x1FFF;
        acceleration = (-8192 + (float) raw_accel) / count;
    } else {
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

uint8_t mma8452_get_status(uint8_t reg_addr) {
    uint8_t read = 0;
    i2c_start();
    i2c_sendaddr(ADDRESS + 0);
    i2c_write(reg_addr);
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
    ressult = (read1 << 8) | read2;
    return ressult;
}

void main(void) {
    uint8_t res_x =0, res_y =0, res_z =0; 
    float real_x = 0.0, real_y = 0.0, real_z = 0.0;
    uint8_t buff[30];
    
    TRISA=0x00;
    TRISB=0x00;
    TRISC=0x00;
    
    uart_set();
    i2c_master(100000);
    __delay_ms(100);
    
    mma8452_write_reg(REG_CTRL_REG1, 0);
    mma8452_write_reg(REG_DATA_CFG, range_config);
    mma8452_write_reg(REG_CTRL_REG1, 1);
    
    while(1){
        
        res_x = mma8452_get_result(REG_X_MSB);
        res_y = mma8452_get_result(REG_Y_MSB);
        res_z = mma8452_get_result(REG_Z_MSB);
        
        real_x = mma8452_convert_accel(res_x);
        real_y = mma8452_convert_accel(res_y);
        real_z = mma8452_convert_accel(res_z);
        
        sprintf(buff, "%.3f,%.3f,%.3f\n", real_x, real_y, real_z);
        
        uart_sendstr(buff);
    }
    return;
}