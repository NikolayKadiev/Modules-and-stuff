#include "mma8452.h"


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
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MMA8452_ADDR | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr , 1);
    i2c_master_write_byte(cmd, data , 1);      
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

uint8_t mma8452_get_status(void) {
    i2c_cmd_handle_t cmd;
    uint8_t read = 0;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MMA8452_ADDR | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, 0x00 , 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MMA8452_ADDR | I2C_MASTER_READ, 1);   
    i2c_master_read_byte(cmd, &read, 0);                           
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return read;
}

uint16_t mma8452_get_result(uint8_t reg_addr) {
    i2c_cmd_handle_t cmd;
    uint8_t read1 = 0, read2 = 0;
    uint16_t ressult = 0;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MMA8452_ADDR | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr , 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MMA8452_ADDR | I2C_MASTER_READ, 1);  
    i2c_master_read_byte(cmd, &read1, 1);   
    i2c_master_read_byte(cmd, &read2, 0);                           
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ressult = (read1 << 4) | (read2 >> 4);
    return ressult;
}

void mma8452_simple_init(void){
    mma8452_write_reg(REG_CTRL_REG1, 0);
    mma8452_write_reg(REG_DATA_CFG, range_config);
    mma8452_write_reg(REG_CTRL_REG1, 1);
    
    mma8452_write_reg(REG_CTRL_REG1, 0);
    mma8452_write_reg(REG_CTRL_REG3, 2);
    mma8452_write_reg(REG_CTRL_REG4, 1);
    mma8452_write_reg(REG_CTRL_REG5, 1);
    mma8452_write_reg(REG_CTRL_REG1, 0b00011001);
}
