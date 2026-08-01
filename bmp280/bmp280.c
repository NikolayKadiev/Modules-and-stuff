#include "bmp280.h"

#define I2C_MS_TIMEOUT 100

uint32_t bmp280_t_fine = 0;
uint16_t bmp280_dig_t1 = 0;
int16_t  bmp280_dig_t2 = 0, bmp280_dig_t3 = 0;
uint16_t bmp280_dig_p1 = 0;
int16_t  bmp280_dig_p2 = 0, bmp280_dig_p3 = 0, bmp280_dig_p4 = 0, bmp280_dig_p5 = 0, bmp280_dig_p6 = 0, bmp280_dig_p7 = 0, bmp280_dig_p8 = 0, bmp280_dig_p9 = 0;

float bmp280_t1_calc_1 = 0.0, bmp280_p3_calc_1 = 0.0;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

void bmp280_default_cfg(void){
    
    i2c_master_bus_config_t bus_conf = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_conf, &bus_handle);

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_ADDR,
        .scl_speed_hz = 100 * 1000,
    };
    i2c_master_bus_add_device(bus_handle, &dev_conf, &dev_handle);

    bmp280_write_reg_8_bit(BMP280_RESET, 0xb6);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    bmp280_dig_t1 = bmp280_read_const(BMP280_CALIBRATION_DIG_T1);
    bmp280_dig_t2 = bmp280_read_const(BMP280_CALIBRATION_DIG_T2);
    bmp280_dig_t3 = bmp280_read_const(BMP280_CALIBRATION_DIG_T3);

    bmp280_dig_p1 = bmp280_read_const(BMP280_CALIBRATION_DIG_P1);
    bmp280_dig_p2 = bmp280_read_const(BMP280_CALIBRATION_DIG_P2);
    bmp280_dig_p3 = bmp280_read_const(BMP280_CALIBRATION_DIG_P3);
    bmp280_dig_p4 = bmp280_read_const(BMP280_CALIBRATION_DIG_P4);
    bmp280_dig_p5 = bmp280_read_const(BMP280_CALIBRATION_DIG_P5);
    bmp280_dig_p6 = bmp280_read_const(BMP280_CALIBRATION_DIG_P6);
    bmp280_dig_p7 = bmp280_read_const(BMP280_CALIBRATION_DIG_P7);
    bmp280_dig_p8 = bmp280_read_const(BMP280_CALIBRATION_DIG_P8);
    bmp280_dig_p9 = bmp280_read_const(BMP280_CALIBRATION_DIG_P9);

    bmp280_write_reg_8_bit(BMP280_CTL_MEAS, 0x27); //0x48
    bmp280_t1_calc_1 = ((float) bmp280_dig_t1) / 8192.0;
    bmp280_p3_calc_1 = ((float) bmp280_dig_p3) / 524288.0;
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

uint8_t bmp280_get_temp_press(float* temperature, float* pressure)
{
    int32_t adc_p = 0;
    int32_t adc_t = 0;
    float t_var1 = 0.0;
    float t_var2 = 0.0;
    float p_var1 = 0.0;
    float p_var2 = 0.0;
    float p = 0.0;

    if (temperature == NULL || pressure == NULL) {
        return 0x11;
    }

    while (bmp280_conv_status()) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    adc_p = bmp280_read_reg_24_bit(BMP280_PRESS_MSB);
    adc_t = bmp280_read_reg_24_bit(BMP280_TEMP_MSB);

    t_var1 = (((float)adc_t) / 16384.0 - ((float)bmp280_dig_t1) / 1024.0) * ((float)bmp280_dig_t2);
    t_var2 = ((((float)adc_t) / 131072.0 - ((float)bmp280_dig_t1) / 8192.0) *
                    (((float)adc_t) / 131072.0 - ((float)bmp280_dig_t1) / 8192.0)) * ((float)bmp280_dig_t3);

    bmp280_t_fine = (int32_t)(t_var1 + t_var2);
    *temperature = (t_var1 + t_var2) / 5120.0;

    p_var1 = ((float)bmp280_t_fine / 2.0) - 64000.0;
    p_var2 = p_var1 * p_var1 * ((float)bmp280_dig_p6) / 32768.0;
    p_var2 = p_var2 + p_var1 * ((float)bmp280_dig_p5) * 2.0;
    p_var2 = (p_var2 / 4.0) + (((float)bmp280_dig_p4) * 65536.0);
    p_var1 = (((float)bmp280_dig_p3) * p_var1 * p_var1 / 524288.0 + ((float)bmp280_dig_p2) * p_var1) / 524288.0;
    p_var1 = (1.0 + p_var1 / 32768.0) * ((float)bmp280_dig_p1);

    if (p_var1 == 0.0) {
        *pressure = 0.0;
        return 0x22; 
    }

    p = 1048576.0 - (float)adc_p;
    p = (p - (p_var2 / 4096.0)) * 6250 / p_var1;
    p_var1 = ((float)bmp280_dig_p9) * p * p / 2147483648.0;
    p_var2 = p * ((float)bmp280_dig_p8) / 32768.0;
    *pressure = (p + (p_var1 + p_var2 + ((float)bmp280_dig_p7)) / 16.0) / 100;

    return 0;
}

uint8_t bmp280_get_status(void){
    return bmp280_read_reg_8_bit(BMP280_STATUS);
}

uint8_t bmp280_conv_status(void){
    return bmp280_get_status() >> 3;
}

void bmp280_write_reg_8_bit(uint8_t w_reg, uint8_t w_data){
    uint8_t wr_buf[2] = {w_reg, w_data};
    i2c_master_transmit(dev_handle, wr_buf, 2, I2C_MS_TIMEOUT / portTICK_PERIOD_MS);
}

uint8_t bmp280_read_reg_8_bit(uint8_t r_reg){
    uint8_t read_reg = 0;
    i2c_master_transmit_receive(dev_handle, &r_reg, 1, &read_reg, 1, I2C_MS_TIMEOUT / portTICK_PERIOD_MS);
	return read_reg;
}

uint16_t bmp280_read_const(uint8_t addr){
    uint8_t data[2];
    i2c_master_transmit_receive(dev_handle, &addr, 1, data, 2, I2C_MS_TIMEOUT / portTICK_PERIOD_MS);
	return (uint16_t) ((data[1]<<8) | data[0]);
}

uint32_t bmp280_read_reg_24_bit(uint8_t r_reg){
    uint8_t data[3];

    i2c_master_transmit_receive(dev_handle, &r_reg, 1, data, 3, I2C_MS_TIMEOUT / portTICK_PERIOD_MS);

    return ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;
}