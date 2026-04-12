#include "max30100.h"


i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;


esp_err_t max30100_write_reg (uint8_t reg, uint8_t data)
{
    uint8_t wr_buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle, wr_buf, 2, I2C_MS_TIMEOUT / portTICK_PERIOD_MS);
}

esp_err_t max30100_read_reg (uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, I2C_MS_TIMEOUT / portTICK_PERIOD_MS);
}

void max30100_init (void)
{
    i2c_master_bus_config_t bus_conf = {
        .i2c_port = I2C_IN_USE,
        .scl_io_num = I2C_PIN_SCL,
        .sda_io_num = I2C_PIN_SDA,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_conf, &bus_handle);

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX301000_ADDR,
        .scl_speed_hz = I2C_FRQZ_HZ,
    };
    i2c_master_bus_add_device(bus_handle, &dev_conf, &dev_handle);

    max30100_write_reg(REG_MODE_CTRL, BIT_RESET);

    max30100_write_reg(REG_FIFO_WR_PTR, 0);
    max30100_write_reg(REG_FIFO_RD_PTR, 0);

    max30100_write_reg(REG_MODE_CTRL, BIT_MODE_SPO2);

    max30100_write_reg(REG_SPO2_CONF, BIT_SPO2_HI_RES | BIT_SPO2_SR_50 | BIT_LED_PW_1600);

    max30100_write_reg(REG_LED_CONF, BIT_RED_3 | BIT_IR_3);

    max30100_write_reg(REG_INT_ENABLE, BIT_SPO2_RDY_EN);
}