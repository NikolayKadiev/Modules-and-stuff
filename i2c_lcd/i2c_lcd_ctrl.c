#include "i2c_lcd_ctrl.h"

// Send comand to LCD
void lcd_control (uint8_t comand, uint8_t baclight_use) {
    uint8_t lcd_rs = 0, lcd_en = 0;
    uint8_t data_out[4];
	i2c_cmd_handle_t cmd_write;
    
    lcd_en = 1;
    lcd_rs = 0;
    data_out[0] = ((comand&0xF0) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));
    lcd_en = 0;
    lcd_rs = 0;
    data_out[1] = ((comand&0xF0) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));
    lcd_en = 1;
    lcd_rs = 0;
    data_out[2] = ((comand << 4) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));
    lcd_en = 0;
    lcd_rs = 0;
    data_out[3] = ((comand << 4) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));

	cmd_write = i2c_cmd_link_create();
	i2c_master_start(cmd_write);
	i2c_master_write_byte(cmd_write, LCD_ADDR | I2C_MASTER_WRITE, 1);
	i2c_master_write(cmd_write, data_out, 4, 1);
	i2c_master_stop(cmd_write);
	i2c_master_cmd_begin(I2C_NUM_0 , cmd_write, 1000 / portTICK_PERIOD_MS); //portMAX_DELAY
	i2c_cmd_link_delete(cmd_write);


}

// Set up LCD for use - 4 wires
void lcd_init(uint8_t pin_sda, uint8_t pin_scl){
	i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100*1000;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_NUM_0 , &conf);
    i2c_driver_install(I2C_NUM_0 , conf.mode, 0, 0, 0);
    vTaskDelay(255 / portTICK_PERIOD_MS);

    lcd_control(0x30, 0);
    ets_delay_us(4500);
    lcd_control(0x30, 0);
    ets_delay_us(4500);
    lcd_control(0x30, 0);
    ets_delay_us(45000);
    lcd_control(0x20, 0);
    ets_delay_us(200);
    lcd_control(0x28, 0);
    ets_delay_us(1000);
    lcd_control(0x0c, 0);
    ets_delay_us(50);
    lcd_control(0x01, 0);
    ets_delay_us(2000);
    lcd_control(0x06, 0);
    lcd_control(0x0c, 0);
    ets_delay_us(2000);
}

// Send a character to LCD
void lcd_send(uint8_t data, uint8_t baclight_use){
    uint8_t lcd_rs = 0, lcd_en = 0;
    uint8_t data_out[4];
	i2c_cmd_handle_t cmd_write;
 
    lcd_en = 1;
    lcd_rs = 1;
    data_out[0] = ((data&0xF0) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));
    lcd_en = 0;
    lcd_rs = 1;
    data_out[1] = ((data&0xF0) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));
    lcd_en = 1;
    lcd_rs = 1;
    data_out[2] = ((data << 4) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));
    lcd_en = 0;
    lcd_rs = 1;
    data_out[3] = ((data << 4) | lcd_rs | (lcd_en << 2) | (baclight_use << 3));

	cmd_write = i2c_cmd_link_create();
	i2c_master_start(cmd_write);
	i2c_master_write_byte(cmd_write, LCD_ADDR | I2C_MASTER_WRITE, 1);
	i2c_master_write(cmd_write, data_out, 4, 1);
	i2c_master_stop(cmd_write);
	i2c_master_cmd_begin(I2C_NUM_0 , cmd_write, 1000 / portTICK_PERIOD_MS); //portMAX_DELAY
	i2c_cmd_link_delete(cmd_write);

}

// Shift left or right
void lcd_shift(char x, uint8_t baclight_use){
    lcd_control(0x05, baclight_use);
    switch(x){
        case 'R': lcd_control(0x18, baclight_use);
        break;
        case 'L': lcd_control(0x1C, baclight_use);
        break;
    }
}

//Send a string to LCD
void lcd_send_msg(uint8_t *msg, uint8_t msg_len, uint8_t baclight_use){
	for (uint8_t i = 0; i < msg_len; i++){
		lcd_send(msg[i], baclight_use);
	}
	return;
}

//Send address to LCD
void lcd_adr(uint8_t row, uint8_t col, uint8_t baclight_use){
    vTaskDelay(10 / portTICK_PERIOD_MS);
	if(row==1){
		lcd_control(0xc0 + col, baclight_use);
	}
	else{
		lcd_control(0x80 + col, baclight_use);
	}
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

//Clear lines of LCD
void lcd_clear(uint8_t baclight_use){
    lcd_control(0x01, baclight_use);
}
