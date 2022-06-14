#include "main.h"

void main(void) {
    uint8_t data_hi = 0, data_lo = 0;
    uint16_t set_val = 0, dac_val = 0;
    char msg_lcd[10];
    
    TRISA=0x03;
    TRISB=0x00;
    TRISC=0x00;
    
    adc_set();
    i2c_master(100000);
    lcd_set();
    
    __delay_ms(100);
    
    while(1){
        adc_ch(0);
        set_val = adc_read();
        
        sprintf(msg_lcd, "adc_in = %d \n", set_val);
        lcd_adr(0,0);
        lcd_sendstr(msg_lcd);
        
        // For fast mode
        data_lo = set_val & 0xFF;
        data_hi = set_val >> 8;
        
        i2c_start();                            
		i2c_sendaddr(0xC0+0);                   
		i2c_write(data_hi);
        i2c_write(data_lo);
        i2c_stop();
        __delay_ms(1);
        
        
        adc_ch(1);
        dac_val = adc_read();
        sprintf(msg_lcd, "dac_out = %d \n", dac_val);
        lcd_adr(1,0);
        lcd_sendstr(msg_lcd);
        __delay_ms(100);
    }
    
    return;
}

