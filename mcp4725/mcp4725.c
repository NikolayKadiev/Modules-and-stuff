#include "main.h"

void main(void) {
    uint8_t data_hi = 0, data_lo = 0;
    uint16_t set_val = 100;
    char msg_lcd[10];
    
    TRISA=0x01;
    TRISB=0x00;
    TRISC=0x00;
    
    i2c_master(100000);
    lcd_set();
    
    __delay_ms(100);
    
    while(1){
        // For fast mode
        data_lo = set_val & 0xFF;
        data_hi = set_val >> 8;
        
        i2c_start();                            
		i2c_sendaddr(0xC0+0);                   
		i2c_write(data_hi);
        i2c_write(data_lo);
        i2c_stop();
        
        sprintf(msg_lcd, "int out = %d \n", set_val);
        lcd_adr(0,0);
        lcd_sendstr(msg_lcd);
        __delay_ms(100);
        __delay_ms(100);
    }
    
    return;
}
