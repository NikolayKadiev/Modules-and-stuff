#include "main.h"

void main(void) {
    uint8_t data_hi = 0, data_lo = 0, pec = 0;
    char msg_lcd[10];
    float temp = 0.0, celc = 0.0;
    
    TRISA=0x01;
    TRISB=0x00;
    TRISC=0x00;
    i2c_master(100000);
    lcd_set();
    sprintf(msg_lcd, "t0 = %d  \n", 0);
    __delay_ms(100);
    lcd_adr(0,0);
    lcd_sendstr(msg_lcd);
    while(1){
        i2c_start();                            
		i2c_sendaddr(0xB4+0);                   
		i2c_write(0x07);                        
        i2c_repstart();                         
		i2c_sendaddr(0xB4+1);                   
		data_lo=i2c_read(1);                      
		data_hi=i2c_read(1);                      
		pec = i2c_read(0);                      
        i2c_stop();
		temp =(((data_hi & 0x7F) << 8)+ data_lo);  
		temp =(temp*0.02)-0.01;                
		celc = temp - 273.15;                  
        
        sprintf(msg_lcd, "t0 = %.2f C \n", celc);
        __delay_ms(55);
        lcd_adr(0,0);
        lcd_sendstr(msg_lcd);
        __delay_ms(100);
        __delay_ms(100);
    }
    
    return;
}
