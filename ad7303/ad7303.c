#include "main.h"

void main(void) {
    uint16_t adc_ress1 = 0, adc_ress2 = 0, adc_ress3 = 0;
    char msg_lcd[20];
    
    TRISA=0x07;
    TRISB=0x00;
    TRISC=0x00;
    RC2 = 1; 
    
    adc_set();
    spi_master(0);
    lcd_set();
    __delay_ms(100);
    
    while(1){                
        adc_ch(0);
        adc_ress1 = adc_read();
        
        RC2 = 0; 
        __delay_ms(1);
        spi_write(0x00); // write both channels at once. See AD7303 datasheet for more info
        spi_write(adc_ress1>>2);
        __delay_ms(1);
        RC2 = 1; 
        
        __delay_ms(10);           
        adc_ch(1);
        adc_ress2 = adc_read();
        sprintf(msg_lcd, "chann A = %d  \n", adc_ress2);
        lcd_adr(0,0);
        lcd_sendstr(msg_lcd);     
        adc_ch(2);
        adc_ress3 = adc_read();
        sprintf(msg_lcd, "chann B = %d  \n", adc_ress3);
        lcd_adr(1,0);
        lcd_sendstr(msg_lcd);
        
        __delay_ms(100);
        __delay_ms(100);
    }
    
    return;
}
