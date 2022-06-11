#include "main.h"

void main(void) {
    uint16_t adc_ress1 = 0, adc_ress2 =0;
    char msg_lcd[20];
    
    TRISA=0x03;
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
        sprintf(msg_lcd, "ad = %d  \n", adc_ress1);
        lcd_adr(0,0);
        lcd_sendstr(msg_lcd); 
        
        RC2 = 0; 
        __delay_ms(1);
        spi_write(0x11);
        __delay_ms(1);
        spi_write(adc_ress1>>2);
        __delay_ms(1);
        RC2 = 1; 
        
        __delay_ms(100);           
        adc_ch(1);
        adc_ress2 = adc_read();
        sprintf(msg_lcd, "pot = %d  \n", adc_ress2);
        lcd_adr(1,0);
        lcd_sendstr(msg_lcd);
        __delay_ms(100);
        __delay_ms(100);
    }
    
    return;
}
