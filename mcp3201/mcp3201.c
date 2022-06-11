#include "main.h"

void main(void) {
    uint16_t ad_1 = 0, ad_2 = 0;
    uint16_t ress = 0;
    float voltage = 0.0, v_ref = 5.0;
    char msg_lcd[20];
    
    TRISA=0x03;
    TRISB=0x00;
    TRISC=0x00;
    RC2 = 1; 
    
    spi_master(1);
    lcd_set();
    __delay_ms(100);
    
    while(1){
        RC2 = 0; 
        __delay_ms(1);
        ad_1 = spi_transfer(0);
        ad_2 = spi_transfer(0);
        __delay_ms(1);
        RC2 = 1; 
        
        ress = (((ad_1 & 0x1F) << 8) | ad_2) >> 1;
        
        sprintf(msg_lcd, "adc = %d   \n", ress);
        lcd_adr(0,0);
        lcd_sendstr(msg_lcd);
       
        voltage = (ress * v_ref) / 4096.0;
        
        sprintf(msg_lcd, "volt = %.2f  \n", voltage);
        lcd_adr(1,0);
        lcd_sendstr(msg_lcd);
        __delay_ms(100);
        __delay_ms(100);
    }
    
    return;
}
