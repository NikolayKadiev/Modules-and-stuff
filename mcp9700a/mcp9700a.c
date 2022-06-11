#include "main.h"

void main(void) {
    uint16_t adc_ress = 0;
    char msg_lcd[10];
    float temp = 0.0;
    
    TRISA=0x01;
    TRISB=0x00;
    TRISC=0x00;
    adc_set();
    adc_ch(0);
    lcd_set();
    sprintf(msg_lcd, "t0 = %d  \n", 0);
    __delay_ms(55);
    lcd_adr(0,0);
    lcd_sendstr(msg_lcd);
    while(1){
        adc_ress = adc_read();
        temp =(((adc_ress * 5.0)/1024.0) - 0.5) / 0.01;
        
        sprintf(msg_lcd, "t0 = %.2f  \n", temp);
        __delay_ms(55);
        lcd_adr(0,0);
        lcd_sendstr(msg_lcd);
        __delay_ms(100);
        __delay_ms(100);
    }
    
    return;
}
