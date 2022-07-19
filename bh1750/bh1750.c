#include "main.h"

uint8_t sensor_adr = 0x23 << 1;

void main(void) {
    uint8_t data_hi = 0, data_lo = 0;
    uint16_t ress = 0; 
    uint8_t buff[100];
    
    TRISA=0x00;
    TRISB=0x00;
    TRISC=0x00;
    
    uart_set();
    i2c_master(100000);
    __delay_ms(100);
    
    while(1){
        
        i2c_start();
        i2c_sendaddr(sensor_adr + 0);
        i2c_write(0x11);
        i2c_stop();
        __delay_ms(180);
        i2c_start();
        i2c_sendaddr(sensor_adr + 1);                   
        data_hi = i2c_read(1);
        data_lo = i2c_read(0);
        i2c_stop();
        
        ress = (data_hi << 8) | data_lo;
        
        sprintf(buff, "light: %d lx\n", ress);

        uart_sendstr(buff);
        __delay_ms(500);
    }
    return;
}