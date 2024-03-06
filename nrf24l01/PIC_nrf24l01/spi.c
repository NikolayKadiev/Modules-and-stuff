#include "spi.h"

// Set up SPI master for pic
// See datasheet for more info
void spi_master(uint8_t mode){
    SSPSTAT=0x00;
    SSPCON1=0x30;
    switch(mode){
        case  0:
            SSPSTAT |= 1<<6;
            SSPCON1 &= ~(1<<4);
            break;
        case  1:
            SSPSTAT &= ~(1<<6);
            SSPCON1 &= ~(1<<4);
            break;
        case  2:
            SSPSTAT |= 1<<6;
            SSPCON1 |= 1<<4;
            break;
        case  3:
            SSPSTAT &= ~(1<<6);
            SSPCON1 |= 1<<4;
            break;
        default: break;
    }
// See datasheet for using TMR2 for clock
//    PR2 = ;
//    T2CON = ;
    TRISC3=0;  // Use appropriate pin of your pic
    TRISC4=1;  // Use appropriate pin of your pic
    TRISC5=0;  // Use appropriate pin of your pic
}

// Set up SPI slave for pic
// See datasheet for more info
void spi_slave(uint8_t mode){
    SSPSTAT=0x00;
    SSPCON1=0x34;
    switch(mode){
        case  0:
            SSPSTAT |= 1<<6;
            SSPCON1 &= ~(1<<4);
            break;
        case  1:
            SSPSTAT &= ~(1<<6);
            SSPCON1 &= ~(1<<4);
            break;
        case  2:
            SSPSTAT |= 1<<6;
            SSPCON1 |= 1<<4;
            break;
        case  3:
            SSPSTAT &= ~(1<<6);
            SSPCON1 |= 1<<4;
            break;
        default: break;
    }
// See datasheet for using TMR2 for clock
//    PR2 = ;
//    T2CON = ;
    TRISC3=1; // Use appropriate pin of your pic
    TRISC4=1; // Use appropriate pin of your pic
    TRISC5=0; // Use appropriate pin of your pic
    TRISA5=1; // Use appropriate pin of your pic
}

// Wait for buffer
void spi_wait(void)
{
    while ( SSPSTATbits.BF==0 ); 
}

// Write data to device
void spi_write(uint8_t dat) 
{   
    SSPBUF = dat;
    spi_wait();
}

// Read data from device
uint8_t spi_read(void)
{
    spi_wait();      
    return SSPBUF; 
}

// Write and read data to/from device
uint8_t spi_transfer(uint8_t out){
    spi_write(out);
    return spi_read();
}
