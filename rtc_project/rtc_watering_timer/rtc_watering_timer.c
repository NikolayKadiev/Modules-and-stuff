
#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define _XTAL_FREQ 4000000

void main(void) {
    CMCON=0x07;
    TRISA=0x1c;
    TRISB=0xc0;
    
    TMR1=0x0000;
    T1CON = 0x0a;
    __delay_ms(100);
    T1CONbits.TMR1ON = 1;
    PIR1bits.TMR1IF = 0;
    
    while(1){
    }
    return;
}
