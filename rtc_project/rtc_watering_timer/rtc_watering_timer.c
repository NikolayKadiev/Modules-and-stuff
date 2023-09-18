// PIC16F648A Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator: Crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = OFF       // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define _XTAL_FREQ 4000000


void long_delay(uint16_t tim){
    for(uint16_t i = 0; i < tim; i++){
        __delay_ms(100);
    }
}


void main(void) {
    uint16_t days = 3, watering = 100;
    
    CMCON=0x07;
    TRISA=0x1c;
    TRISB=0x00;
    RA1 = 1;
    RB3 = 0;
  
    long_delay(10);
    
    if(PORTAbits.RA2  == 0){ //set 3 days
        days = 16200;
        watering = 500;
    }
    
    if(PORTAbits.RA3  == 0){ //set 5 days
        days = 27000;
        watering = 900;
    }
    
    if(PORTAbits.RA4  == 0){ //set 7 days
        days = 37800;
        watering = 1100;
    }
    
    RA1 = 0;
    
    TMR1=0x0000;
    T1CON = 0x3E;
    
    RB3 = 1;
    long_delay(watering);
    RB3 = 0;
        
    INTCONbits.GIE = 0;
    INTCONbits.PEIE = 1;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    T1CONbits.TMR1ON = 1;
    
    while(1){
        
        for(uint16_t a = 0; a < days; a ++){
            SLEEP();
            RB3 = 0;
            PIR1bits.TMR1IF = 0;
        }  
        
        RB3 = 1;
        long_delay(watering);
        RB3 = 0;
        
    }
    return;
}
