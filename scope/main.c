
// PIC12F1822 Configuration Bit Settings

// 'C' source line config statements


#define USE_8 0
#define USE_16 1
#define OSC_USE_V USE_8

#if OSC_USE_V == USE_16
#define _XTAL_FREQ 16000000
#endif

#if OSC_USE_V == USE_8
#define _XTAL_FREQ 8000000
#endif


// CONFIG1
#if OSC_USE_V == USE_16
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#endif
#if OSC_USE_V == USE_8
#pragma config FOSC = XT        // Oscillator Selection (XT Oscillator, Crystal/resonator connected between OSC1 and OSC2 pins)
#endif
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

 void EUSART_Write(uint8_t txData)
{
    while(0 == PIR1bits.TXIF)
    {
    }

    TXREG = txData;    // Write the data byte to the USART.
}
 
 
 void bin_bcd(uint16_t bin_tem, uint8_t *bcd){
	*bcd=0x30;
	*(bcd+1)=0x30;
	*(bcd+2)=0x30;
	*(bcd+3)=0x30;
    *(bcd+4) = '\n';


    if(bin_tem >= 1000){
        *bcd += 1;
        bin_tem = bin_tem - 1000;
    }
    if(bin_tem >= 100){
    	while(bin_tem > 100){
    		*(bcd+1) += 1;
    		bin_tem = bin_tem - 100;
    	}
    }
    if(bin_tem >= 10){
    	while(bin_tem > 10){
    		*(bcd+2) += 1;
    		bin_tem = bin_tem - 10;
    	}
    }

    *(bcd+3) |= bin_tem;

    return;
}
 
void main(void) {
    uint16_t ressult = 0;
    char msg[10];
    uint8_t i =0;
    
    OSCCON = 0b11110010;
    
    TRISA = 0x3F;
    ANSELA = 0x04;
    
    APFCON = 0x00;
    
    ADCON1 = 0xD0;
    ADRESL = 0x00;
    ADRESH = 0x00;
    ADCON0 = 0x01;
    ADCON0bits.CHS = 2;
    
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 1;
    
    BAUDCON = 0x08;
    RCSTA = 0x90;
    TXSTA = 0x24;
#if OSC_USE_V == USE_16
    SPBRGL = 34;    // SPBRG = (_XTAL_FREQ / (4 * Boud)) - 1
#endif
#if OSC_USE_V == USE_8   // SPBRG = (_XTAL_FREQ / (4 * Boud)) - 1
    SPBRGL = 16;
#endif
    
    SPBRGH = 0x00;
    
    char msgx[]= {"Hello\n"};
    
    while(1){
        
    ADCON0bits.ADON = 1;
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE){ }
    ressult = (uint16_t)((ADRESH << 8) + ADRESL);
    
    bin_bcd(ressult, msg);
    
    for(i=0; i<10; i ++){
        EUSART_Write(msg[i]);
        if(msg[i] == '\n'){
            break;
        }
    }
    
    }
    return;
}
