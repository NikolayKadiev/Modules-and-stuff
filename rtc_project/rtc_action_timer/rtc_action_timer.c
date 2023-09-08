
// PIC16F648A Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define _XTAL_FREQ 12000000


    uint8_t segments[14] = {0x81, 0xb7, 0xc2, 0x92, 0xb4, 0x98, 0x88, 0xb3, 0x80, 0x90, 0xc9, 0xc8, 0xcc, 0xa0};

// Set up USRT for use as master
// See datasheet for more info
void usrt_master(void){ 	
    TRISB2 = 1; // Use appropriate pin of your pic
	TRISB1 = 1; // Use appropriate pin of your pic
	SPBRG=200;	// See datasheet for value
    TXSTA=0xB0;		
	RCSTA=0x80;		
}

// Send data
void usrt_write(uint8_t data){
  TXREG = data;
  while(!TRMT){};
}

void long_delay(uint8_t tim){
    for(uint8_t i = 0; i < tim; i++){
        __delay_ms(100);
    }
}

void set_time(uint8_t *tim){
        for(uint8_t c = 0; c < 4; c++){
            usrt_write(segments[*(tim+c)]);
            for( ; ; ){
                if(RA3 == 0){
                    __delay_ms(100);
                    if(RA3 == 0){
                        *(tim+c) += 1;
                        if(*(tim+0) > 2){
                            *(tim+0) = 0;
                        }
                        if(*(tim+1) > 9){
                            *(tim+1) = 0;
                        }
                        if(*(tim+2) > 5){
                            *(tim+2) = 0;
                        }
                        if(*(tim+3) > 9){
                            *(tim+3) = 0;
                        }
                        usrt_write(segments[*(tim+c)]);
                        while(1){
                            if(RA3 == 1){
                                __delay_ms(100);
                                if(RA3 == 1){
                                    break;
                                }
                            }
                        }
                    }
                }
                if(RA4 == 0){
                    __delay_ms(100);
                    if(RA4 == 0){
                        while(1){
                            if(RA4 == 1){
                                __delay_ms(100);
                                if(RA4 == 1){
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }
        return;
}

uint8_t set_min(void){
    uint8_t min = 0;
    usrt_write(segments[min]);
        for( ; ; ){
            if(RA3 == 0){
                __delay_ms(100);
                if(RA3 == 0){
                    min += 1;
                    if(min > 9){
                        min = 0;
                    }
                    usrt_write(segments[min]);
                    while(1){
                        if(RA3 == 1){
                            __delay_ms(100);
                            if(RA3 == 1){
                                break;
                            }
                        }
                    }
                }
            }
            if(RA4 == 0){
                __delay_ms(100);
                if(RA4 == 0){
                    while(1){
                        if(RA4 == 1){
                            __delay_ms(100);
                            if(RA4 == 1){
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }
    
    return min;
}

void main(void) {
    uint8_t clock[4] = {0, 0, 0, 0};
    uint8_t cli = 0, to_min1 = 0, min1 = 0, start2[4] = {0, 0, 0, 0}, end2[4] = {0, 0, 0, 0}, cc = 0, active = 0;
    CMCON=0x07;
    TRISA=0x1c;
    TRISB=0xc0;
    RB3 = 1;
    RB5 = 0;
    RB4 = 0;
    usrt_master();
    long_delay(10);
    
    if( RA2 == 0){
        set_time(clock);
        
        usrt_write(segments[10]);
        long_delay(30);
        
        set_time(start2);

        usrt_write(segments[11]);
        long_delay(30);
        
        set_time(end2);

        usrt_write(segments[12]);
        long_delay(30);
        
        to_min1 = set_min();
    }
    
    else{
        end2[0] = 1;
        end2[1] = 2;
        end2[2] = 0;
        end2[3] = 0;
    }
    
    if((clock[0] >= start2[0]) && (clock[1] >= start2[1]) && (clock[2] >= start2[2]) && (clock[3] >= start2[3])){
        RB5 = 1;
        RB4 = 0;
        active = 0;
    }
    else{
        RB5 = 0;
        RB4 = 0;
        active = 1;
    }
    
    TMR1=0x0000;
    T1CON = 0x0a;
    __delay_ms(100);
    T1CONbits.TMR1ON = 1;
    PIR1bits.TMR1IF = 0;
    
    while(1){
        for(uint8_t a = 0; a < 4; a ++){
            if((a == 2) && (cli == 7)){
                break;
            }
            RB3 = 1;
            if((a == 0) || (a == 1)){
                usrt_write(segments[clock[a]] & 0x7f);   
            }
            else{
                usrt_write(segments[clock[a]]);
            }
//            long_delay(20);
            while(PIR1bits.TMR1IF == 0){
                NOP();
            }
            PIR1bits.TMR1IF = 0;
            RB3 = 0;
        }
        cli ++;
        if(cli == 8){
            cli = 0;
            clock[3] += 1;
            if(active == 1){
                min1 ++;
            }
            if(clock[3] == 10){
                clock[3] = 0;
                clock[2] += 1;
            }
            if(clock[2] == 6){
                clock[2] = 0;
                clock[1] += 1;
            }
            if(clock[1] == 10){
                clock[1] = 0;
                clock[0] += 1;
            }
            if((clock[0] == 2) && (clock[1] == 4)){
                clock[0] = 0;
                clock[1] = 0;
                clock[2] = 0;
                clock[3] = 0;  
            }
            if((min1 == to_min1) && (active == 1)){
                min1 = 0;
                cc ++;
                if(cc & 0x01){
                    RB4 = 1;
                }
                else{
                    RB4 = 0;
                }
            }
            if((clock[0] == start2[0]) && (clock[1] == start2[1]) && (clock[2] == start2[2]) && (clock[3] == start2[3])){
                RB5 = 1;
                RB4 = 0;
                active = 0;
                min1 = 0;
            }
            if((clock[0] == end2[0]) && (clock[1] == end2[1]) && (clock[2] == end2[2]) && (clock[3] == end2[3])){
                RB5 = 0;
                active = 1;
            }
        }
    }
    return;
}