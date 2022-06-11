#include "adc_op.h"

// Set up for using the ADC
// See datasheet for more info
void adc_set(void){
    ADCON0=0x80;
    ADCON1=0x0E;
    return;
}

// Read selected channel
uint16_t adc_read(void){
    int ADC_res, RES_temp;
    ADCON0bits.ADON=1;
    __delay_ms(1);
    ADCON0bits.GO=1;
    while(ADCON0bits.GO){
		NOP();
    }
    ADC_res=ADRESH;
    ADC_res=ADC_res<<2;
    RES_temp=ADRESL;
    RES_temp=RES_temp>>6;
    ADC_res=ADC_res+RES_temp;
	return ADC_res;
}

// Select which ADC channel to read
void adc_ch(int n){
    switch(n){
    case 0:
    ADCON0bits.CHS0=0;
    ADCON0bits.CHS1=0;
    ADCON0bits.CHS2=0;
    return;
    case 1:
    ADCON0bits.CHS0=1;
    ADCON0bits.CHS1=0;
    ADCON0bits.CHS2=0;
    return;
    case 2:
    ADCON0bits.CHS0=0;
    ADCON0bits.CHS1=1;
    ADCON0bits.CHS2=0;
    return;
    case 3:
    ADCON0bits.CHS0=1;
    ADCON0bits.CHS1=1;
    ADCON0bits.CHS2=0;
    return;
    case 4:
    ADCON0bits.CHS0=0;
    ADCON0bits.CHS1=0;
    ADCON0bits.CHS2=1;
    return;
    case 5:
    ADCON0bits.CHS0=1;
    ADCON0bits.CHS1=0;
    ADCON0bits.CHS2=1;
    return;
    case 6:
    ADCON0bits.CHS0=0;
    ADCON0bits.CHS1=1;
    ADCON0bits.CHS2=1;
    return;
    case 7:
    ADCON0bits.CHS0=1;
    ADCON0bits.CHS1=1;
    ADCON0bits.CHS2=1;
    return;
}
}