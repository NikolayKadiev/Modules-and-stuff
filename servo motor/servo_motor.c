#include "main.h"

#define OUT_S PORTBbits.RB7

void servo_set(){
    CCP1CON=0b00001010;
    T1CON = 0x80;
    OUT_S=0;
}

void servo_update(uint16_t puls,uint16_t paus){
      OUT_S=1;
      CCPR1H = puls >> 8;
      CCPR1L = puls & 0xFF;
      TMR1=0x0000;
      T1CONbits.TMR1ON=1;
      while(PIR1bits.CCP1IF==0){}
      PIR1bits.CCP1IF=0;
      T1CONbits.TMR1ON=0;
      OUT_S=0;
      CCPR1H = paus >> 8;
      CCPR1L = paus & 0xFF;
      TMR1=0x0000;
      T1CONbits.TMR1ON=1;
      while(PIR1bits.CCP1IF==0){}
      PIR1bits.CCP1IF=0;
      T1CONbits.TMR1ON=0;
}

void main(void) {
    uint16_t pul, pau, per = 1;
    uint8_t stepp = 25;
    TRISA=0x00;
    TRISB=0x00;
    TRISC=0x00;
    
    pau=20200;
    pul=500;
    
    servo_set();
    
  while(1){
      for(int i=0; i<5; i++){
          servo_update(pul,pau);
      }
      if(pul < 500) {
          per = 0;
      }
      if(pul > 2400) {
          per = 1;
      }
      if(per == 0){
          pau-=stepp;
          pul+=stepp;
      }
      if(per == 1){
          pau+=stepp;
          pul-=stepp;
      }
  }
    return;
}
