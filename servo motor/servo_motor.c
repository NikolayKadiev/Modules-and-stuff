#include "main.h"

#define OUT_S RB7

void servo_set(){
    CCP1CON=0b00001010;
    RD16=1;
    OUT_S=0;
    OUT_S=1;
    OUT_S=0;
    __delay_ms(100);
    return;
}

void servo_update(uint16_t puls,uint16_t paus){
      OUT_S=1;
      TMR1L=0x0000;
      CCPR1=puls;
      TMR1ON=1;
      while(CCP1IF==0){}
      CCP1IF=0;
      TMR1ON=0;
      OUT_S=0;
      TMR1L=0x0000;
      CCPR1=paus;
      TMR1ON=1;
      while(CCP1IF==0){}
      CCP1IF=0;
      TMR1ON=0;
      return;
}

void main(void) {
    uint16_t pul, pau, per;
    int stepp = 10;
    TRISA=0x00;
    TRISB=0x00;
    TRISC=0x00;
    
    pau=19000;
    pul=1000;
    per=0;
    
    servo_set();
    
  while(1){
      for(int i=0; i<5; i++){
          servo_update(pul,pau);
      }
      pau-=stepp;
      pul+=stepp;
      if(pul == 700){
          stepp = 10;
      }
      if(pul == 2300){
          stepp = -10;
      }
  }
    return;
}

