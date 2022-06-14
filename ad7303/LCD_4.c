#include "LCD_4.h"

// Send comand to LCD
void lcd_control (uint8_t comand) {
    __delay_ms(1);
    RS_LCD=0;
    PT_OUT=comand&0xF0;;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    PT_OUT=comand<<0x04;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    return;
}

// Set up LCD for use - 4 wires
void lcd_set(void){
    __delay_ms(100);
    __delay_ms(100);
    __delay_ms(55);
    PT_OUT= 0x30;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    __delay_ms(50);
    
    PT_OUT= 0x30;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    __delay_ms(10);
    
    PT_OUT= 0x30;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    __delay_ms(10);
    
    PT_OUT= 0x20;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    __delay_ms(10);
    
    lcd_control(0x01);
    lcd_control(0x28);
    lcd_control(0x08);
    lcd_control(0x06);
    lcd_control(0x14);
    lcd_control(0x0C);
    __delay_ms(3);
    lcd_control(0x28);
    return;
}

// Send a character to LCD
void lcd_send(char data){
    __delay_ms(1);
    PT_OUT=data&0xF0;
    RS_LCD=1;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    PT_OUT=data<<0x04;
    RS_LCD=1;
    EN_LCD=1;
    NOP();
    EN_LCD=0;
    RS_LCD=0;
    return;
}

// Shift left or right
void lcd_shift(char x){
    lcd_control(0x05);
    switch(x){
        case 'R': lcd_control(0x18);
        break;
        case 'L': lcd_control(0x1C);
        break;
    }
}

//Send a string to LCD
void lcd_sendstr(char str[]){
	for (uint8_t ar = 0; str[ar] != '\n'; ar++){
		lcd_send(str[ar]);
	}
	return;
}

//Send address to LCD
void lcd_adr(uint8_t row, uint8_t col){
   __delay_ms(1);
	if(row==1){
		lcd_control(0xC0 + col);
	}
	else{
		lcd_control(0x80 + col);
	}
	__delay_ms(5);
}
