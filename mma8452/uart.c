#include "uart.h"

// Set up UART for use
// See datasheet for more info
void uart_set(){
    TRISC7=1;  // Use appropriate pin of your pic
    TRISC6=0;  // Use appropriate pin of your pic
    SPBRG = 25;  // See datasheet for value
    TXSTA=0x24;		
	RCSTA=0x90;
}

// Send data
void uart_send(uint8_t art_s){
    TXREG = art_s;
    while(!TRMT){};
}

// Read data
char uart_read(void){
    while(!RCIF){};
    return RCREG;
}

// Send a line
void uart_sendline(uint8_t *x, uint8_t len){
	for(uint8_t i = 0; i < len; i++){
		uart_send(*(x+i));
	}
}

// Send a string
void uart_sendstr(uint8_t *x){
	for(uint8_t i=0; *(x + i) != '\n'; i++){
		uart_send(x[i]);
	}
	uart_send('\n');
}