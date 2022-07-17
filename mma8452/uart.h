#include <xc.h>
#include <stdint.h>

void uart_set();

void uart_send(uint8_t art_s);

char uart_read(void);

void uart_sendline(uint8_t *x, uint8_t len);

void uart_sendstr(uint8_t *x);