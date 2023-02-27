#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

#include "hardware/spi.h"
#include "nrf24l01.h"

#define led_pin     25

uint8_t status = 0, data_in = 0;
void gpio_callback(uint gpio, uint32_t events) {
    status = gpio;
    return;
}

void pins_init(void){
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
  
    gpio_init(SPI_CSN_PIN);
    gpio_set_dir(SPI_CSN_PIN, GPIO_OUT);
	gpio_put(SPI_CSN_PIN, 1);
  
    gpio_init(PIN_CE);
    gpio_set_dir(PIN_CE, GPIO_OUT);
	gpio_put(PIN_CE, 0);

    gpio_init(PIN_DRDY);
    gpio_set_dir(PIN_DRDY, GPIO_IN);
     
    spi_init(spi1, 2 * 500 * 1000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(SPI_RX_PIN, SPI_TX_PIN, SPI_SCK_PIN, GPIO_FUNC_SPI));

    gpio_put(led_pin, 0);
}

void main(void){
	nrf24l01_config_t radio_config = {
		.reg_config = MASK_MAX_RT | EN_CRC | CRC | PRIM_RX_PRX,
		.reg_en_aa = ENAA_P1,
		.reg_en_rxaddr = ERX_P1 | ERX_P0,
		.reg_setup_aw = SETUP_AW_5_bytes,
		.reg_setup_retr = (0 << ARD) | (2 << ARC),
		.reg_rf_chann = 2,
		.reg_rf_setup = RF_PWR_0 | RF_DR_1M,
		.reg_rx_addr_p0[0] = 0xe7,
		.reg_rx_addr_p0[1] = 0xe7,
		.reg_rx_addr_p0[2] = 0xe7,
		.reg_rx_addr_p0[3] = 0xe7,
		.reg_rx_addr_p0[4] = 0xe7,
		.reg_rx_addr_p1[0] = 0xc2,
		.reg_rx_addr_p1[1] = 0xc2,
		.reg_rx_addr_p1[2] = 0xc2,
		.reg_rx_addr_p1[3] = 0xc2,
		.reg_rx_addr_p1[4] = 0xc2,
		.reg_tx_addr[0] = 0xe7,
		.reg_tx_addr[1] = 0xe7,
		.reg_tx_addr[2] = 0xe7,
		.reg_tx_addr[3] = 0xe7,
		.reg_tx_addr[4] = 0xe7,
		.reg_rx_pw_p0 = 0,
		.reg_rx_pw_p1 = 32
	};

    board_init();
    tusb_init(); 
	pins_init();
	
    gpio_set_irq_enabled_with_callback(PIN_DRDY, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

	gpio_put(PIN_CE, 0);
	nrf24l01_config(radio_config);

    nrf24l01_power_on();
	
    nrf24l01_read_reg(CONFIG);
    nrf24l01_read_reg(EN_AA);
    nrf24l01_read_reg(EN_RXADDR);
    nrf24l01_read_reg(SETUP_AW);
    nrf24l01_read_reg(SETUP_RETR);
    nrf24l01_read_reg(RF_CH);
    nrf24l01_read_reg(RF_SETUP);
	
    nrf24l01_read_reg(RX_PW_P0);
    nrf24l01_read_reg(RX_PW_P1);

	gpio_put(PIN_CE, 1);

	while(1){

	}
}
