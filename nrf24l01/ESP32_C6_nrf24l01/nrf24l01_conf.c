#include "nrf24l01_conf.h"

spi_device_handle_t spi;

void set_pins(void){
    gpio_reset_pin(PIN_SCK);
    gpio_reset_pin(PIN_MOSI);
    gpio_reset_pin(PIN_MISO);
    gpio_reset_pin(PIN_CSN);
    gpio_reset_pin(PIN_CE);
    gpio_reset_pin(PIN_IRQ);

    gpio_set_direction(PIN_CSN, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CSN, 1);
    gpio_set_direction(PIN_CE, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CE, 1);
    gpio_set_direction(15, GPIO_MODE_OUTPUT);
	gpio_set_level(15, 0);
    
    gpio_set_direction(PIN_IRQ, GPIO_MODE_INPUT);

    spi_bus_config_t buscfg={
            .miso_io_num=PIN_MISO,
            .mosi_io_num=PIN_MOSI,
            .sclk_io_num=PIN_SCK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz=150,
    };

    spi_device_interface_config_t devcfg={
            .clock_source = SPI_CLK_SRC_XTAL,
            .clock_speed_hz= 1*1000*1000,           
            .mode=0,                          
            .spics_io_num=-1,               
            .queue_size=150,
    };	

    printf("bus init %d\n", spi_bus_initialize(SPI2_HOST, &buscfg, 0));
    printf("device add %d\n",spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
}

void nRF24_CE_L(void) {
    gpio_set_level(PIN_CE, 0);
}

void nRF24_CE_H(void) {
    gpio_set_level(PIN_CE, 1);
}

void nRF24_CSN_L(void) {
    gpio_set_level(PIN_CSN, 0);
}

void nRF24_CSN_H(void) {
    gpio_set_level(PIN_CSN, 1);
}

uint8_t nRF24_LL_RW(uint8_t data) {
    spi_transaction_t txrx;
    memset(&txrx, 0, sizeof(txrx));

    txrx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    txrx.tx_buffer = NULL;
    txrx.length = 1*8;
    txrx.tx_data[0] = data;
    txrx.rxlength = 1*8;

	spi_device_polling_transmit(spi, &txrx);
	return txrx.rx_data[0];
}

void Delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
