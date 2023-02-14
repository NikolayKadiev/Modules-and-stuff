#include "nrf24l01.h"

void nrf24l01_write_reg(uint8_t reg_addr, uint8_t reg_value){
    uint8_t to_send[2];
    to_send[0] = W_register | reg_addr;
    to_send[1] = reg_value;
    spi_write_blocking(spi1, to_send, 2);
}

uint8_t nrf24l01_read_reg(uint8_t reg_addr){
    uint8_t to_send[2], to_receive[2];
    to_send[0] = R_register | reg_addr;
    to_send[1] = NOP;
    spi_write_read_blocking(spi1, to_send, to_receive, 2);
    return to_receive[1];
}

uint8_t nrf24l01_get_pl(void){
    uint8_t to_send, to_receive;
    to_send = NOP;
    spi_write_read_blocking(spi1, &to_send, &to_receive, 1);
    return (to_receive >> 1) & 7;
}

uint8_t nrf24l01_get_pl_len(uint8_t pipe){
    return (nrf24l01_read_reg(0x11+pipe) & 0x3F);
}

void nrf24l01_write_pl(uint8_t *buf, uint8_t lenght){
    // uint8_t to_send = W_TX_payload;
    // spi_write_blocking(spi1, &to_send, 1);
    spi_write_blocking(spi1, buf, lenght);
}

void nrf24l01_read_pl(uint8_t *buf, uint8_t lenght){
    uint8_t to_send = R_RX_pyload;
    spi_write_read_blocking(spi1, &to_send, buf, lenght);
}

void nrf24l01_write_addr(uint8_t reg_addr, uint8_t *buf){
    uint8_t to_send[6];
    to_send[0] = W_register | reg_addr;
    to_send[1] = *(buf+0);
    to_send[2] = *(buf+1);
    to_send[3] = *(buf+2);
    to_send[4] = *(buf+3);
    to_send[5] = *(buf+4);
    spi_write_blocking(spi1, to_send, 6);
}

void nrf24l01_power_on(void){
    uint8_t trans;
    trans = nrf24l01_read_reg(CONFIG);
    trans |= 1 << PWR_UP;
    nrf24l01_write_reg(CONFIG, trans);
}

void nrf24l01_power_off(void){
    uint8_t trans;
    trans = nrf24l01_read_reg(CONFIG);
    trans &= ~(1 << PWR_UP);
    nrf24l01_write_reg(CONFIG, trans);
}

void nrf24l01_set_ptx(void){
    uint8_t trans;
    trans = nrf24l01_read_reg(CONFIG);
    trans |=  PRIM_RX_PRX;
    nrf24l01_write_reg(CONFIG, trans);
}

void nrf24l01_set_prx(void){
    uint8_t trans;
    trans = nrf24l01_read_reg(CONFIG);
    trans &=  ~(PRIM_RX_PTX);
    nrf24l01_write_reg(CONFIG, trans);
}

void nrf24l01_config(nrf24l01_config_t conf_data){
    nrf24l01_write_reg(CONFIG, conf_data.reg_config);
    nrf24l01_write_reg(EN_AA, conf_data.reg_en_aa);
    nrf24l01_write_reg(EN_RXADDR, conf_data.reg_en_rxaddr);
    nrf24l01_write_reg(SETUP_AW, conf_data.reg_setup_aw);
    nrf24l01_write_reg(SETUP_RETR, conf_data.reg_setup_retr);
    nrf24l01_write_reg(RF_CH, conf_data.reg_rf_chann);
    nrf24l01_write_reg(RF_SETUP, conf_data.reg_rf_setup);
    nrf24l01_write_addr(RX_ADDR_P0, conf_data.reg_rx_addr_p0);
    nrf24l01_write_addr(TX_ADDR, conf_data.reg_tx_addr);
    nrf24l01_write_addr(RX_ADDR_P1, conf_data.reg_rx_addr_p1);
    nrf24l01_write_reg(RX_PW_P0, conf_data.reg_rx_pw_p0);
    nrf24l01_write_reg(RX_PW_P1, conf_data.reg_rx_pw_p1);
    #if (NRF_IN_USE == NRF24L01P)
    nrf24l01_write_reg(DYNPD, conf_data.reg_dynpd);
    nrf24l01_write_reg(FEATURE, conf_data.reg_feature);
    #endif
    nrf24l01_power_on();
}
