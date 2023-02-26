#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define NRF24L01 0
#define NRF24L01P 1

#define NRF_IN_USE NRF24L01

// Commands for the module
#define R_register 0x00
#define W_register 0x20
#define R_RX_pyload 0x61
#define W_TX_payload 0xA0
#define Flush_Tx 0xE1
#define Flush_Rx 0xE2
#define Reuse_Tx_pl 0xE3
#define NOP 0xFF
#if (NRF_IN_USE == NRF24L01P)
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8
#define W_TX_PAYLOAD_NOACK 0xB0
#endif

//Regiter address for the module
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#if (NRF_IN_USE == NRF24L01P)
#define DYNPD 0x1C
#define FEATURE 0x1D
#endif

// Bit masks for the module
//config reg
#define MASK_RX_DR 0x06
#define MASK_TX_DS 0x05
#define MASK_MAX_RT 0x04
#define EN_CRC 0x03
#define CRC 0x02
#define PWR_UP 0x01
#define PRIM_RX_PTX 0x00
#define PRIM_RX_PRX 0x01
//en_aa
#define ENAA_P5 0x20 
#define ENAA_P4 0x10
#define ENAA_P3 0x08 
#define ENAA_P2 0x04 
#define ENAA_P1 0x02 
#define ENAA_P0 0x01 
//en_rxaddr
#define ERX_P5 0x20
#define ERX_P4 0x10
#define ERX_P3 0x08
#define ERX_P2 0x04
#define ERX_P1 0x02
#define ERX_P0 0x01
//setup_aw
#define SETUP_AW_3_bytes 0x01
#define SETUP_AW_4_bytes 0x02
#define SETUP_AW_5_bytes 0x03
//setup_retr
#define ARD 4
#define ARC 0
//rf_setup
#if (NRF_IN_USE == NRF24L01P)
#define RF_DR_250k 0x20
#define CONT_WAVE 0x80
#endif
#define PLL_LOCK 0x10
#define RF_DR_1M 0x00
#define RF_DR_2M 0x08
#define RF_PWR_18 0x00
#define RF_PWR_12 0x02
#define RF_PWR_6 0x04
#define RF_PWR_0 0x06
#define LNA_HCURR 0x01
//status
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4 
#define RX_P_NO 1
#define TX_FULL 0
//observe_tx
#define PLOS_CNT 4
#define ARC_CNT 0 
//fifo_status
#define FIFO_TX_REUSE 6
#define FIFO_TX_FULL 5 
#define FIFO_TX_EMPTY 4 
#define FIFO_RX_FULL 1 
#define FIFO_RX_EMPTY 0

#if (NRF_IN_USE == NRF24L01P)
//dynpd
#define DPL_P5 0x20
#define DPL_P4 0x10
#define DPL_P3 0x08
#define DPL_P2 0x04
#define DPL_P1 0x02
#define DPL_P0 0x01
//feature
#define EN_DPL 0x04
#define EN_ACK_PAYd 0x02
#define EN_DYN_ACK 0x01
#endif

//struct for module
typedef struct {
    uint8_t reg_config;
    uint8_t reg_en_aa;
    uint8_t reg_en_rxaddr;
    uint8_t reg_setup_aw;
    uint8_t reg_setup_retr;
    uint8_t reg_rf_chann;
    uint8_t reg_rf_setup;
    uint8_t reg_rx_addr_p0[5];
    uint8_t reg_rx_addr_p1[5];
    uint8_t reg_tx_addr[5];
    uint8_t reg_rx_pw_p0;
    uint8_t reg_rx_pw_p1;
    #if (NRF_IN_USE == NRF24L01P)
    uint8_t reg_dynpd;
    uint8_t reg_feature;
    #endif
} nrf24l01_config_t;

//functions for the module
void nrf24l01_write_reg(uint8_t reg_addr, uint8_t reg_value);
uint8_t nrf24l01_read_reg(uint8_t reg_addr);
uint8_t nrf24l01_get_pl(void);
uint8_t nrf24l01_get_pl_len(uint8_t pipe);
void nrf24l01_write_pl(uint8_t *buf, uint8_t lenght);
void nrf24l01_read_pl(uint8_t *buf, uint8_t lenght);
void nrf24l01_write_addr(uint8_t reg_addr, uint8_t *buf);
void nrf24l01_power_on(void);
void nrf24l01_power_off(void);
void nrf24l01_set_ptx(void);
void nrf24l01_set_prx(void);
void nrf24l01_config(nrf24l01_config_t conf_data);
