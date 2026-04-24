
#include "nrf24l01.h"

#include "driver/i2c.h"
#include "oled_i2c.h"


#define HEX_CHARS      "0123456789ABCDEF"

#define FIRST_TX  1 // 1 - first do Tx and send data, 0 - do  Rx and wait for data

uint8_t nRF24_payload[32];
// uint8_t nRF24_msg[] ={"NRF24\n"};
uint8_t nRF24_msg[10];
uint8_t nRF24_ack[10] = {'A', 'C', 'K', '\n', '\r'};
uint8_t var_send = 0;
char oled_txt[50];

// Pipe number
nRF24_RXResult pipe;

uint32_t i,j,k;

// Length of received payload
uint8_t payload_length;

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
	printf("[%d]", status);

	// Clear pending IRQ flags
    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}


void app_main(void)
{
	uint8_t ack_flag = 0, retry_num = 0;
  
	printf("\r\nESP32 as TX-RX is online.\r\n");

	i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 20;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 19;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100*1000;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_NUM_0 , &conf);
    i2c_driver_install(I2C_NUM_0 , conf.mode, 0, 0, 0);
	
    OLED_Init();

	set_pins();
	Delay_ms(500);
	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+
	printf("nRF24L01+ check: ");
	
	OLED_SetCursor(0,1);
	OLED_DisplayString("nRF24L01+ check: ");

	if (!nRF24_Check()) {
		printf("FAIL\r\n");
		
        OLED_SetCursor(1,1);
        OLED_DisplayString("FAIL\r\n");

		while (1) {
			// printf("FAIL\r\n");
			Delay_ms(50);
		}
	}
	printf("OK\r\n");
	
	OLED_SetCursor(1,1);
	OLED_DisplayString("OK\r\n");
	Delay_ms(500);

	// Initialize the nRF24L01 to its default state
	nRF24_Init();
	Delay_ms(500);

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(100);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_250kbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

	#if FIRST_TX == 0
    static const uint8_t nRF24_ADDR_TX[] = { 0xE7, 0x1C, 0xE3 };
    static const uint8_t nRF24_ADDR_RX[] = { 0xE7, 0x1C, 0x3E };
	#endif

	#if FIRST_TX == 1
    static const uint8_t nRF24_ADDR_TX[] = { 0xE7, 0x1C, 0x3E };
    static const uint8_t nRF24_ADDR_RX[] = { 0xE7, 0x1C, 0xE3 };
	#endif

	nRF24_SetTXPower(nRF24_TXPWR_0dBm);

    // Configure TX PIPE
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR_TX); // program TX address
    // Configure RX PIPE#1
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR_RX); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

	nRF24_SetRXIRQ(); //test for IRQ on pin
    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();

    // The main loop
    j = 0;
    payload_length = 5;
	Delay_ms(500);

	#if FIRST_TX == 1
		var_send = 10;
		nRF24_CE_L();
		nRF24_SetOperationalMode(nRF24_MODE_TX);
		nRF24_CE_H();
	#endif

	retry_num = 0;

    while (1) {


	#if FIRST_TX == 0
	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
			
			gpio_set_level(15, 0);
            // Get a payload from the transceiver
            pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();
            // Print a payload contents to UART
            printf("RCV PIPE# %d\n", pipe);
            printf(" PAYLOAD:>%s\n", (char *)nRF24_payload);
            printf("<\r\n");
			
			sprintf(oled_txt, "Rx: %s\n", (char *)nRF24_payload);
			OLED_SetCursor(3,1);
			OLED_DisplayString(oled_txt);

			gpio_set_level(15, 1);		
			
			var_send ++;
			sprintf((char *) nRF24_msg, "%05d\n", var_send);

			printf("PAYLOAD:> %s < ... TX: \n", (char *)nRF24_msg);
			
			nRF24_CE_L();
			nRF24_SetOperationalMode(nRF24_MODE_TX);
			nRF24_CE_H();

			sprintf(oled_txt, "Sending: %s\n", (char *)nRF24_ack);
			OLED_SetCursor(5,1);
			OLED_DisplayString((char *)oled_txt);

			// Transmit a packet
			// tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
			tx_res = nRF24_TransmitPacket(nRF24_ack, 5);
			switch (tx_res) {
				case nRF24_TX_SUCCESS:
					printf("OK\n");
					break;
				case nRF24_TX_TIMEOUT:
					printf("TIMEOUT\n");
					break;
				case nRF24_TX_MAXRT:
					printf("MAX RETRANSMIT\n");
					break;
				default:
					printf("ERROR\n");
					break;
			}
			printf("\r\n");
			nRF24_CE_L();
			nRF24_SetOperationalMode(nRF24_MODE_RX);
			nRF24_CE_H();
			Delay_ms(10);

        }
		else{
			Delay_ms(100);
		}
	#endif

	#if FIRST_TX == 1
		// Print a payload
		sprintf((char *) nRF24_msg, "%05d\n", var_send);

		printf("PAYLOAD:> %s < ... TX: \n", (char *)nRF24_msg);
		
		sprintf(oled_txt, "Sending: %s\n", (char *)nRF24_msg);
		OLED_SetCursor(3,1);
		OLED_DisplayString((char *)oled_txt);

		// Transmit a packet
		// tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
		tx_res = nRF24_TransmitPacket(nRF24_msg, 5);
		switch (tx_res) {
			case nRF24_TX_SUCCESS:
				printf("OK\n");
				break;
			case nRF24_TX_TIMEOUT:
				printf("TIMEOUT\n");
				break;
			case nRF24_TX_MAXRT:
				printf("MAX RETRANSMIT\n");
				break;
			default:
				printf("ERROR\n");
				break;
		}
		printf("\r\n");
		
		nRF24_CE_L();
		nRF24_SetOperationalMode(nRF24_MODE_RX);
		nRF24_CE_H();
		ack_flag = 0;

		for(uint8_t i = 0; i < 30; i++){
			if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
				gpio_set_level(15, 0);
				// Get a payload from the transceiver
				pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
				// Clear all pending IRQ flags
				nRF24_ClearIRQFlags();
				// Print a payload contents to UART
				printf("RCV PIPE# %d\n", pipe);
				printf(" PAYLOAD:>%s\n", (char *)nRF24_payload);
				printf("<\r\n");
				
				sprintf(oled_txt, "Rx:       %s\n", (char *)nRF24_payload);
				OLED_SetCursor(5,1);
				OLED_DisplayString(oled_txt);

				gpio_set_level(15, 1);	
				ack_flag = 1;
				retry_num = 0;
				break; 	
			}
			Delay_ms(100);
		}
		
		if(ack_flag == 0){
			sprintf(oled_txt, "Rx: FAILED %d      \n", retry_num);
			OLED_SetCursor(5,1);
			OLED_DisplayString(oled_txt);
			retry_num ++;
			if(retry_num == 4){
				retry_num = 0;
				var_send ++;
			}
		}
		
		else{
			var_send ++;
		}
		nRF24_CE_L();
		nRF24_SetOperationalMode(nRF24_MODE_TX);
		nRF24_CE_H();

		Delay_ms(1000);
		
	#endif
    }
    
}