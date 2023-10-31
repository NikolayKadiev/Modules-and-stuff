
#include "nrf24l01.h"


#define HEX_CHARS      "0123456789ABCDEF"

#define FIRST_TX  0

uint8_t nRF24_payload[32];
uint8_t nRF24_msg[] ={"NRF24\n"};

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
  
	printf("\r\nESP32 as TX-RX is online.\r\n");
	set_pins();
	Delay_ms(500);
	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+
	printf("nRF24L01+ check: ");
	if (!nRF24_Check()) {
		printf("FAIL\r\n");
		while (1) {
			// printf("FAIL\r\n");
			Delay_ms(50);
		}
	}
	printf("OK\r\n");

	// Initialize the nRF24L01 to its default state
	nRF24_Init();

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

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

    // Configure TX PIPE
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR_TX); // program TX address
    // Configure RX PIPE#1
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR_RX); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

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
		nRF24_CE_L();
		nRF24_SetOperationalMode(nRF24_MODE_TX);
		nRF24_CE_H();
		// Print a payload
		printf("PAYLOAD:> %s < ... TX: \n", (char *)nRF24_msg);

		// Transmit a packet
		tx_res = nRF24_TransmitPacket(nRF24_msg, payload_length);
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
	#endif

    while (1) {
        //
        // Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
        //
        // This is far from best solution, but it's ok for testing purposes
        // More smart way is to use the IRQ pin :)
        //
    
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
            // Get a payload from the transceiver
            pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();
            // Print a payload contents to UART
            printf("RCV PIPE# %d\n", pipe);
            printf(" PAYLOAD:>%s\n", (char *)nRF24_payload);
            printf("<\r\n");

			Delay_ms(500);

			nRF24_CE_L();
			nRF24_SetOperationalMode(nRF24_MODE_TX);
			nRF24_CE_H();
			// Print a payload
			printf("PAYLOAD:> %s < ... TX: \n", (char *)nRF24_payload);
			// Transmit a packet
			tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
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

        }
		else{
			Delay_ms(10);
		}
    }
    
}
