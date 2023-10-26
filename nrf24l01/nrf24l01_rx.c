#include "nrf24l01.h"

#define HEX_CHARS      "0123456789ABCDEF"

uint8_t nRF24_payload[32];
// Pipe number
nRF24_RXResult pipe;
uint32_t i, j, k;
// Length of received payload
uint8_t payload_length;


void app_main(void)
{
    printf("\r\nESP32 is online.\r\n");

    // RX/TX disabled
    nRF24_CE_L();

    // Configure the nRF24L01+
    printf("nRF24L01+ check: ");
    if (!nRF24_Check()) {
        printf("FAIL\r\n");
        while (1) {
            printf("FAIL\r\n");
            Delay_ms(50);
        }
    }

    printf("OK\r\n");

    // Initialize the nRF24L01 to its default state
    nRF24_Init();

/***************************************************************************/
    // This is simple receiver with one RX pipe:
    //   - pipe#1 address: '0xE7 0x1C 0xE3'
    //   - payload: 5 bytes
    //   - RF channel: 115 (2515MHz)
    //   - data rate: 250kbps (minimum possible, to increase reception reliability)
    //   - CRC scheme: 2 byte

    // The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

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

    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();


    // The main loop
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
        }
    }
}
