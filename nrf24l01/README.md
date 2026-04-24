# nRF24L01 module testing for microcontroller project

Testing in progress for nRF24L01 module library.

| MCU | Function |
|-----|----------|
|ESP32|Tested/working|
|PIC18F/16F|In testing|
|RPI PICO|In testing|

Made possible thanks to the work of: https://github.com/elmot/nrf24l01-lib

# Things to keep in mind
1. Solder a 100uF cap near the power supply pins of the nRF24L01 module
2. If testin inside, use lower nRF24_SetTXPower; for outeside - use higher nRF24_SetTXPower
