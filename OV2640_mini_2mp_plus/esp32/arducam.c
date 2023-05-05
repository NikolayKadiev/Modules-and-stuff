
#include "arducam.h"
#include "ov2640.h"



// Define sensor slave address
void picoSystemInit(){
    // This example will use I2C
	i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = PIN_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100*1000;
    i2c_param_config(I2C_NUM_1, &conf);
    i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);

     // This example will use SPI
    spi_bus_config_t buscfg={
            .miso_io_num=PIN_MISO,
            .mosi_io_num=PIN_MOSI,
            .sclk_io_num=PIN_SCK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz=100,
    };
    spi_device_interface_config_t devcfg={
            .clock_speed_hz=1000*1000,           
            .mode=0,                          
            .spics_io_num=-1,               
            .queue_size=150,
    };	
    spi_bus_initialize(VSPI_HOST, &buscfg, 0);
    spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CS, 1);

    // Set up our UART with the required speed.
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue,0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void rdSensorReg8_8(uint8_t regID, uint8_t* regDat ){
	i2c_cmd_handle_t cmd_read;
	cmd_read = i2c_cmd_link_create();
	i2c_master_start(cmd_read);
	i2c_master_write_byte(cmd_read, ( arducam.slave_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd_read, regID , 1);
	i2c_master_start(cmd_read);
	i2c_master_write_byte(cmd_read, ( arducam.slave_address << 1) | I2C_MASTER_READ, 1);
	i2c_master_read_byte(cmd_read, regDat, I2C_MASTER_NACK);
	i2c_master_stop(cmd_read);
	i2c_master_cmd_begin(I2C_NUM_1, cmd_read, portMAX_DELAY);
	i2c_cmd_link_delete(cmd_read);
}

 void wrSensorReg8_8(uint8_t regID, uint8_t regDat ){
	i2c_cmd_handle_t cmd_comm;
	cmd_comm = i2c_cmd_link_create();
	i2c_master_start(cmd_comm);
	i2c_master_write_byte(cmd_comm, ( arducam.slave_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd_comm, regID , 1);
	i2c_master_write_byte(cmd_comm, regDat, 1);
	i2c_master_stop(cmd_comm);
	i2c_master_cmd_begin(I2C_NUM_1, cmd_comm, portMAX_DELAY);
	i2c_cmd_link_delete(cmd_comm);
}

 void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_set_level(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

 void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_set_level(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

 void write_reg(uint8_t address, uint8_t value){
    spi_transaction_t tx;
    memset(&tx, 0, sizeof(tx));
    // tx.cmd = address| WRITE_BIT;
    tx.length = 2*8;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.tx_data[0] = address| WRITE_BIT;
    tx.tx_data[1] = value;
    cs_select();
	spi_device_polling_transmit(spi, &tx);
    cs_deselect();
    vTaskDelay(10 / portTICK_PERIOD_MS); 
}

 uint8_t read_reg(uint8_t address)
 {
    spi_transaction_t txrx;
    memset(&txrx, 0, sizeof(txrx));

    //txrx.cmd = address & 0x7f;
    txrx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    txrx.tx_buffer = NULL;
    txrx.length = 2*8;
    txrx.tx_data[0] = address & 0x7f;
    txrx.rxlength = 2*8;

    cs_select();
	spi_device_polling_transmit(spi, &txrx);
	cs_deselect();
    vTaskDelay(10 / portTICK_PERIOD_MS);
	return txrx.rx_data[1];
}

void wrSensorRegs8_8(const struct sensor_reg reglist[])
{
  unsigned int reg_addr = 0;
  unsigned int reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr =next->reg;
    reg_val = next->val;
    wrSensorReg8_8(reg_addr, reg_val);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    next++;
  }
}

unsigned char read_fifo(void)
{
	unsigned char data;
	data = read_reg(SINGLE_FIFO_READ);
	return data;
}

void set_fifo_burst()
{
    spi_transaction_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.length=8;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.tx_data[0] = BURST_FIFO_READ;
    spi_device_polling_transmit(spi, &tx);
}

void flush_fifo(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void start_capture(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void clear_fifo_flag(void )
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

unsigned int read_fifo_length()
{
    unsigned int len1,len2,len3,len=0;
    len1 = read_reg(FIFO_SIZE1);
    len2 = read_reg(FIFO_SIZE2);
    len3 = read_reg(FIFO_SIZE3) & 0x7f;
    len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return len;	
}

//Set corresponding bit  
void set_bit(unsigned char addr, unsigned char bit)
{
	unsigned char temp;
	temp = read_reg(addr);
	write_reg(addr, temp | bit);
}
//Clear corresponding bit 
void clear_bit(unsigned char addr, unsigned char bit)
{
	unsigned char temp;
	temp = read_reg(addr);
	write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
unsigned char get_bit(unsigned char addr, unsigned char bit)
{
  unsigned char temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}

void OV2640_set_JPEG_size(unsigned char size)
{
	switch(size)
	{
		case res_160x120:
			wrSensorRegs8_8(OV2640_160x120_JPEG);
			break;
		case res_176x144:
			wrSensorRegs8_8(OV2640_176x144_JPEG);
			break;
		case res_320x240:
			wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
		case res_352x288:
	  	wrSensorRegs8_8(OV2640_352x288_JPEG);
			break;
		case res_640x480:
			wrSensorRegs8_8(OV2640_640x480_JPEG);
			break;
		case res_800x600:
			wrSensorRegs8_8(OV2640_800x600_JPEG);
			break;
		case res_1024x768:
			wrSensorRegs8_8(OV2640_1024x768_JPEG);
			break;
		case res_1280x1024:
			wrSensorRegs8_8(OV2640_1280x1024_JPEG);
			break;
		case res_1600x1200:
			wrSensorRegs8_8(OV2640_1600x1200_JPEG);
			break;
		default:
			wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
	}

    flush_fifo();
    clear_fifo_flag();
}

void ov2640Init(){
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x12, 0x80);
    wrSensorRegs8_8(OV2640_JPEG_INIT);
    wrSensorRegs8_8(OV2640_YUV422);
    wrSensorRegs8_8(OV2640_JPEG);
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x15, 0x00);
    wrSensorRegs8_8(OV2640_320x240_JPEG);
}

void singleCapture(void){
   uint8_t * value = NULL;
   uint8_t * tx_empty = NULL;
   int len_out = 0;
   clear_fifo_flag();
   vTaskDelay(10 / portTICK_PERIOD_MS);
   //Start capture
   start_capture(); 
   while(!get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)){
    asm volatile("nop");
   }
   int length = read_fifo_length();
   len_out = length;
   //printf("len = %d\n", length);
   value = (uint8_t *)malloc(sizeof(uint8_t)*(64));
   tx_empty = (uint8_t *)malloc(sizeof(uint8_t)*(64));
   memset(tx_empty, 0, 64);
   spi_transaction_t rx;
   memset(&rx, 0, sizeof(rx));
   rx.length=64*8;
   rx.rxlength=64*8;
   rx.flags = 0;
   rx.tx_buffer = tx_empty;
   rx.rx_buffer = value;
   cs_select();
   set_fifo_burst();//Set fifo burst mode
   while(1){
    spi_device_transmit(spi, &rx);
    uart_write_bytes(UART_NUM_0, value, 64);
    if(len_out < 64){
        rx.length=len_out*8;
        rx.rxlength=len_out*8;
        spi_device_transmit(spi, &rx);
        uart_write_bytes(UART_NUM_0, value, len_out);
        break;
    }
    else{
        len_out -= 64;
    }
   }
   cs_deselect();
   free(value);
   free(tx_empty);
}

uint8_t spiBusDetect(void){
    write_reg(0x00, 0x55);
    if(read_reg(0x00) == 0x55){
        return 0;
    }else{
        return 1;
    }      
}

uint8_t ov2640Probe(){
    uint8_t id_H,id_L;
    rdSensorReg8_8(0x0A,&id_H);
    rdSensorReg8_8(0x0B,&id_L);
    if(id_H == 0x26 && (id_L == 0x40 ||id_L == 0x41 || id_L == 0x42)){
        return 0;
    }else{
        return 1;
    }
}

struct camera_operate arducam = {
    .slave_address = 0x30,
    .systemInit  = picoSystemInit,
    .busDetect   = spiBusDetect,
    .cameraProbe = ov2640Probe,
    .cameraInit  = ov2640Init,
    .setJpegSize = OV2640_set_JPEG_size,
    .takePicture = singleCapture,
};
