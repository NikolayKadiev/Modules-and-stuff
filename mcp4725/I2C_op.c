#include "I2C_op.h"


//Initialize I2C Module as Master
//See datasheet
void i2c_master(const unsigned long I2C_c)
{
  TRISC3 = 1;     // Use appropriate pin of your pic           
  TRISC4 = 1;     // Use appropriate pin of your pic             
  SSPSTAT = 0x80;
  SSPCON1 = 0x28;            
  SSPCON2 = 0;
  SSPADD = (_XTAL_FREQ/(4*I2C_c))-1; //Setting Clock.  Speed See datasheet
}

//For Waiting
//Transmit is in progress
void i2c_wait(void)
{
  while ((SSPSTATbits.BF) || (SSPSTATbits.R_nW));
  SSPIF = 0;
}

//Start Condition
void i2c_start(void)
{    
  SEN = 1;             
  while(SSPCON2bits.SEN);
  SSPIF = 0;
}

//Repieted Start Condition
void i2c_repstart(void)
{    
  RSEN = 1;             
  while(SSPCON2bits.RSEN);
  SSPIF = 0;
}


//Stop Condition
void i2c_stop(void)
{
  i2c_wait();
  PEN = 1;           
  while(PEN);
  SSPIF=0;
}

//Write Data
void i2c_write(uint8_t I2C_d)
{
  SSPBUF = I2C_d;         
  i2c_wait();
}

void i2c_sendaddr(uint8_t adr){
    i2c_write(adr);
}

//Read Data
uint8_t i2c_read(uint8_t I2C_a)
{
  uint8_t I2C_temp;
  RCEN = 1;
  while(!SSPSTATbits.BF);
  I2C_temp = SSPBUF;
  if(I2C_a==1){
    ACKDT = 0;
	ACKEN = 1;
  }
  else{
    ACKDT = 1;			
    ACKEN = 1;
  }
  while(ACKEN);
  i2c_wait();
  return I2C_temp;
}
