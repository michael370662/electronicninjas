#include <SPI.h>

/*NRF24L01 Macros*/

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD      0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

#define CE 9
#define CSN 10

#define PAYLOAD_WIDTH 5

#define BUZZER_PIN 7

typedef uint8_t uchar;

//we read from the register given the address and the values are placed into data
//size specifies the number of bytes to read from the register
void readRegister(uchar address, uchar* data, uchar size)
{
  //start the command
  digitalWrite(CSN, LOW);

  //send the the command R_REGISTER | address
  SPI.transfer(R_REGISTER | (address & 0x1F));
  
  //when we write to the MOSI, at the same time, the device writes to MISO
  //the MISO value is returned to us by SPI.transfer
  //the NOP stands for No Operation so the device won't react to the NOP value
  for(uint8_t i = 0; i < size; i++)
    data[i] = SPI.transfer(NOP);

  //done reading, set CSN back to HIGH
  digitalWrite(CSN,HIGH);
}


//the values in "data" are written to register of the given "address"
//size specifies the number of bytes in data
void writeRegister(uchar address, uchar* data, uchar size)
{
    //start the command by setting CSN low
    digitalWrite(CSN, LOW);
  
  //send the command W_REGISTER | address
    SPI.transfer(W_REGISTER | (address & 0x1F));
  
  //write the bytes on MOSI
  //we are not reading any values so the return values of SPI.transfer() is discarded
    for(int i = 0; i < size; i++)
      SPI.transfer(data[i]);
  
  //done with command, set CSN back to HIGH
    digitalWrite(CSN, HIGH);
}

//This is used to enable the data pipe indexed by pipeNum
void enableDataPipe(uint8_t pipeNum)
{
  uchar val;
  //read the initial value
  readRegister(EN_RXADDR, &val, 1);
  //toggle the bit indexed by pipeNum
  val |= (1 << pipeNum);
  //write the new value back in
  writeRegister(EN_RXADDR, &val, 1);
}

//This is used to send a single byte command
void sendCommand(uchar cmd)
{
  //start the command
  digitalWrite(CSN, LOW);
  //send the command
  SPI.transfer(cmd);
  //end the command
  digitalWrite(CSN, HIGH);
}

//Takes n bytes address from addrData and sets the TX_ADDR
void setTxAddress(uchar* addrData, uint8_t n)
{
  //simply use writeRegister to write the addrData into TX_ADDR
  writeRegister(TX_ADDR, addrData, n);
}

void setRxP0Address(uchar* addrData, uint8_t n)
{
  //set the address in RX_ADDR_P0 
  writeRegister(RX_ADDR_P0, addrData, n);
}

//This is used to check if there is data in the RX FIFO
//If there is data, read it into buffer
//Returns true if there was data to be read and false if the RX FIFO was empty
//Number of bytes to read should correspond with the payload width set during initialization
bool readRxFifo(uchar* buffer, uchar numBytes)
{
  bool hasData = false;
  uchar val;
  
  readRegister(FIFO_STATUS, &val, 1);
  //check if RX_EMPTY == 0 in FIFO_STATUS
  if(!(val & (1 << RX_EMPTY)))
  {
    //temporarily stand by while we read the RX information
    stopListening();
    //start the command
    digitalWrite(CSN, LOW);
    //tell the device that we want to read from the FIFO
    SPI.transfer(R_RX_PAYLOAD);
    for(uint8_t i = 0; i < numBytes; i++)
    {
      //read the values from the device
      buffer[i] = SPI.transfer(NOP);
    }

    //end the command
    digitalWrite(CSN, HIGH);

    //start listening again
    startListening();

    hasData = true;
  }
  
  return hasData;
}
void writeTxFifo(uchar* buffer, uchar numBytes)
{
  //start the command
  digitalWrite(CSN, LOW);
  //tell the device that we want to write into the TX FIFO
  SPI.transfer(W_TX_PAYLOAD);
  
  //write the bytes into the TX_FIFO
  for(uint8_t i = 0; i < numBytes; i++)
    SPI.transfer(buffer[i]);

  //finish the command
  digitalWrite(CSN, HIGH);

  //pull CE high to send through the air
  digitalWrite(CE, HIGH);

  //spec says to delay for at least 10us
  //we are delaying by 10ms
  delay(10);

  //pull CE back to LOW after sending
  digitalWrite(CE,LOW);
}

//this function is used to set the device to RX mode
void setRxMode()
{
  uchar val;
  //read the initial CONFIG value
  readRegister(CONFIG, &val, 1);
  
  //toggle PRIM_RX bit to 1 to set RX mode
  val |= (1 << PRIM_RX);
  
  //write new value back to CONFIG
  writeRegister(CONFIG, &val, 1);
}

//this function is used to set the device to TX mode
void setTxMode()
{
  uchar val;
  //read the initial CONFIG value
  readRegister(CONFIG, &val, 1);
  
  //clear PRIM_RX bit to 0 to set TX mode
  val &= ~(1 << PRIM_RX);
  
  //write new value back to CONFIG
  writeRegister(CONFIG, &val, 1);
}

//this is used to clear the MAX_RT bit 
void clearMaxRt(void)
{
  uchar val;
  readRegister(STATUS, &val, 1);
  
  //if the MAX_RT bit is set, we write a 1 to clear the bit. 
  //since it's already a 1, we simply rewrite the same value back 
  //along with the rest of the bits
  if(val & (1 << MAX_RT))
    writeRegister(STATUS, &val, 1);
}

void powerOn()
{
  uchar val;

  //read the initial CONFIG value
  readRegister(CONFIG, &val, 1);

  //toggle the PWR_UP bit 1 to power up the module
  val |= (1 << PWR_UP);
  
  //write the new CONFIG value back
  writeRegister(CONFIG, &val, 1);
}

void stopListening()
{
  digitalWrite(CE, LOW);
  delay(10);
}

void startListening()
{
  digitalWrite(CE, HIGH);
  delay(10);
}

void setup()
{
  pinMode(BUZZER_PIN, OUTPUT);
  
  //used to hold temporary values
  uchar val; 

  //initialize the SPI communication
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);

  //set the CE and CSN pins to outputs
  pinMode(CE, OUTPUT);
  pinMode(CSN, OUTPUT);

  //chip stand by
  digitalWrite(CE, LOW);
  //chip not selected
  digitalWrite(CSN, HIGH);

  //set the module to RX mode
  setRxMode();
  
  //set the RX address to 0x0403020100
  uchar addr[5] = {0x00, 0x01, 0x02, 0x03, 0x04}; 
  setRxP0Address(addr, 5);
  
  //enable the data pipe 0
  enableDataPipe(0);

  //set payload width to 5 bytes
  val = PAYLOAD_WIDTH;
  writeRegister(RX_PW_P0, &val, 1);

  //power on
  powerOn();
  
  //begin to receive from the air
  startListening();
  
  //clear the rx fifo before we begin
  sendCommand(FLUSH_RX);
}

void loop()
{

  uchar val;
  uchar buf[PAYLOAD_WIDTH] = {0};
  
  int currDistance = 0;
  unsigned long currMillis = millis();
  unsigned long prevMillis = currMillis;
  unsigned long duration = currMillis - prevMillis;
  unsigned int buzzerDelayMs = currDistance * 5;
  bool validDistance = false;
  bool buzzerState = LOW;
  //set up another infinite loop to avoid using global variables
  while(1)
  {
    currMillis = millis();
    duration = currMillis - prevMillis;

    //read from the RXFIFO if there is data available
    if(readRxFifo(buf, PAYLOAD_WIDTH))
    {
      //update the delay
      currDistance = *((unsigned int*)buf);
      buzzerDelayMs = currDistance * 5;
      validDistance = true;
    }
    //if we received no data and it has been more than 500ms
    else if(duration > 500)
        validDistance = false;

    if(validDistance)
    {
      if(currDistance < 20)
        digitalWrite(BUZZER_PIN, HIGH);
      else if(currDistance > 200)
        digitalWrite(BUZZER_PIN, LOW);
      else if(duration > buzzerDelayMs)
      {
        buzzerState = !buzzerState;
        prevMillis = currMillis;
        digitalWrite(BUZZER_PIN, buzzerState);
      }
    }
    else
      digitalWrite(BUZZER_PIN, LOW);
  }
}
