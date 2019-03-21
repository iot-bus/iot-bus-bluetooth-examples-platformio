#include "NFCReader.h"

#include "soc/spi_struct.h"

struct spi_struct_t {
  spi_dev_t * dev;
  #if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
  #endif
  uint8_t num;
};

NFCReader::NFCReader(){

    // initialize SPI
    pinMode(SSPin, OUTPUT);
    Serial.println(SSPin);  
    vspi = new SPIClass(VSPI); // initialize SPI
    vspi->begin();
    spi_t * _spi;
    _spi= vspi->bus();
    _spi->dev->ctrl2.miso_delay_mode = 2; // this is required as timing is out if we don't use delay mode
    digitalWrite(SSPin, HIGH);
}



void NFCReader::wakeUp(){
  // wake up the CR95HF
  pinMode(IRQPin, OUTPUT);
  digitalWrite(IRQPin, HIGH); 
  delay(10);                      // send a wake up
  digitalWrite(IRQPin, LOW);      // pulse to put the 
  delayMicroseconds(100);         // BM019 into SPI
  digitalWrite(IRQPin, HIGH);     // mode 
  delay(10);
  //digitalWrite(IRQPin, LOW);
}

bool NFCReader::setProtocolCommand() {

  // Set 15693 Mode
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SSPin, LOW);
  vspi->transfer(0x00);  // SPI control byte to send command to CR95HF
  vspi->transfer(0x02);  // Set protocol command
  vspi->transfer(0x02);  // length of data to follow
  vspi->transfer(0x01);  // code for ISO/IEC 15693
  vspi->transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);
 
  while(!pollReady()){
    // wait until we receive ready bit
  }

  read(RXBuffer, sizeof(RXBuffer));

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))  // is response code good?
  {
    Serial.println("Protocol set to 15693");
    status = READY; // NFC is ready
    return true;
  }
  else
  {
    Serial.println("Protocol NOT set");
    status = NOTREADY; // NFC not ready
    return false;
  }
}

bool NFCReader::inventoryCommand() {

  // See if we have a sensor in range (able to read it?)
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SSPin, LOW);
  vspi->transfer(0x00);  // SPI control byte to send command to CR95HF
  vspi->transfer(0x04);  // Send Receive CR95HF command
  vspi->transfer(0x03);  // length of data that follows is 0
  vspi->transfer(0x26);  // request Flags byte
  vspi->transfer(0x01);  // Inventory Command for ISO/IEC 15693
  vspi->transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);
 
  while(!pollReady()){
    // wait until we receive ready bit
  }
  
  read(RXBuffer, sizeof(RXBuffer));
  
  if (RXBuffer[0] == 128)  // is response code good?
  {
    Serial.println("Sensor in range ... OK");
    status = INRANGE;
    return true;
  }
  else
  {
    Serial.println("Sensor out of range");
    status = READY;
  }
  return false;
 }

bool NFCReader::idnCommand()
{
  // step 1 send the command
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  vspi->transfer(0);  // SPI control byte to send command to CR95HF
  vspi->transfer((uint8_t)0x01);  // IDN command
  vspi->transfer(0);  // length of data that follows is 0
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);

  // step 2, poll for data ready
  // data is ready when a read byte
  // has bit 3 set (ex:  B'0000 1000')
  while(!pollReady()){
    // wait until we receive ready bit
  }
  
  // step 3, read the data
  read(RXBuffer, sizeof(RXBuffer));
  
  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 15))
  {  
    CR95HF = true;
    Serial.println("IDN COMMAND-");  //
    Serial.print("RESPONSE CODE: ");
    Serial.print(RXBuffer[0]);
    Serial.print(" LENGTH: ");
    Serial.println(RXBuffer[1]);
    Serial.print("DEVICE ID: ");
    Serial.println((char*)&RXBuffer[2]);
    // for(i=2;i<(RXBuffer[1]);i++)
    // {
    //   Serial.print(RXBuffer[i], HEX);
    //   Serial.print(" ");
    // }
    Serial.println(" ");
    Serial.print("ROM CRC: ");
    Serial.print(RXBuffer[RXBuffer[1]],HEX);
    Serial.print(RXBuffer[RXBuffer[1]+1],HEX);
    Serial.println(" ");
    return true;
  }
  else{
    Serial.println("BAD RESPONSE TO IDN COMMAND!");
    Serial.print("Response: ");
    Serial.println(RXBuffer[0], HEX);
    Serial.print("Length: ");
    Serial.println(RXBuffer[1]);
  }

  Serial.println(" ");
  return false;
}

#define   ECHO            0x55

bool NFCReader::echo()
{
  // send the command
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  vspi->transfer(0);      // SPI control byte to send command to CR95HF
  vspi->transfer(0x55);   // Echo mode
  //vspi->transfer(0);    // No length for echo
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);

  // poll
  while(!pollReady()){
    // wait until we receive ready bit
  }

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  vspi->transfer(0x02);
  byte res = vspi->transfer(0);
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  
  Serial.print("Echo response: 0x");
  Serial.println(res, HEX);
  if (res == ECHO)
  {
    return true;
  }
 return false;
}

bool NFCReader::pollReady(){
  // Poll for data ready
  // data is ready when a read byte
  // has bit 3 set (ex:  B'0000 1000')
  uint8_t data = 0x00;
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  while(data != 8)
  {
    data = vspi->transfer((uint8_t)0x03);  // Write 3 until
    data = data & (uint8_t)0x08;           // bit 3 is set
  }
  digitalWrite(SSPin,HIGH);
  vspi->endTransaction();
  delay(20);
  return true;
}

uint8_t NFCReader::read(uint8_t* buffer, uint8_t length){
  // read up to length bytes into buffer
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SSPin, LOW);
  vspi->write(0x02);   // SPI control byte for read         
  buffer[0] = vspi->transfer(0);  // response code
  buffer[1] = vspi->transfer(0);  // length of data
  for (byte i=0; i < RXBuffer[1] && i < length; i++)      
      buffer[i+2] = vspi->transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);
  return RXBuffer[1];
}

bool NFCReader::readBlock(int block, uint8_t* buffer, uint8_t length){
  // read one block
  bool readError = false;
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  vspi->transfer(0x00);  // SPI control byte to send command to CR95HF
  vspi->transfer(0x04);  // Send Receive CR95HF command
  vspi->transfer(0x03);  // length of data that follows
  vspi->transfer(0x02);  // request Flags byte
  vspi->transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  vspi->transfer(block);     // memory block address
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);

  while(!pollReady()){
    // wait for poll to complete
  }

  read(RXBuffer, sizeof(RXBuffer));
  if (RXBuffer[0] != 128)
      readError = true; 

  memset(buffer, 0x00, length); 
  for (int i = 0; i < 8; i++)
    buffer[i] = RXBuffer[i+3];
    
  return readError;
}


