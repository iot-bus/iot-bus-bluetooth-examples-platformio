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
    vspi = new SPIClass(VSPI); // initialize SPI
    vspi->begin();
    spi_t * _spi;
    _spi= vspi->bus();
    _spi->dev->ctrl2.miso_delay_mode = 2; // this is required as timing is out if we don't use delay mode
    digitalWrite(SSPin, HIGH);
}


void NFCReader::wakeUp(){

    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); 
    delay(10);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(10);
    //digitalWrite(IRQPin, LOW);
}

float NFCReader::Glucose_Reading(unsigned int val) {
    int bitmask = 0x0FFF;
    return ((val & bitmask) / 8.5);
}

void NFCReader::SetProtocol_Command() {

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
    Serial.println("Protocol Set Command OK");
    NFCReady = 1; // NFC is ready
    }
  else
    {
    Serial.println("Protocol Set Command FAIL");
    NFCReady = 0; // NFC not ready
    }
}

void NFCReader::Inventory_Command() {

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
    NFCReady = 2;
  }
  else
  {
    Serial.println("Sensor out of range");
    NFCReady = 1;
  }
 }

float NFCReader::Read_Memory() {

  byte oneBlock[8];

  byte readError = false;
  int readTry;
  String trendValues = "";
  String elapsedMinutes = "";
  String hexMinutes = "";

  String str;
  // trend values from blocks 3 to 15
  for ( int b = 3; b < 16; b++) {
    readTry = 0;
    do {
      readError = false;
      readError = readBlock(b, oneBlock, sizeof(oneBlock));
      if (!readError)       // is response code good?
      { 
        str = convertToString(oneBlock, sizeof(oneBlock));
        Serial.println(str);
        trendValues += str;
      }
      readTry++;
    } while( (readError) && (readTry < MAX_NFC_READTRIES) );    
  }

  // sensor minutes from block 39
  readTry = 0;
  do {
    readError = false;  
    readError = readBlock(39, oneBlock, sizeof(oneBlock));
    if (!readError)
      str = convertToString(oneBlock, sizeof(oneBlock));
      Serial.print("Raw elapsed minutes: ");
      Serial.println(str);
      elapsedMinutes = str;
    readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );
      
  if (!readError)
  {
    // minutes elapsed
    String hexMinutes = elapsedMinutes.substring(10,12) + elapsedMinutes.substring(8,10);
    sensorMinutesElapsed = strtoul(hexMinutes.c_str(), NULL, 16);
    Serial.print("Minutes elapsed: ");
    Serial.println(sensorMinutesElapsed); 
    // update from trend data
    return update(trendValues);
  }
  else
    {
    Serial.print("Read Memory Block Command FAIL");
    NFCReady = 0;
    readError = 0;
    }
    return 0;
}

float NFCReader::update(String& trendValues){
  // glucose pointer       
  Serial.println("");
  String hexPointer = trendValues.substring(4,6);
  glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);
  Serial.print("Glucose pointer: ");
  Serial.print(glucosePointer);
  Serial.println("");
  
  analyzeTrendData(trendValues);
  
  // analyze trend data
  for (int i=8, j=0; i<200; i+=12,j++) {
      String t = trendValues.substring(i+2,i+4) + trendValues.substring(i,i+2);
      trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL ,16));
      Serial.println(trend[j]);
    }
  
  int validTrendCounter = 0;
  
  for (int i=0; i<16; i++)
  {
    if (((lastGlucose - trend[i]) > 50) || ((trend[i] - lastGlucose) > 50)) // invalid trend check
        continue;
    else
    {
        validTrend[validTrendCounter] = trend[i];
        validTrendCounter++;
    }
  }
  Serial.print("Valid trend counter : ");
  Serial.println(validTrendCounter);
  // calculate average trend
  if (validTrendCounter > 0)
  { 
    for (int i=0; i < validTrendCounter; i++)
        averageGlucose += validTrend[i];
        
    averageGlucose = averageGlucose / validTrendCounter;
    Serial.print("Average glucose : ");
    Serial.println(averageGlucose);
    
    if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
        shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
    else
        shownGlucose = currentGlucose; // All went well. Take and show the current value
  }
  else
    shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value 
  
  // sensor may have expired - this was different in US but maybe the same now
  if ((lastGlucose == currentGlucose) && (sensorMinutesElapsed > 21000)) // Expired sensor check
    noDiffCount++;

  if (lastGlucose != currentGlucose) // Reset the counter
    noDiffCount = 0;

  if (currentGlucose != 0)
    lastGlucose = currentGlucose; 

  
  NFCReady = 2;
  FirstRun = 0;

  if (noDiffCount > 5)
    return 0;
  else  
    return shownGlucose;
    
}

void NFCReader::analyzeTrendData(String& trendValues){
    // 15 min trend values in 8 byte blocks #3 - #15
  int ii = 0;
  // read all 6 bytes BG reading blocks, one block is 12 ASCII
  for (int i=8; i<=200; i+=12) {

    String trendNow;
    String trendOne;
    String trendTwo;

    if (glucosePointer == ii)
    {
      if (glucosePointer == 0)
      {
        trendNow = trendValues.substring(190,192) + trendValues.substring(188,190);
        trendOne = trendValues.substring(178,180) + trendValues.substring(176,178);
        trendTwo = trendValues.substring(166,168) + trendValues.substring(164,166);
      }
      else if (glucosePointer == 1)
      {
        trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
        trendOne = trendValues.substring(190,192) + trendValues.substring(188,190);
        trendTwo = trendValues.substring(178,180) + trendValues.substring(176,178);
      }
      else
      {
        trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
        trendOne = trendValues.substring(i-22,i-20) + trendValues.substring(i-24,i-22);
        trendTwo = trendValues.substring(i-34,i-32) + trendValues.substring(i-36,i-34);
      }
    }  
    currentGlucose  = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
    trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
    trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));
    
    if (FirstRun == 1)
        lastGlucose = currentGlucose;
        
    if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
    {
        if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
          currentGlucose = trendTwoGlucose;
        else
          currentGlucose = trendOneGlucose;
    }
    ii++;
  }
  Serial.print("Current glucose : ");
  Serial.println(currentGlucose);
}

void NFCReader::IDN_Command()
{
  byte i = 0;

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
RXBuffer[0] = 0x00;
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
  {
    RXBuffer[0] = vspi->transfer((uint8_t)0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & (uint8_t)0x08;    // bit 3 is set
  }
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);

// step 3, read the data
  digitalWrite(SSPin, LOW);
  vspi->transfer((uint8_t)0x02);   // SPI control byte for read         
  RXBuffer[0] = vspi->transfer(0);  // response code
  RXBuffer[1] = vspi->transfer(0);  // length of data

  for (i=0; i < RXBuffer[1] ;i++)      
      RXBuffer[i+2] = vspi->transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  delay(1);
  
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
  }
  else
    Serial.println("BAD RESPONSE TO IDN COMMAND!");

  Serial.println(" ");
}

#define   ECHO            0x55

bool NFCReader::EchoResponse()
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

String NFCReader::convertToString(uint8_t* buffer, uint8_t length){
  char str[24];
  unsigned char * pin = buffer;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < buffer+8; pout+=2, pin++) {
    pout[0] = hex[(*pin>>4) & 0xF];
    pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
  Serial.println(str);
  return String(str);
}

