#include <arduino.h>
#include <SPI.h>
#include "soc/spi_struct.h"
struct spi_struct_t {
  spi_dev_t * dev;
  #if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
  #endif
  uint8_t num;
};

// forward declarations
bool EchoResponse();
void NFC_Setup();

byte RXBuffer[128];

// once only setup
void setup() {
  Serial.begin(115200);    // initialize serial communication
  SPI.begin(18, 19, 23, -1);
  spi_t * _spi;
  _spi= SPI.bus();
  _spi->dev->ctrl2.miso_delay_mode = 2;
  SPI.setFrequency(1000000);
  // setup reader
  NFC_Setup();
  bool ready = EchoResponse();
  if(ready){
    Serial.println("NFC Ready");
  }
  else{
    Serial.println("NFC Not Ready");
  }
}

void loop() {

}

/*
    BM019 to IoT-Bus connections
    
    P1 (DOUT) - NC
    P2 (DIN)  - 32
    P3 (SS)   - 5
    P4 (MISO) - 19
    P5 (MOSI) - 23
    P6 (SCK)  - 18
    P7 (SS_0) - Bridge (3V3)
    P8 (VDD)  - Bridge (3v3)
    P9 (VIN)  - VBAT
    P10 (GND) - GND

*/
#define SSpin 5
#define IRQPin 32

void NFC_Setup(){
    pinMode(SSpin, OUTPUT);

    //digitalWrite(SSPin, LOW);
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); 
    delay(10);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(10);
    //digitalWrite(IRQPin, LOW);

}

#define ECHO 0x55

bool EchoResponse()
{
  // write to CR95HF
  digitalWrite(SSpin, LOW);
  delay(100);
  SPI.write(0x00);
  SPI.write(0x55);
  //SPI.transfer(0x00);
  digitalWrite(SSpin, HIGH);
  //delay(1000);
  Serial.println("Sent echo command");

  // poll to CR95HF, is it ready?
  memset(RXBuffer, 0x00, sizeof(RXBuffer));
  digitalWrite(SSpin, LOW);
  while(RXBuffer[0] != 8)
  {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;    // bit 3 is set
  }
  digitalWrite(SSpin, HIGH);
  Serial.println("Polling OK");
  delay(20);

  // ready to read
  byte res = 0;

  digitalWrite(SSpin, LOW);
  //delay(100);
  SPI.write(0x02);
  res = SPI.transfer(0);

  digitalWrite(SSpin, HIGH);
  Serial.print("Response: 0x");
  Serial.println(res, HEX);
  if (res == ECHO)
  {
    Serial.println("Echo OK");
    return true;
  }
 Serial.println("Echo failed"); 
 return false;
}


