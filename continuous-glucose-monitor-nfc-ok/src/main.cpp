#include <BLEDevice.h>
#include <arduino.h>
#include "bluetooth_definitions.h"
#include "bluetooth_time.h"
#include <time.h>
#include <WiFi.h>
#include <SPI.h>
#include "soc/spi_struct.h"
struct spi_struct_t {
  spi_dev_t * dev;
  #if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
  #endif
  uint8_t num;
};

const char *ssid     = "NETGEAR96";
const char *password = "phoebe1984";

SPIClass * vspi = NULL; // using vspi for BM019

// ntp time 
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3600*8;
const int   daylightOffset_sec = 0;

// NFC Reader
byte NFCReady = 0;  // used to track NFC state
bool CR95HF = false; 

// glucose reading
float glucoseReading = 0;

// bluetooth services, characteristics and advertising objects
BLEServer *pServer = NULL;
BLEService *pService;
BLEService *pBatteryService;

BLECharacteristic *pCGM_MEASUREMENT_Characteristic;
BLECharacteristic *pCGM_FEATURE_Characteristic;                                     
BLECharacteristic *pCGM_STATUS_Characteristic;
BLECharacteristic *pCGM_SESSION_START_TIME_Characteristic;
BLECharacteristic *pCGM_SESSION_RUN_TIME_Characteristic;
BLECharacteristic *pCGM_SPECIFIC_OPS_CTRLPT_Characteristic;

BLECharacteristic *pBatteryCharacteristic;

bool deviceConnected = false;

BLEAdvertisementData advert;
BLEAdvertisementData scan_response;

/**
* @enum Sensor Location.
* @brief Location of sensor on the body.
*/
enum SensorLocation_t {
    LOCATION_ARMPIT = 1,    /*!< Armpit. */
    LOCATION_BODY,          /*!< Body. */
    LOCATION_EAR,           /*!< Ear. */
    LOCATION_FINGER,        /*!< Finger. */
    LOCATION_GI_TRACT,      /*!< GI tract */
    LOCATION_MOUTH,         /*!< Mouth. */
    LOCATION_RECTUM,        /*!< Rectum. */
    LOCATION_TOE,           /*!< Toe. */
    LOCATION_EAR_DRUM,      /*!< Eardrum. */
};

#define TEMPERATURE_UNITS_CELSIUS     0x00
#define TEMPERATURE_UNITS_FAHRENHEIT  0x01
#define LOCATION_PRESENT              0x02
#define DATETIME_PRESENT              0x04

#define LEDPIN 5
#define TEMPERATUREPIN 35

// Define the B-value of the thermistor.
// This value is a property of the thermistor used in the Grove - Temperature Sensor,
// and used to convert from the analog value it measures and a temperature value.
const int B = 3975;

// these variables are volatile because they are used during the interrupt service routine!
volatile int interruptCounter;
volatile int secondInterruptCounter;

// lock
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

float temperature;
uint8_t thermPayload[13] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// forward declarations
void interruptSetup();
void IRAM_ATTR onTimer();
void IRAM_ATTR onSecondTimer();
void handleInterrupt();
void handleSecondInterrupt();
uint32_t quick_ieee11073_from_float(float temperature);
date_time_t getDateTime();
float Glucose_Reading(unsigned int val);
void bluetoothSetup();
void NFC_Setup();
void SetProtocol_Command();
void Inventory_Command();
float Read_Memory();
void IDN_Command();
bool EchoResponse();
bool EchoResponse2();

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Bluetooth server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Connected to central");
          // turn on the LED to indicate the connection:
      digitalWrite(LEDPIN, HIGH);
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Disconnected from central");
      digitalWrite(LEDPIN, LOW);
      deviceConnected = false;
    }
};

// once only setup
void setup() {
  Serial.begin(115200);    // initialize serial communication
  pinMode(LEDPIN, OUTPUT);   // initialize the LED to indicate when a central is connected

  // connect to WiFi to get NTP time
  // WiFi.begin(ssid, password);
  // while ( WiFi.status() != WL_CONNECTED ) {
  //   delay ( 500 );
  //   Serial.print ( "." );
  // }
  // Serial.println(" CONNECTED");

  // Init and get the time
  // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  // printLocalTime();

  // Disconnect WiFi as it's no longer needed
  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  // initialize bluetooth
  //bluetoothSetup();
  
  // setup reader
  NFC_Setup();

  // interrupt driven reading and sending
  interruptSetup(); // to handle measurement and sending bluetooth data
}

void loop() {
  // time to measure blood glucose
  if (interruptCounter > 0) { 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    handleInterrupt();
  } 
  // one second interrupt to send CGM to central
  if (secondInterruptCounter > 0) { 
    portENTER_CRITICAL(&timerMux);
    secondInterruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    handleSecondInterrupt();
  }
}

void bluetoothSetup(){
    // Initialie Bluetooth
  std::string name = "";
  name += "IoT-Bus CGM";
  BLEDevice::init(name);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Setup service 
  pService = pServer->createService((uint16_t)BLE_UUID_CGM_SERVICE);  
  pCGM_MEASUREMENT_Characteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_CGM_MEASUREMENT,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );
  pCGM_FEATURE_Characteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_CGM_FEATURE,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );

  pCGM_STATUS_Characteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_CGM_STATUS,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );

  pCGM_SESSION_START_TIME_Characteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_CGM_SESSION_START_TIME,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );  
  pCGM_SESSION_RUN_TIME_Characteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_CGM_SESSION_RUN_TIME,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );  
  
  pCGM_SPECIFIC_OPS_CTRLPT_Characteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_CGM_SPECIFIC_OPS_CTRLPT,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );  
  // Start the service

  pService->start();

  // This is the battery level service - we just mock up the value being sent
  pBatteryService = pServer->createService((uint16_t)BLE_UUID_BATTERY_SERVICE);  
  pBatteryCharacteristic = pBatteryService->createCharacteristic(
                                          (uint16_t) BLE_UUID_BATTERY_LEVEL_CHAR,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );
  // Start the service
  pBatteryService->start();
  
  // Start advertising both services
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID((uint16_t)BLE_UUID_CGM_SERVICE);
  pAdvertising->addServiceUUID((uint16_t)BLE_UUID_BATTERY_SERVICE);
  advert.setName("IoT-Bus CGM");
  pAdvertising->setAdvertisementData(advert);
  pAdvertising->start();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void handleSecondInterrupt(){

  if(!CR95HF){
    Serial.println("No CR95HF detected");
  }
  else{
    Serial.println(glucoseReading);
    if (deviceConnected){
      // update the bluetooth characteristics   
      
      pCGM_MEASUREMENT_Characteristic->setValue(glucoseReading);  // and update the heart rate measurement characteristic
      pCGM_MEASUREMENT_Characteristic->notify();

      int batteryLevel = 50;
      pBatteryCharacteristic->setValue(batteryLevel);  // and update the heart rate measurement characteristic
      pBatteryCharacteristic->notify();
    }
  }
}

date_time_t getDateTime(){
  // get the local time
  time_t now;
  time(&now);
  Serial.println(now);

  // Convert to BT
  date_time_t dateTime;
  dateTime.seconds = localtime(&now)->tm_sec;
  dateTime.minutes = localtime(&now)->tm_min;
  dateTime.hours = localtime(&now)->tm_hour;
  dateTime.day = localtime(&now)->tm_mday;
  dateTime.month = localtime(&now)->tm_mon;
  dateTime.year = localtime(&now)->tm_year;
  return dateTime;
}

hw_timer_t * timer = NULL;
hw_timer_t * timer2 = NULL;

void interruptSetup(){     
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  
  // Initializes Timer to run the ISR to sample every 2mS as per original Sketch.
  // Attach ISRTr function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call isr function every 2 milliseconds (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 2000, true);

  // Use 2nd timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer2 = timerBegin(1, 80, true);
  
  // Initializes Timer to run the ISR to sample every second.
  // Attach ISRTr function to our timer.
  timerAttachInterrupt(timer2, &onSecondTimer, true);

  // Set alarm to call isr function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer2, 1000000, true);

  // Start both alarms
  timerAlarmEnable(timer);
  timerAlarmEnable(timer2);
   
} // end interruptSetup

// THIS IS THE HW-TIMER INTERRUPT SERVICE ROUTINE. 
// Timer makes sure that we take a reading every 2 miliseconds
void IRAM_ATTR onTimer(){    // triggered when timer fires...
    portENTER_CRITICAL(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL(&timerMux);
} // end onTimer

// THIS IS THE HW-TIMER INTERRUPT SERVICE ROUTINE. 
// Timer makes sure that we send a reading every second
void IRAM_ATTR onSecondTimer(){    // triggered when timer fires...
    portENTER_CRITICAL(&timerMux);
    secondInterruptCounter++;
    portEXIT_CRITICAL(&timerMux);
} // end onTimer

// read the blood glucose level
void handleInterrupt(){
  
  if(!CR95HF)
    return;
  // read sensor
  if (NFCReady == 0)
  {
    SetProtocol_Command(); // ISO 15693 settings
    delay(100);
  }
  else if (NFCReady == 1)
  {
    for (int i=0; i<3; i++) {
      Inventory_Command(); // sensor in range?
      if (NFCReady == 2)
        break;
      delay(1000);
    }
    if (NFCReady == 1) {
      //goToSleep (0b100001, SLEEP_TIME);
      //wakeUp();
      delay(100); 
    }
  }
  else
  {
    //String xdripPacket = Build_Packet(Read_Memory());
    //Send_Packet(xdripPacket);
    //goToSleep (0b100001, SLEEP_TIME);
    //wakeUp();
    glucoseReading = Read_Memory();
    delay(100);
  }
}

static const int spiClk = 500000; // 1 MHz

const int SSPin = 5;  // Slave Select pin
// const int MOSIPin = 23; VSPI default
// const int MISOPin = 19; VSPI default
// const int SCKPin = 18; VSPI default

#define MAX_NFC_READTRIES 10 // Amount of tries for every nfc block-scan

const int IRQPin  = 32; // Sends wake-up pulse for BM019
//const int NFCPin1 = 27; // Power pin BM019
//const int NFCPin2 = 32; // Power pin BM019
//const int NFCPin3 = 33; // Power pin BM019

byte RXBuffer[24];
byte FirstRun = 1;
int noDiffCount = 0;
int sensorMinutesElapse;
float lastGlucose;
float trend[16];

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

void NFC_Setup(){
    pinMode(SSPin, OUTPUT);

    //digitalWrite(SSPin, LOW);
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); 
    delay(10);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(10);
    //digitalWrite(IRQPin, LOW);
    
    // initialize SPI
    vspi = new SPIClass(VSPI); // initialize SPI
    vspi->begin();
    spi_t * _spi;
    _spi= vspi->bus();
    _spi->dev->ctrl2.miso_delay_mode = 2;
    digitalWrite(SSPin, HIGH);

    bool ready = EchoResponse();
    if(ready){
      Serial.println("NFC Ready");
    }
    else{
      Serial.println("NFC Not Ready");
    }

    IDN_Command();
}

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 8.5);
}

void SetProtocol_Command() {

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
 
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = vspi->transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SSPin, LOW);
  vspi->transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = vspi->transfer(0);  // response code
  RXBuffer[1] = vspi->transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();

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

void Inventory_Command() {

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
 
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = vspi->transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SSPin, LOW);
  vspi->transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = vspi->transfer(0);  // response code
  RXBuffer[1] = vspi->transfer(0);  // length of data
  for (byte i=0;i<RXBuffer[1];i++)      
      RXBuffer[i+2] = vspi->transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);
  
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

float Read_Memory() {

  byte oneBlock[8];
  String hexPointer = "";
  String trendValues = "";
  String hexMinutes = "";
  String elapsedMinutes = "";
  float trendOneGlucose;
  float trendTwoGlucose;
  float currentGlucose = 0;
  float shownGlucose;
  float averageGlucose = 0;
  int glucosePointer;
  int validTrendCounter = 0;
  float validTrend[16];
  byte readError = 0;
  int readTry;
  
  for ( int b = 3; b < 16; b++) {
    readTry = 0;
    do {
      readError = 0;  

      vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
      digitalWrite(SSPin, LOW);
      vspi->transfer(0x00);  // SPI control byte to send command to CR95HF
      vspi->transfer(0x04);  // Send Receive CR95HF command
      vspi->transfer(0x03);  // length of data that follows
      vspi->transfer(0x02);  // request Flags byte
      vspi->transfer(0x20);  // Read Single Block command for ISO/IEC 15693
      vspi->transfer(b);  // memory block address
      digitalWrite(SSPin, HIGH);
      vspi->endTransaction();
      delay(1);
    
      vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
      digitalWrite(SSPin, LOW);
      while(RXBuffer[0] != 8)
      {
        RXBuffer[0] = vspi->transfer(0x03);  // Write 3 until
        RXBuffer[0] = RXBuffer[0] & 0x08;    // bit 3 is set
      }
      digitalWrite(SSPin, HIGH);
      vspi->endTransaction();
      delay(1);

      vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
      digitalWrite(SSPin, LOW);
      vspi->transfer(0x02);   // SPI control byte for read         
      RXBuffer[0] = vspi->transfer(0);  // response code
      RXBuffer[1] = vspi->transfer(0);  // length of data
      for (byte i=0;i<RXBuffer[1];i++)
        RXBuffer[i+2] = vspi->transfer(0);  // data
      if (RXBuffer[0] != 128)
          readError = 1;  
      digitalWrite(SSPin, HIGH);
      vspi->endTransaction();
      delay(1);
        
      for (int i = 0; i < 8; i++)
        oneBlock[i] = RXBuffer[i+3];
        
      char str[24];
      unsigned char * pin = oneBlock;
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for(; pin < oneBlock+8; pout+=2, pin++) {
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
      }
      pout[0] = 0;
      if (!readError)       // is response code good?
      { 
        Serial.println(str);
        trendValues += str;
      }
      readTry++;
    } while( (readError) && (readTry < MAX_NFC_READTRIES) );    
  }
  readTry = 0;
  do {
    readError = 0;  

    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
    digitalWrite(SSPin, LOW);
    vspi->transfer(0x00);  // SPI control byte to send command to CR95HF
    vspi->transfer(0x04);  // Send Receive CR95HF command
    vspi->transfer(0x03);  // length of data that follows
    vspi->transfer(0x02);  // request Flags byte
    vspi->transfer(0x20);  // Read Single Block command for ISO/IEC 15693
    vspi->transfer(39);  // memory block address
    digitalWrite(SSPin, HIGH);
    vspi->endTransaction();
    delay(1);
  
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
    digitalWrite(SSPin, LOW);
    while(RXBuffer[0] != 8)
    {
      RXBuffer[0] = vspi->transfer(0x03);  // Write 3 until
      RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
    digitalWrite(SSPin, HIGH);
    vspi->endTransaction();
    delay(1);

    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
    digitalWrite(SSPin, LOW);
    vspi->transfer(0x02);   // SPI control byte for read         
    RXBuffer[0] = vspi->transfer(0);  // response code
    RXBuffer[1] = vspi->transfer(0);  // length of data
    for (byte i=0;i<RXBuffer[1];i++)
      RXBuffer[i+2] = vspi->transfer(0);  // data
    if (RXBuffer[0] != 128)
      readError = 1;  
    digitalWrite(SSPin, HIGH);
    vspi->endTransaction();
    delay(1);
  
    for (int i = 0; i < 8; i++)
      oneBlock[i] = RXBuffer[i+3];
      
    char str[24];
    unsigned char * pin = oneBlock;
    const char * hex = "0123456789ABCDEF";
    char * pout = str;
    for(; pin < oneBlock+8; pout+=2, pin++) {
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
    }
    pout[0] = 0;
    if (!readError)
      elapsedMinutes += str;
    readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );
      
  if (!readError)
    {
      hexMinutes = elapsedMinutes.substring(10,12) + elapsedMinutes.substring(8,10);
      hexPointer = trendValues.substring(4,6);
      sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
      glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);
             
      Serial.println("");
      Serial.print("Glucose pointer: ");
      Serial.print(glucosePointer);
      Serial.println("");
      
      int ii = 0;
      for (int i=8; i<=200; i+=12) {
        if (glucosePointer == ii)
        {
          if (glucosePointer == 0)
          {
            String trendNow = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendOne = trendValues.substring(178,180) + trendValues.substring(176,178);
            String trendTwo = trendValues.substring(166,168) + trendValues.substring(164,166);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
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
          }
          else if (glucosePointer == 1)
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendTwo = trendValues.substring(178,180) + trendValues.substring(176,178);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
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
          }
          else
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(i-22,i-20) + trendValues.substring(i-24,i-22);
            String trendTwo = trendValues.substring(i-34,i-32) + trendValues.substring(i-36,i-34);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
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
          }
        }  

        ii++;
      }
     
     for (int i=8, j=0; i<200; i+=12,j++) {
          String t = trendValues.substring(i+2,i+4) + trendValues.substring(i,i+2);
          trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL ,16));
       }

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

    if (validTrendCounter > 0)
    { 
      for (int i=0; i < validTrendCounter; i++)
         averageGlucose += validTrend[i];
         
      averageGlucose = averageGlucose / validTrendCounter;
      
      if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
         shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      else
         shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value 

    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
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
  else
    {
    Serial.print("Read Memory Block Command FAIL");
    NFCReady = 0;
    readError = 0;
    }
    return 0;
}

void IDN_Command()
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

bool EchoResponse()
{
  // step 1 send the command
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  vspi->transfer(0);    // SPI control byte to send command to CR95HF
  vspi->transfer(0x55); // Echo mode
  //vspi->transfer(0);    // length of data that follows is 0
  digitalWrite(SSPin, HIGH);
  vspi->endTransaction();
  delay(1);

// step 2, poll for data ready
// data is ready when a read byte
// has bit 3 set (ex:  B'0000 1000')
  RXBuffer[0] = 0x00;
  Serial.println("polling");
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
  {
    RXBuffer[0] = vspi->transfer((uint8_t)0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & (uint8_t)0x08;    // bit 3 is set
  }
  Serial.println("Polling ok in echo");
  vspi->endTransaction();
  digitalWrite(SSPin,HIGH);
  delay(20);
  // ready to write
  byte res = 0;

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0)); 
  digitalWrite(SSPin, LOW);
  vspi->transfer(0x02);
  res = vspi->transfer(0);
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

