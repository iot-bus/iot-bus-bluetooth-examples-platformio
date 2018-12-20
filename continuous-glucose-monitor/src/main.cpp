#include <BLEDevice.h>
#include <arduino.h>
#include "bluetooth_definitions.h"
#include "bluetooth_time.h"
#include <time.h>
#include <WiFi.h>
#include <SPI.h>
#include "NFCReader.h"
#include "BLEPeripheral.h"
#include "bluetooth_cgm.h"

const char *ssid     = "NETGEAR96";
const char *password = "phoebe1984";

NFCReader* nfc;
BLEPeripheral* bluetooth;

// ntp time 
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3600*8;
const int   daylightOffset_sec = 0;

// glucose reading
float glucoseReading = 0;

bool deviceConnected;

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
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
  Serial.println(" CONNECTED");

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  // Disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // initialize bluetooth
  MyServerCallbacks *pCallbacks = new MyServerCallbacks();
  bluetooth = new BLEPeripheral();
  bluetooth->start(pCallbacks);
  
  // setup reader
  NFC_Setup();

  // interrupt driven reading and sending
  interruptSetup(); // to handle measurement and sending bluetooth data
  handleSecondInterrupt(); // set an initial glucose reading of zero
}

// this is what is returned as a reading if sensor has not been started or started and not ready
#define SENSORSTARTUP 62C2000000000000

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

ble_cgms_meas_t rec;

void handleSecondInterrupt(){

  static int minutes;

  if(!nfc->CR95HF){
    Serial.println("No CR95HF detected");
  }
  else{
    Serial.println(glucoseReading);
    //if (deviceConnected){
      // update the bluetooth characteristics   

      memset(&rec, 0, sizeof(ble_cgms_rec_t));
      Serial.println("Device connected");

      glucoseReading = 110;

      rec.glucose_concentration                 = BLEPeripheral::quick_SFLOAT_from_float(glucoseReading);
      rec.sensor_status_annunciation.warning    = 0;
      rec.sensor_status_annunciation.calib_temp = 0;
      rec.sensor_status_annunciation.status     = 0;
      rec.flags                                 = 0;
      rec.time_offset                           = minutes;
      
      bluetooth->pCGM_MEASUREMENT_Characteristic->setValue((uint8_t*) &rec, sizeof(rec));  // and update the heart rate measurement characteristic
      bluetooth->pCGM_MEASUREMENT_Characteristic->notify();

      int batteryLevel = 50;
      bluetooth->pBatteryCharacteristic->setValue(batteryLevel);  // and update the heart rate measurement characteristic
      bluetooth->pBatteryCharacteristic->notify();
      minutes++;
   // }
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

  // Set alarm to call isr function every minute (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer2, 1000000*60, true);

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
  
  if(!nfc->CR95HF)
    return;
  // read sensor
  if (nfc->NFCReady == 0)
  {
    nfc->SetProtocol_Command(); // ISO 15693 settings
    delay(100);
  }
  else if (nfc->NFCReady == 1)
  {
    for (int i=0; i<3; i++) {
      nfc->Inventory_Command(); // sensor in range?
      if (nfc->NFCReady == 2)
        break;
      delay(1000);
    }
    if (nfc->NFCReady == 1) {
      //goToSleep (0b100001, SLEEP_TIME);
      //wakeUp();
      delay(100); 
    }
  }
  else
  {
    glucoseReading = nfc->Read_Memory();
    delay(100);
  }
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

void NFC_Setup(){

  nfc = new NFCReader();
  bool ready = nfc->EchoResponse();
  if(ready){
    Serial.println("NFC Ready");
  }
  else{
    Serial.println("NFC Not Ready");
  }

  nfc->IDN_Command();
}

