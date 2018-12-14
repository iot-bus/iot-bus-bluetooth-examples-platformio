#include <BLEDevice.h>
#include <arduino.h>
#include "bluetooth_definitions.h"
#include "bluetooth_time.h"
#include <time.h>
#include <WiFi.h>

const char *ssid     = "NETGEAR96";
const char *password = "phoebe1984";

// ntp time 
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3600*8;
const int   daylightOffset_sec = 0;

// bluetooth services, characteristics and advertising objects
BLEServer *pServer = NULL;
BLEService *pService;
BLEService *pBatteryService;
BLECharacteristic *pCharacteristic;
BLECharacteristic *pLocationCharacteristic;
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
  pinMode(LEDPIN, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

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

  std::string name = "";

  // Initialie Bluetooth
  name += "IoT-Bus HTM";
  BLEDevice::init(name);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Setup service 
  pService = pServer->createService((uint16_t)BLE_UUID_HEALTH_THERMOMETER_SERVICE);  
  pCharacteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR,
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
  pAdvertising->addServiceUUID((uint16_t)BLE_UUID_HEALTH_THERMOMETER_SERVICE);
  pAdvertising->addServiceUUID((uint16_t)BLE_UUID_BATTERY_SERVICE);
  advert.setName("IoT-Bus HTM");
  pAdvertising->setAdvertisementData(advert);
  pAdvertising->start();

  Serial.println("Bluetooth device active, waiting for connections...");

  interruptSetup(); // to handle heartbest pulse measurement
}

void loop() {
  // time to measure temperature
  if (interruptCounter > 0) { 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    handleInterrupt();
  } 
  // one second interrupt to send temperature to central
  if (secondInterruptCounter > 0) { 
    portENTER_CRITICAL(&timerMux);
    secondInterruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    handleSecondInterrupt();
  }
}

void handleSecondInterrupt(){

  if (deviceConnected){
    // update the bluetooth characteristics   
    thermPayload[0] = TEMPERATURE_UNITS_FAHRENHEIT | DATETIME_PRESENT | LOCATION_PRESENT;
    
    uint32_t temp = quick_ieee11073_from_float(temperature);
    memcpy(thermPayload+1, &temp, sizeof(temp));
    
    date_time_t t = getDateTime();
    memcpy(thermPayload+5, &t, sizeof(t));
    
    thermPayload[12] = LOCATION_FINGER;

    pCharacteristic->setValue(thermPayload, sizeof(thermPayload));  // and update the heart rate measurement characteristic
    pCharacteristic->notify();

    int batteryLevel = 50;
    pBatteryCharacteristic->setValue(batteryLevel);  // and update the heart rate measurement characteristic
    pBatteryCharacteristic->notify();
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
// Timer makes sure that we take a reading every second
void IRAM_ATTR onSecondTimer(){    // triggered when timer fires...
    portENTER_CRITICAL(&timerMux);
    secondInterruptCounter++;
    portEXIT_CRITICAL(&timerMux);
} // end onTimer

// read the temperature  - currently mocking the return
void handleInterrupt(){
    // read sensor
    /* Read the current voltage level on the analog input pin.
     This is used here to simulate the heart rate's measurement.
    */
    // Get the (raw) value of the temperature sensor.
    int val = analogRead(TEMPERATUREPIN);

    // Determine the current resistance of the thermistor based on the sensor value.
    float resistance = (float)(1023-val)*10000/val;

    // Calculate the temperature based on the resistance value.
    temperature = 1/(log(resistance/10000)/B+1/298.15)-273.15;
    // no sensor connected !!!!!!!!
    temperature = 96.4;
}

/**
 * @brief A very quick conversion between a float temperature and 11073-20601 FLOAT-Type.
 * @param temperature The temperature as a float.
 * @return The temperature in 11073-20601 FLOAT-Type format.
 */
uint32_t quick_ieee11073_from_float(float temperature)
{
  uint8_t  exponent = 0xFF; //exponent is -1
  uint32_t mantissa = (uint32_t)(temperature*10);
    
  return ((exponent << 24) & 0xFF000000) | ((mantissa <<  0) & 0x00FFFFFF);
}


     