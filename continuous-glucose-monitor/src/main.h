#include <arduino.h>
#include <time.h>
#include "bluetooth_time.h"
#include <BLEDevice.h>
#include "bluetooth_definitions.h"
#include <WiFi.h>
#include <SPI.h>
#include "NFCReader.h"
#include "BLEPeripheral.h"
#include "bluetooth_cgm.h"
#include "GlucoseMeter.h"

const char *ssid     = "NETGEAR96";
const char *password = "phoebe1984";

// lock
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

GlucoseMeter* glucoseMeter;
BLEPeripheral* bluetooth;

// these variables are volatile because they are used during the interrupt service routine!
volatile int interruptCounter;
volatile int secondInterruptCounter;

ble_cgms_meas_t rec;

hw_timer_t * timer = NULL;
hw_timer_t * timer2 = NULL;

// ntp time 
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3600*8;
const int   daylightOffset_sec = 0;

// this is what is returned as a reading if sensor has not been started or started and not ready
#define SENSORSTARTUP 62C2000000000000

// glucose reading
float glucoseReading = 0;

bool deviceConnected;

#define LEDPIN 12

// forward declarations
void setupInterrupts();
void IRAM_ATTR onTimer();
void IRAM_ATTR onSecondTimer();
void handleInterrupt();
void handleSecondInterrupt();
uint32_t quick_ieee11073_from_float(float temperature);
date_time_t getDateTime();
float Glucose_Reading(unsigned int val);
void bluetoothSetup();
bool setupNFC();
void printLocalTime();