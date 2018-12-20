#include "main.h"

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
  setupNFC();

  // interrupt driven reading and sending bluetooth data
  setupInterrupts(); 
}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void setupInterrupts(){     
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  
  // Initializes Timer to run the ISR to sample every 2mS as per original Sketch.
  // Attach ISRTr function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call isr function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 10000000, true);

  // Use 2nd timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer2 = timerBegin(1, 80, true);
  
  // Initializes Timer to run the ISR to sample every second.
  // Attach ISRTr function to our timer.
  timerAttachInterrupt(timer2, &onSecondTimer, true);

  // Set alarm to call isr function every minute (value in microseconds).
  // Repeat the alarm (third parameter) - currently 15 seconds
  timerAlarmWrite(timer2, 1000000*15, true);

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

void handleSecondInterrupt(){

  static int minutes;
  Serial.print("handleSecondInterrupt glucose reading: ");
  Serial.println(glucoseReading);

  if(!nfc->CR95HF){
    Serial.println("No CR95HF detected");
  }
  else{
    if (deviceConnected){
      // update the bluetooth characteristics   

      memset(&rec, 0, sizeof(ble_cgms_rec_t));
      Serial.println("Device connected");

      //glucoseReading = 110;

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

// read the blood glucose level
void handleInterrupt(){
  
  // return if no sensor detected
  if(!nfc->CR95HF)
    return;
  // if not ready set protocol
  if (nfc->NFCReady == 0)
  {
    if(!nfc->setProtocolCommand()){ // ISO 15693 settings
      // return if we could not set protocol
      return;
    } 
  }
  if (nfc->NFCReady == 1) // sensor is ready
  {
    for (int i=0; i<3; i++) {
      nfc->inventoryCommand(); // sensor in range?
      if (nfc->NFCReady == 2)
        break;
      delay(1000);
    }
  }
  if (nfc->NFCReady == 2){ // sensor is in range
    glucoseReading = nfc->readMemory();
  }
}

bool setupNFC(){

  nfc = new NFCReader();

  nfc->wakeUp();

  bool ready = nfc->echo();
  if(ready){
    Serial.println("NFC Ready");
  }
  else{
    Serial.println("NFC Not Ready");
    return false;
  }
  return nfc->idnCommand();
}

