
/*
   This sketch example partially implements the standard Bluetooth Low-Energy Heart Rate service.
   For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
*/

#include <BLEDevice.h>
#include <arduino.h>
#include "bluetooth_definitions.h"

// mutex
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// these variables are volatile because they are used during the interrupt service routine!
volatile int interruptCounter;
volatile int secondInterruptCounter;

// the heart rate calculation variables
uint8_t BPM;               // used to hold the pulse rate
int Signal;                // holds the incoming raw data
int IBI = 600;             // holds the time between beats, must be seeded! 
boolean Pulse = false;     // true when pulse wave is high, false when it's low
boolean QS = false;        // becomes true when a beat is found.

// pins used
#define HEARTRATEPIN 34
#define LEDPIN 5

// forward declarations
void interruptSetup();
void onTimer();
void onSecondTimer();
void handleInterrupt();
void handleSecondInterrupt();

// bluetooth objects
BLEServer *pServer = NULL;
BLEService *pService;
BLECharacteristic *pCharacteristic;
BLEAdvertisementData advert;
BLEAdvertisementData scan_response;

// Bluetooth status set by callbacks
bool deviceConnected = false;

// called when central connects or disconnects to/form this peripheral
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// once-only setup
void setup() {

  Serial.begin(115200);    // initialize serial communication

  // initialize pins
  pinMode(HEARTRATEPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);

  std::string name = "";
  name += "IoT-Bus HRM";
  BLEDevice::init(name);
  
  // create server and callbacks
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // create bluetooth heart rate service - we use the standard uuid
  // so anything that can read this service can get it e.g. nrfToolbox app on iPhone and Android
  pService = pServer->createService((uint16_t)BLE_UUID_HEART_RATE_SERVICE);
  // add the standard characteristic
  pCharacteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_HEART_RATE_MEASUREMENT_CHAR,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        ); 
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  // same service
  pAdvertising->addServiceUUID((uint16_t)BLE_UUID_HEART_RATE_SERVICE);
  // same name
  advert.setName("IoT-Bus HRM");
  pAdvertising->setAdvertisementData(advert);
  pAdvertising->start();

  Serial.println("Bluetooth device active, waiting for connections...");

  interruptSetup(); // to handle heartbest pulse measurement and one second time to send bluetooth data

  QS = false; // no heartbeat detected yet
} // end setup


void loop() {
  // 2ms heartbeat measurement interrupt 
  if (interruptCounter > 0) { 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    handleInterrupt();
  }
  // one second interrupt to send heartbeat
  if (secondInterruptCounter > 0) { 
    portENTER_CRITICAL(&timerMux);
    secondInterruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    handleSecondInterrupt();
  }
} // end loop

// timers
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

// Heartbeat measurement
int rate[10];                    // array to hold last ten IBI values
unsigned long sampleCounter = 0; // used to determine pulse timing
unsigned long lastBeatTime = 0;  // used to find IBI
int P = 512;                     // used to find peak in pulse wave, seeded
int T = 512;                     // used to find trough in pulse wave, seeded
int thresh = 512;                // used to find instant moment of heart beat, seeded
int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

// Handle interrupt
void handleInterrupt(){                       // called by main loop to handle interrupt
  Signal = analogRead(HEARTRATEPIN);          // read the Pulse Sensor on pin 34 3.3v sensor power......default ADC setup........
  Signal = map(Signal, 0, 4095, 0, 1023);     // Map the value back to original sketch range......
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

  //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                          // T is the trough
      T = Signal;                             // keep track of lowest point in pulse wave 
    }
  }

  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
    P = Signal;                               // P is the peak
  }                                           // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                              // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
      Pulse = true;                          // set the Pulse flag when we think there is a pulse
      IBI = sampleCounter - lastBeatTime;    // measure time between beats in mS
      lastBeatTime = sampleCounter;          // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        return;                              // IBI value is unreliable so discard it
      }   
      digitalWrite(LEDPIN,HIGH);             // turn on LED

      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      Serial.print("BPM: ");
      Serial.println(BPM);
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }

  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    digitalWrite(LEDPIN,LOW);              // turn off LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }
}

// Send to bluetooth every second if we have QS and connected with central
void handleSecondInterrupt(){
  // if a central is connected to peripheral:
  if (deviceConnected && QS == true) {
    uint8_t heartRateCharArray[2] = { 0, BPM }; 
    Serial.print("Bluetooth BPM: ");
    Serial.println(BPM);
    pCharacteristic->setValue(heartRateCharArray, 2);  // and update the heart rate measurement characteristic
    pCharacteristic->notify();
    QS = false; 
  } 
}// end handleSecondInterrupt