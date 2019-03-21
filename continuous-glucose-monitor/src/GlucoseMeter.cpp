#include "GlucoseMeter.h"

GlucoseMeter::GlucoseMeter(){

  nfc = new NFCReader();
}

bool GlucoseMeter::ready(){
  
  nfc->wakeUp();

  bool ready = nfc->echo();
  Serial.println("After echo");  
  if(ready){
    Serial.println("NFC Ready");
  }
  else{
    Serial.println("NFC Not Ready");
    return false;
  }
  return nfc->idnCommand();
}

float GlucoseMeter::glucoseReading(unsigned int val) {
    
    // convert raw data to glucose reading mg/dl
    int bitmask = 0x0FFF;
    return ((val & bitmask) / 8.5);
}

float GlucoseMeter::readMemory() {

  // Read the sensor memory
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
      readError = nfc->readBlock(b, oneBlock, sizeof(oneBlock));
      if (!readError)       // is response code good?
      { 
        str = convertToString(oneBlock, sizeof(oneBlock));
        // Serial.println(str);
        trendValues += str;
      }
      readTry++;
    } while( (readError) && (readTry < MAX_NFC_READTRIES) );    
  }

  // sensor minutes from block 39
  readTry = 0;
  do {
    readError = false;  
    readError = nfc->readBlock(39, oneBlock, sizeof(oneBlock));
    if (!readError)
      str = convertToString(oneBlock, sizeof(oneBlock));
      //Serial.print("Raw elapsed minutes: ");
      //Serial.println(str);
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
    Serial.println("Read Memory Block Command FAIL");
    readError = 0;
  }
  return 0;
}

float GlucoseMeter::update(String& trendValues){
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
      trend[j] = glucoseReading(strtoul(t.c_str(), NULL ,16));
    }
  
  int validTrendCounter = 0;
  
  Serial.print("lastGlucose: ");Serial.println(lastGlucose);
  for (int i=0; i<16; i++)
  {
    Serial.print("Trend ");Serial.print(i);Serial.print(": ");Serial.println(trend[i]);
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

  
  //NFCReady = 2;
  FirstRun = 0;

  if (noDiffCount > 5)
    return 0;
  else  
    return shownGlucose;
    
}

void GlucoseMeter::analyzeTrendData(String& trendValues){
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
      currentGlucose  = glucoseReading(strtoul(trendNow.c_str(), NULL ,16));
      trendOneGlucose = glucoseReading(strtoul(trendOne.c_str(), NULL ,16));
      trendTwoGlucose = glucoseReading(strtoul(trendTwo.c_str(), NULL ,16));
      
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
    ii++;
  }
  Serial.print("Current glucose : ");
  Serial.println(currentGlucose);
}

String GlucoseMeter::convertToString(uint8_t* buffer, uint8_t length){
  char str[24];
  unsigned char * pin = buffer;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < buffer+8; pout+=2, pin++) {
    pout[0] = hex[(*pin>>4) & 0xF];
    pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
  //Serial.println(str);
  return String(str);
}

float GlucoseMeter::reading(){

  // if not ready set protocol
  if (nfc->status == NOTREADY)
  {
    if(!nfc->setProtocolCommand()){ // ISO 15693 settings
      // return if we could not set protocol
      return 0;
    } 
  }
  if (nfc->status == READY) // sensor is ready
  {
    for (int i=0; i<3; i++) {
      nfc->inventoryCommand(); // sensor in range?
      if (nfc->status == INRANGE)
        break;
      delay(1000);
    }
  }
  if (nfc->status == INRANGE){ // sensor is in range
    return readMemory();
  }
  return -1;
}