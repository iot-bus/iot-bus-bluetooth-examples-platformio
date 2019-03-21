#include <arduino.h>
#include "NFCReader.h"

class GlucoseMeter{
    public:
        GlucoseMeter();
        bool ready();
        float reading();
    
    private:
        NFCReader* nfc;

        byte FirstRun = 1;
        int noDiffCount = 0;

        // sensor values
        int sensorMinutesElapsed;
        float lastGlucose;
        float trend[16];
        float validTrend[16];
        int glucosePointer;

        float trendOneGlucose;
        float trendTwoGlucose;

        float currentGlucose = 0;
        float shownGlucose;
        float averageGlucose = 0;

        float glucoseReading(unsigned int val);
        float update(String& trendValues);
        void analyzeTrendData(String& trendValues);
        String convertToString(uint8_t* buffer, uint8_t length);
        float readMemory();

};