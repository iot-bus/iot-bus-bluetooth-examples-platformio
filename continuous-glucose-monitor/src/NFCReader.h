#include <arduino.h>
#include <SPI.h>

#define MAX_NFC_READTRIES 10 // Amount of tries for every nfc block-scan

class NFCReader{
    private:
        SPIClass *vspi;

        byte RXBuffer[24];
        byte FirstRun = 1;
        int noDiffCount = 0;
        int sensorMinutesElapse;
        float lastGlucose;
        float trend[16];
        static const int spiClk = 1000000; // 1 MHz
        const int SSPin = 5;  // Slave Select pin
        const int IRQPin = 32;  // Slave Select pin

    public:

        byte NFCReady = 0;  // used to track NFC state
        bool CR95HF = false; 

        NFCReader();
        bool wakeUp();
        float Glucose_Reading(unsigned int val);
        void SetProtocol_Command();
        void Inventory_Command();
        float Read_Memory();
        void IDN_Command();
        bool EchoResponse();

};