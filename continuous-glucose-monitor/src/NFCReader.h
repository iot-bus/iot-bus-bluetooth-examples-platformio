#ifndef NFCREADER_H_
#define NFCREADER_H_

#include <arduino.h>
#include <SPI.h>

/* CR95HF Commands */
#define IDN               0x01  // identification number of CR95HF
#define SELECT_PROTOCOL   0x02  // select protocol
#define POLL              0x03  // poll
#define SENDRECEIVE       0x04  // send and receive data (most commonly used)
#define READ              0x05  // read values from registers internal to CR95HF
#define WRITE             0x06  // write values to registers internal to CR95HF
#define ECHO              0x55

// send receive commands for ISO/IEC 15693 protocol
#define INVENTORY               0x01  // receives information about tags in range
#define STAY_QUIET              0x02  // selected unit will not send back a response
#define READ_BLOCK              0x20  // read single block of memory from RF tag
#define WRITE_BLOCK             0x21  // write single block to memory of RF tag
#define LOCK_BLOCK              0x22  // permanently locks a block of memory on RF tag
#define READ_BLOCKS             0x23  // reads multiple blocks of memory from RF tag
#define WRITE_BLOCKS            0x24  // writes multiple blocks of memory to RF tag
#define SELECT                  0x25  // used to select a specific tag for communication via the uid
#define RESET_TO_READY          0x26  // resets RF tag to ready state
#define WRITE_AFI               0x27  // writes application family identifier to RF tag
#define LOCK_AFI                0x28  // permanently locks application family identifier
#define WRITE_DSFID             0x29  // writes data storage format identifier to RF tag
#define LOCK_DSFID              0x2A  // permanentlylocks data storage format identifier
#define GET_SYSTEM_INFORMATION  0x2B  // gets information from RF tag that includes memory
// block size in bytes and number of memory blocks
#define GET_BLOCKS_SECURITY_STATUS  0x2C

#define MAX_NFC_READTRIES 10 // Amount of tries for every nfc block-scan

enum Status {NOTREADY, READY, INRANGE};

class NFCReader{
    private:
        SPIClass *vspi;

        byte RXBuffer[24];
        
        static const int spiClk = 1000000; // 1 MHz
        // const int SSPin = 4;  // Slave Select pin
        // const int IRQPin = 14;  // interrupt pin

        const int SSPin = 5;  // Slave Select pin
        const int IRQPin = 14;  // interrupt pin

        bool pollReady();

    public:

        Status status = NOTREADY;  // used to track NFC state
        bool CR95HF = false; 

        NFCReader();
        void wakeUp();
        bool setProtocolCommand();
        bool inventoryCommand();
        uint8_t read(uint8_t* buffer, uint8_t length);
        bool readBlock(int block, uint8_t* buffer, uint8_t length);
        bool idnCommand();
        bool echo();

};

#endif