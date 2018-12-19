#include <BLEDevice.h>
#include "bluetooth_definitions.h"
#include "bluetooth_time.h"
#include "bluetooth_cgm.h"

class BLEPeripheral{

public:
    void start(BLEServerCallbacks* callbacks);

    BLECharacteristic *pCGM_MEASUREMENT_Characteristic;
    BLECharacteristic *pCGM_FEATURE_Characteristic;                                     
    BLECharacteristic *pCGM_STATUS_Characteristic;
    BLECharacteristic *pCGM_SESSION_START_TIME_Characteristic;
    BLECharacteristic *pCGM_SESSION_RUN_TIME_Characteristic;
    BLECharacteristic *pCGM_SPECIFIC_OPS_CTRLPT_Characteristic;

    BLECharacteristic *pBatteryCharacteristic;

    static uint16_t quick_SFLOAT_from_float(float value);
    static uint32_t quick_ieee11073_from_float(float value);

private:
    // bluetooth services, characteristics and advertising objects
    BLEServer *pServer = NULL;
    BLEService *pService;
    BLEService *pBatteryService;

    BLEAdvertisementData advert;
    BLEAdvertisementData scan_response;

};