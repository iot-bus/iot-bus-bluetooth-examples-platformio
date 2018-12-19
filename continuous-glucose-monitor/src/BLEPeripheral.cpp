#include "BLEPeripheral.h"
#include "bluetooth_cgm.h"

void BLEPeripheral::start(BLEServerCallbacks* callbacks){
    // Initialie Bluetooth
  std::string name = "";
  name += "IoT-Bus CGM";
  BLEDevice::init(name);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(callbacks);
  
  // Setup service 
  BLEPeripheral::pService = pServer->createService((uint16_t)BLE_UUID_CGM_SERVICE);  
  pCGM_MEASUREMENT_Characteristic = pService->createCharacteristic(
                                          (uint16_t) BLE_UUID_CGM_MEASUREMENT,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );
//   pCGM_FEATURE_Characteristic = pService->createCharacteristic(
//                                           (uint16_t) BLE_UUID_CGM_FEATURE,
//                                           BLECharacteristic::PROPERTY_READ |
//                                           BLECharacteristic::PROPERTY_NOTIFY
//                                         );

//   pCGM_STATUS_Characteristic = pService->createCharacteristic(
//                                           (uint16_t) BLE_UUID_CGM_STATUS,
//                                           BLECharacteristic::PROPERTY_READ |
//                                           BLECharacteristic::PROPERTY_NOTIFY
//                                         );

//   pCGM_SESSION_START_TIME_Characteristic = pService->createCharacteristic(
//                                           (uint16_t) BLE_UUID_CGM_SESSION_START_TIME,
//                                           BLECharacteristic::PROPERTY_READ |
//                                           BLECharacteristic::PROPERTY_NOTIFY
//                                         );  
//   pCGM_SESSION_RUN_TIME_Characteristic = pService->createCharacteristic(
//                                           (uint16_t) BLE_UUID_CGM_SESSION_RUN_TIME,
//                                           BLECharacteristic::PROPERTY_READ |
//                                           BLECharacteristic::PROPERTY_NOTIFY
//                                         );  
  
//   pCGM_SPECIFIC_OPS_CTRLPT_Characteristic = pService->createCharacteristic(
//                                           (uint16_t) BLE_UUID_CGM_SPECIFIC_OPS_CTRLPT,
//                                           BLECharacteristic::PROPERTY_READ |
//                                           BLECharacteristic::PROPERTY_NOTIFY
//                                        );  
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
  pAdvertising->addServiceUUID((uint16_t)BLE_UUID_CGM_SERVICE);
  pAdvertising->addServiceUUID((uint16_t)BLE_UUID_BATTERY_SERVICE);
  advert.setName("IoT-Bus CGM");
  pAdvertising->setAdvertisementData(advert);
  pAdvertising->start();

  Serial.println("Bluetooth device active, waiting for connections...");
}

// static uint8_t cgms_meas_encode(ble_cgms_t            * p_cgms,
//                                 const ble_cgms_meas_t * p_meas,
//                                 uint8_t               * p_encoded_buffer)
// {
//     uint8_t len = 2;

//     uint8_t flags = p_meas->flags;

//     len += uint16_encode(p_meas->glucose_concentration,
//                          &p_encoded_buffer[len]);
//     len += uint16_encode(p_meas->time_offset,
//                          &p_encoded_buffer[len]);

//     if (p_meas->sensor_status_annunciation.warning != 0)
//     {
//         p_encoded_buffer[len++] = p_meas->sensor_status_annunciation.warning;
//         flags                  |= NRF_BLE_CGMS_STATUS_FLAGS_WARNING_OCT_PRESENT;
//     }

//     if (p_meas->sensor_status_annunciation.calib_temp != 0)
//     {
//         p_encoded_buffer[len++] = p_meas->sensor_status_annunciation.calib_temp;
//         flags                  |= NRF_BLE_CGMS_STATUS_FLAGS_CALTEMP_OCT_PRESENT;
//     }

//     if (p_meas->sensor_status_annunciation.status != 0)
//     {
//         p_encoded_buffer[len++] = p_meas->sensor_status_annunciation.status;
//         flags                  |= NRF_BLE_CGMS_STATUS_FLAGS_STATUS_OCT_PRESENT;
//     }

//     // Trend field
//     if (p_cgms->feature.feature & NRF_BLE_CGMS_FEAT_CGM_TREND_INFORMATION_SUPPORTED)
//     {
//         if (flags & NRF_BLE_CGMS_FLAG_TREND_INFO_PRESENT)
//         {
//             len += uint16_encode(p_meas->trend, &p_encoded_buffer[len]);
//         }
//     }

//     // Quality field
//     if (p_cgms->feature.feature & NRF_BLE_CGMS_FEAT_CGM_QUALITY_SUPPORTED)
//     {
//         if (flags & NRF_BLE_CGMS_FLAGS_QUALITY_PRESENT)
//         {
//             len += uint16_encode(p_meas->quality, &p_encoded_buffer[len]);
//         }
//     }

//     p_encoded_buffer[1] = flags;
//     p_encoded_buffer[0] = len;
//     return len;
// }

/**
 * @brief A very quick conversion between a float value and 11073-20601 FLOAT-Type.
 * @param value The value as a float.
 * @return Thevalue in 11073-20601 FLOAT-Type format.
 */
uint32_t BLEPeripheral::quick_ieee11073_from_float(float value)
{
  uint8_t  exponent = 0xFF; //exponent is -1 - only works for this range of values
  uint32_t mantissa = (uint32_t)(value*10);
    
  return ((exponent << 24) & 0xFF000000) | ((mantissa <<  0) & 0x00FFFFFF);
}

uint16_t BLEPeripheral::quick_SFLOAT_from_float(float value)
{
  uint8_t  exponent = 0xFF; //exponent is -1 - only works for this range of values
  uint32_t mantissa = (uint32_t)(value*10);
    
  return ((exponent << 12) & 0xF000) | ((mantissa <<  0) & 0x0FFF);
}

