#include <arduino.h>

#ifndef BLUETOOTH_CGM_H_ 
#define BLUETOOTH_CGM_H_ 

// CGM Feature characteristic defines

#define BLE_CGMS_FEAT_CALIBRATION_SUPPORTED                           (0x01 << 0)  //!< Calibration supported.
#define BLE_CGMS_FEAT_PATIENT_HIGH_LOW_ALERTS_SUPPORTED               (0x01 << 1)  //!< Patient High/Low Alerts supported.
#define BLE_CGMS_FEAT_HYPO_ALERTS_SUPPORTED                           (0x01 << 2)  //!< Hypo Alerts supported.
#define BLE_CGMS_FEAT_HYPER_ALERTS_SUPPORTED                          (0x01 << 3)  //!< Hyper Alerts supported.
#define BLE_CGMS_FEAT_RATE_OF_INCREASE_DECREASE_ALERTS_SUPPORTED      (0x01 << 4)  //!< Rate of Increase/Decrease Alerts supported.
#define BLE_CGMS_FEAT_DEVICE_SPECIFIC_ALERT_SUPPORTED                 (0x01 << 5)  //!< Device Specific Alert supported.
#define BLE_CGMS_FEAT_SENSOR_MALFUNCTION_DETECTION_SUPPORTED          (0x01 << 6)  //!< Sensor Malfunction Detection supported.
#define BLE_CGMS_FEAT_SENSOR_TEMPERATURE_HIGH_LOW_DETECTION_SUPPORTED (0x01 << 7)  //!< Sensor Temperature High-Low Detection supported.
#define BLE_CGMS_FEAT_SENSOR_RESULT_HIGH_LOW_DETECTION_SUPPORTED      (0x01 << 8)  //!< Sensor Result High-Low Detection supported.
#define BLE_CGMS_FEAT_LOW_BATTERY_DETECTION_SUPPORTED                 (0x01 << 9)  //!< Low Battery Detection supported.
#define BLE_CGMS_FEAT_SENSOR_TYPE_ERROR_DETECTION_SUPPORTED           (0x01 << 10) //!< Sensor Type Error Detection supported.
#define BLE_CGMS_FEAT_GENERAL_DEVICE_FAULT_SUPPORTED                  (0x01 << 11) //!< General Device Fault supported.
#define BLE_CGMS_FEAT_E2E_CRC_SUPPORTED                               (0x01 << 12) //!< E2E-CRC supported.
#define BLE_CGMS_FEAT_MULTIPLE_BOND_SUPPORTED                         (0x01 << 13) //!< Multiple Bond supported.
#define BLE_CGMS_FEAT_MULTIPLE_SESSIONS_SUPPORTED                     (0x01 << 14) //!< Multiple Sessions supported.
#define BLE_CGMS_FEAT_CGM_TREND_INFORMATION_SUPPORTED                 (0x01 << 15) //!< CGM Trend Information supported.
#define BLE_CGMS_FEAT_CGM_QUALITY_SUPPORTED                           (0x01 << 16) //!< CGM Quality supported.

// Continuous Glucose Monitoring type

#define BLE_CGMS_MEAS_TYPE_CAP_BLOOD     0x01  //!< Capillary Whole blood.
#define BLE_CGMS_MEAS_TYPE_CAP_PLASMA    0x02  //!< Capillary Plasma.
#define BLE_CGMS_MEAS_TYPE_VEN_BLOOD     0x03  //!< Venous Whole blood.
#define BLE_CGMS_MEAS_TYPE_VEN_PLASMA    0x04  //!< Venous Plasma.
#define BLE_CGMS_MEAS_TYPE_ART_BLOOD     0x05  //!< Arterial Whole blood.
#define BLE_CGMS_MEAS_TYPE_ART_PLASMA    0x06  //!< Arterial Plasma.
#define BLE_CGMS_MEAS_TYPE_UNDET_BLOOD   0x07  //!< Undetermined Whole blood.
#define BLE_CGMS_MEAS_TYPE_UNDET_PLASMA  0x08  //!< Undetermined Plasma.
#define BLE_CGMS_MEAS_TYPE_FLUID         0x09  //!< Interstitial Fluid (ISF).
#define BLE_CGMS_MEAS_TYPE_CONTROL       0x0A  //!< Control Solution.

// CGM sample location

#define BLE_CGMS_MEAS_LOC_FINGER         0x01  //!< Finger.
#define BLE_CGMS_MEAS_LOC_AST            0x02  //!< Alternate Site Test (AST).
#define BLE_CGMS_MEAS_LOC_EAR            0x03  //!< Earlobe.
#define BLE_CGMS_MEAS_LOC_CONTROL        0x04  //!< Control solution.
#define BLE_CGMS_MEAS_LOC_SUB_TISSUE     0x05  //!< Subcutaneous tissue.
#define BLE_CGMS_MEAS_LOC_NOT_AVAIL      0x0F  //!< Sample Location value not available.

// CGM Measurement Sensor Status Annunciation

#define BLE_CGMS_STATUS_SESSION_STOPPED                  (0x01 << 0) //!< Status: Session Stopped.
#define BLE_CGMS_STATUS_DEVICE_BATTERY_LOW               (0x01 << 1) //!< Status: Device Battery Low.
#define BLE_CGMS_STATUS_SENSOR_TYPE_INCORRECT_FOR_DEVICE (0x01 << 2) //!< Status: Sensor type incorrect for device.
#define BLE_CGMS_STATUS_SENSOR_MALFUNCTION               (0x01 << 3) //!< Status: Sensor malfunction.
#define BLE_CGMS_STATUS_DEVICE_SPECIFIC_ALERT            (0x01 << 4) //!< Status: Device Specific Alert.
#define BLE_CGMS_STATUS_GENERAL_DEVICE_FAULT             (0x01 << 5) //!< Status: General device fault has occurred in the sensor.
/** @} */

// CGM Measurement flags

#define BLE_CGMS_FLAG_TREND_INFO_PRESENT                 0x01        //!< CGM Trend Information Present.
#define BLE_CGMS_FLAGS_QUALITY_PRESENT                   0x02        //!< CGM Quality Present.
#define BLE_CGMS_STATUS_FLAGS_WARNING_OCT_PRESENT        0x20        //!< Sensor Status Annunciation Field, Warning-Octet present.
#define BLE_CGMS_STATUS_FLAGS_CALTEMP_OCT_PRESENT        0x40        //!< Sensor Status Annunciation Field, Cal/Temp-Octet present.
#define BLE_CGMS_STATUS_FLAGS_STATUS_OCT_PRESENT         0x80        //!< Sensor Status Annunciation Field, Status-Octet present.

/**@brief CGM Measurement Sensor Status Annunciation. */
typedef struct
{
    uint8_t warning;               /**< Warning annunciation. */
    uint8_t calib_temp;            /**< Calibration and Temperature annunciation. */
    uint8_t status;                /**< Status annunciation. */
} ble_cgms_sensor_annunc_t;


/**@brief CGM measurement. */
typedef struct
{
    uint8_t                      flags;                      /**< Indicates the presence of optional fields and the Sensor Status Annunciation field. */
    uint16_t                     glucose_concentration;      /**< Glucose concentration. 16-bit word comprising 4-bit exponent and signed 12-bit mantissa. */
    uint16_t                     time_offset;                /**< Time offset. Represents the time difference between measurements. */
    ble_cgms_sensor_annunc_t sensor_status_annunciation; /**< Sensor Status Annunciation. Variable length, can include Status, Cal/Temp, and Warning. */
    uint16_t                     trend;                      /**< Optional field that can include Trend Information. */
    uint16_t                     quality;                    /**< Optional field that includes the Quality of the measurement. */
} ble_cgms_meas_t;


/**@brief CGM Measurement record. */
typedef struct
{
    ble_cgms_meas_t meas; /**< CGM measurement. */
} ble_cgms_rec_t;


/**@brief Features supported by the CGM Service. */
typedef struct
{
    uint32_t feature;         /**< Information on supported features in the CGM Service. */
    uint8_t  type;            /**< Type. */
    uint8_t  sample_location; /**< Sample location. */
} ble_cgms_feature_t;


/**@brief Status of the CGM measurement. */
typedef struct
{
    uint16_t                     time_offset; /**< Time offset. */
    ble_cgms_sensor_annunc_t status;      /**< Status. */
} ble_cgm_status_t;


/**@brief Specific Operation Control Point response structure. */
// typedef struct
// {
//     uint8_t opcode;                               /**< Opcode describing the response. */
//     uint8_t req_opcode;                           /**< The original opcode for the request to which this response belongs. */
//     uint8_t rsp_code;                             /**< Response code. */
//     uint8_t resp_val[BLE_CGMS_SOCP_RESP_LEN]; /**< Array containing the response value. */
//     uint8_t size_val;                             /**< Length of the response value. */
// } ble_socp_rsp_t;

#endif