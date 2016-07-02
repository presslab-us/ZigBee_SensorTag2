/**
  @file  sensortagapp.c
  @brief TI RTOS ZCL Home Automation SensorTag sample application
         interfacing with ZStack.

         The application interacts with the ZStack Thread
         via both messaging interface and C function interface.

  <!--
  Copyright 2014 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
  -->
*/

/**

 On the SensorTag:

  Key Press Description:

    KEY LEFT
      - Send On/Off Command

    KEY RIGHT
      - Invoke EZMode for Switch Endpoint

    KEY_LEFT and KEY RIGHT together
      - Invoke EZMode for Temp Sensor Endpoint and Light Sensor Endpoint
*/



//*****************************************************************************
// Includes
//*****************************************************************************

#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <string.h>
#include <inc/hw_ints.h>
#include "ICall.h"

#include "zstackapi.h"

#include <ioc.h>

#include "Board.h"

#include "board_key.h"
#include "board_led.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"

#include "util.h"
#include "zcl_port.h"
#include "sensortagapp.h"
#include "znwk_config.h"

#include "bsp_i2c.h"
#include "ext_flash.h"
#include "ext_flash_layout.h"
#include "sensor.h"
#include "sensor_ppd42.h"
#include "sensor_bmp280.h"
#include "sensor_tmp007.h"
#include "sensor_hdc1000.h"
#include "sensor_opt3001.h"
#include "sensor_seeed_multi_gas.h"
#include "math.h"

#include "scif.h"

//*****************************************************************************
// Constants
//*****************************************************************************

/* Event IDs */
#define SENSORTAGAPP_IDENTIFY_TIMEOUT_EVT        0x0001
#define SENSORTAGAPP_POLL_CONTROL_TIMEOUT_EVT    0x0002
#define SENSORTAGAPP_KEY_EVENT                   0x0004
//#define SENSORTAGAPP_X2                          0x0008
#define SENSORTAGAPP_TEMPSENSOR_MIN_TIMEOUT_EVT  0x0010
#define SENSORTAGAPP_TEMPSENSOR_MAX_TIMEOUT_EVT  0x0020
#define SENSORTAGAPP_HUMIDITYSENSOR_MIN_TIMEOUT_EVT  0x0040
#define SENSORTAGAPP_HUMIDITYSENSOR_MAX_TIMEOUT_EVT  0x0080
#define SENSORTAGAPP_PRESSENSOR_MIN_TIMEOUT_EVT  0x0100
#define SENSORTAGAPP_PRESSENSOR_MAX_TIMEOUT_EVT  0x0200
#define SENSORTAGAPP_LIGHTSENSOR_MIN_TIMEOUT_EVT 0x0400
#define SENSORTAGAPP_LIGHTSENSOR_MAX_TIMEOUT_EVT 0x0800
#define SENSORTAGAPP_DUSTSENSOR_TIMEOUT_EVT  0x1000
#define SENSORTAGAPP_GASSENSOR_TIMEOUT_EVT  0x2000

/* Debounce timeout in ticks */
#define SENSORTAGAPP_KEY_DEBOUNCE_TIMEOUT       200

#define SENSORTAGAPP_TS_MAX_ATTRIBUTES          15
#define SENSORTAGAPP_PS_MAX_ATTRIBUTES          16
#define SENSORTAGAPP_LS_MAX_ATTRIBUTES          13
#define SENSORTAGAPP_GSDS_MAX_ATTRIBUTES        14

#define SENSORTAGAPP_TS_EP                      9
#define SENSORTAGAPP_PS_EP                      10
#define SENSORTAGAPP_LS_EP                      11
#define SENSORTAGAPP_GSDS_EP                    12

#define SENSORTAGAPP_DEVICE_VERSION             0
#define SENSORTAGAPP_FLAGS                      0

#define SENSORTAGAPP_HWVERSION                  0
#define SENSORTAGAPP_ZCLVERSION                 0

#define SENSORTAGAPP_TS_MAX_INCLUSTERS          4
#define SENSORTAGAPP_TS_MAX_OUTCLUSTERS         1

#define SENSORTAGAPP_PS_MAX_INCLUSTERS          3
#define SENSORTAGAPP_PS_MAX_OUTCLUSTERS         1

#define SENSORTAGAPP_LS_MAX_INCLUSTERS          3
#define SENSORTAGAPP_LS_MAX_OUTCLUSTERS         1

#define SENSORTAGAPP_GSDS_MAX_INCLUSTERS        4
#define SENSORTAGAPP_GSDS_MAX_OUTCLUSTERS       1

#define SENSORTAGAPP_MAX_ENDPOINTS_DEFINED      4

#define SENSORTAGAPP_TS_EP_IDX                  0
#define SENSORTAGAPP_PS_EP_IDX                  1
#define SENSORTAGAPP_LS_EP_IDX                  2
#define SENSORTAGAPP_GSDS_EP_IDX                3

// Temperature sensor offset due to heating inside enclosure
#define CAL_SELFHEATING 1.7

// Delay from temp sensor enable to reading measurememt
// (allow for 250 ms conversion time)
#define SENSORTAGAPP_TEMP_MEAS_DELAY            275

// Delay from humidity sensor enable to reading measurememt
#define SENSORTAGAPP_HUMIDITY_MEAS_DELAY        15

// Delay from pressure sensor enable to reading measurememt
// (for two stage reading)
#define SENSORTAGAPP_PRES_MEAS_DELAY            80

// Delay from light sensor enable to reading measurememt
// (allow for 100 ms conversion time)
#define SENSORTAGAPP_ILLUM_MEAS_DELAY           113

// Constants for two-stage reading
#define SENSORTAGAPP_ILLUM_MEAS_DELAY           113


#define SENSORTAGAPP_INIT_TIMEOUT_VALUE         100

#define SENSORTAGAPP_1SEC_MSEC                  1000

#define SENSORTAGAPP_CONVERT_TO_SECONDS(a)      ((a)/SENSORTAGAPP_1SEC_MSEC)

#define SENSORTAGAPP_BL_OFFSET                   0x1F001

#if defined(ZCL_REPORT)
#define MIN_HA_MIN_REPORTING_INT         1     // Should be >= 1 by HA spec
#define MIN_HA_MAX_REPORTING_INT         0x3C  // Should be >= 0x3C by HA spec
#define MAX_REPORT_ATTR                  6
#define ZCL_PORT_REPORT_NV_ID            0x0002
#endif

#define ATTRID_MS_PRESSURE_MEASUREMENT_SCALED_VALUE         0x0010
#define ATTRID_MS_PRESSURE_MEASUREMENT_MIN_SCALED_VALUE     0x0011
#define ATTRID_MS_PRESSURE_MEASUREMENT_MAX_SCALED_VALUE     0x0012
#define ATTRID_MS_PRESSURE_MEASUREMENT_SCALED_TOLERANCE     0x0013
#define ATTRID_MS_PRESSURE_MEASUREMENT_SCALE                0x0014

#define ZCL_HA_DEVICEID_GSDS_SENSOR 0x277

#define ZCL_CLUSTER_ID_MS_DUST_MEASUREMENT 0x277
#define ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT 0x278

#define ATTRID_MS_DUST_1UM_MEASURED_VALUE 0
#define ATTRID_MS_DUST_25UM_MEASURED_VALUE 1

#define ATTRID_MS_GAS_NH3_MEASURED_VALUE 0
#define ATTRID_MS_GAS_CO_MEASURED_VALUE 1 
#define ATTRID_MS_GAS_NO2_MEASURED_VALUE 2

//*****************************************************************************
// Typedefs
//*****************************************************************************

// Structure used to read the current Temperature from the TMP007
typedef union
{
  struct
  {
    uint16_t  tempTarget, tempLocal;
  } v;
  uint16_t  a[2];
} TempData_t;

#if defined(ZCL_REPORT)
// Function pointer definition to callback timer setting function
typedef void (*configReportTimer_t)
             (uint16_t minInterval, uint16_t maxInterval);

// Structure used to manage report attribute configuration
typedef struct
{
  uint16_t minReportInt; // top 6 bytes are saved to nvram
  uint16_t maxReportInt;
  int16_t reportableChange;
  void* pAttr;
  void* pAttrOld;
  configReportTimer_t configTimer;
} reportConfig_t;
#endif

//*****************************************************************************
// Global Variables
//*****************************************************************************

#if defined(EXTERNAL_IMAGE_CHECK)
/* SPI Pin Handle */
PIN_Handle hGpioPin;
#endif
//*****************************************************************************
// Local Variables
//*****************************************************************************

// Semaphore used to post events to the application thread
static ICall_Semaphore sem;
static ICall_EntityID staEntity;

// Passed in function pointers to the NV driver
static NVINTF_nvFuncts_t *pfnStaNV = NULL;

// Hold the device's Zstack state when recieved
static zstack_DevState savedState = zstack_DevState_HOLD;

// Task pending events
volatile static uint16_t events = 0;

// Destination address to send the readings
static afAddrType_t staDstAddr;

// Transaction ID used to send data messages
static uint8_t staTransID = 0;

// ZStack Thread network information
static zstack_sysNwkInfoReadRsp_t *pNwkInfo = NULL;

// Key press parameters
static uint8_t keys;

// Clock/timer resources
static Clock_Struct identifyClkStruct;
static Clock_Handle identifyClkHandle;

#if defined(ZCL_REPORT)
// Clock resources for temp/light sensors
static Clock_Struct tempSensorMinClkStruct;
static Clock_Struct tempSensorMaxClkStruct;
static Clock_Handle tempSensorMinClkHandle;
static Clock_Handle tempSensorMaxClkHandle;
static Clock_Struct humiditySensorMinClkStruct;
static Clock_Struct humiditySensorMaxClkStruct;
static Clock_Handle humiditySensorMinClkHandle;
static Clock_Handle humiditySensorMaxClkHandle;
static Clock_Struct presSensorMinClkStruct;
static Clock_Struct presSensorMaxClkStruct;
static Clock_Handle presSensorMinClkHandle;
static Clock_Handle presSensorMaxClkHandle;

#if defined(LIGHTSENSOR_ENABLED)
static Clock_Struct lightSensorMinClkStruct;
static Clock_Struct lightSensorMaxClkStruct;
static Clock_Handle lightSensorMinClkHandle;
static Clock_Handle lightSensorMaxClkHandle;
#endif
#if defined(DUSTSENSOR_ENABLED)
static Clock_Struct dustSensorMaxClkStruct;
static Clock_Handle dustSensorMaxClkHandle;
#endif
#if defined(GASSENSOR_ENABLED)
static Clock_Struct gasSensorMaxClkStruct;
static Clock_Handle gasSensorMaxClkHandle;
#endif
#endif

// Cluster lists for the simple descriptor
static uint16_t tsInputClusters[SENSORTAGAPP_TS_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY
};
static uint16_t tsOutputClusters[SENSORTAGAPP_TS_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};

static uint16_t psInputClusters[SENSORTAGAPP_PS_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT
};
static uint16_t psOutputClusters[SENSORTAGAPP_PS_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};

#if defined(LIGHTSENSOR_ENABLED)
static uint16_t lsInputClusters[SENSORTAGAPP_LS_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
};
static uint16_t lsOutputClusters[SENSORTAGAPP_LS_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};
#endif

#if defined(DUSTSENSOR_ENABLED) || defined(GASSENSOR_ENABLED)
static uint16_t gsdsInputClusters[SENSORTAGAPP_GSDS_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
#if defined(DUSTSENSOR_ENABLED)
  ZCL_CLUSTER_ID_MS_DUST_MEASUREMENT,
#endif
#if defined(GASSENSOR_ENABLED)
  ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
#endif
};
static uint16_t gsdsOutputClusters[SENSORTAGAPP_GSDS_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};
#endif

// Endpoint descriptors
static endPointDesc_t staEpDesc[SENSORTAGAPP_MAX_ENDPOINTS_DEFINED] =
{
  0
};
static SimpleDescriptionFormat_t
  afSimpleDesc[SENSORTAGAPP_MAX_ENDPOINTS_DEFINED] =
{
  0
};

#if defined(EXTERNAL_IMAGE_CHECK)
// PIN table for the SPI
static PIN_Config spiPinTable[] =
{
  Board_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH |
  PIN_PUSHPULL | PIN_DRVSTR_MIN,  /* External flash chip select    */
  Board_SPI_DEVPK_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH |
  PIN_PUSHPULL | PIN_DRVSTR_MIN,  /* DevPack chip select           */
  PIN_TERMINATE /* Terminate list */
};

/* SPI pin state */
static PIN_State spiPinState;
#endif // EXTERNAL_IMAGE_CHECK

/**
 * @internal A semaphore used to wait on a clock event
 */
extern ti_sysbios_knl_Semaphore_Handle semaphore0;

//*****************************************************************************
// Attribute Variables
//*****************************************************************************

// Attributes that aren't writable
static const uint8_t staHWRevision = SENSORTAGAPP_HWVERSION;
static const uint8_t staZCLVersion = SENSORTAGAPP_ZCLVERSION;
static const uint8_t staManufacturerName[] =
{
  16,
  'P', 'r', 'e', 's', 's', 'l', 'a', 'b',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '
};
static const uint8_t staModelId[] =
{
  16,
  'T', 'I', '0', '0', '0', '1', ' ', ' ',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '
};
static const uint8_t staDateCode[] =
{
  16,
  '2', '0', '0', '6', '0', '8', '3', '1',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '
};
static const uint8_t staPowerSource = POWER_SOURCE_BATTERY;

// Device location - updatable over the air
static uint8_t staLocationDescription[17] =
{
  16,
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '
};
static uint8_t staPhysicalEnvironment = PHY_UNSPECIFIED_ENV;

// Identify Cluster
static uint16_t staIdentifyTime = 0;

// Temperature Sensor Cluster
static int16_t staTempMeasuredValue = 2200;  // 22.00C
static const int16_t staTempMinMeasuredValue = -4000;   // -40.00C
static const int16_t staTempMaxMeasuredValue = 12500;  // 125.00C
#if defined(ZCL_REPORT)
static int16_t staTempMeasuredValueOld;
#endif

// Humidity Sensor Cluster
static uint16_t staHumidityMeasuredValue = 5000;  // 50.00% RH
static const uint16_t staHumidityMinMeasuredValue = 0;   // 0% RH
static const uint16_t staHumidityMaxMeasuredValue = 10000;  // 100.00% RH
#if defined(ZCL_REPORT)
static uint16_t staHumidityMeasuredValueOld;
#endif

// Pressure Sensor Cluster
static int16_t staPresMeasuredValue = 1013;  // 101.3 kPa
static const int16_t staPresMinMeasuredValue = 300;   // 30.0 kPa
static const int16_t staPresMaxMeasuredValue = 1100;  // 110.0 kPa
static int16_t staPresScaledValue = 10133;  // 101.33 kPa
static const int16_t staPresMinScaledValue = 3000;   // 30.00 kPa
static const int16_t staPresMaxScaledValue = 11000;  // 110.00 kPa
static const int8_t staPresScale = -2;  // scale to 10e-2, for 0.01 resolution
#if defined(ZCL_REPORT)
static int16_t staPresScaledValueOld;
#endif

#if defined(LIGHTSENSOR_ENABLED)
// Illuminance Measurement Cluster
static uint16_t staIllumMeasuredValue = 30001;  // 1000 Lux
static const uint16_t staIllumMinMeasuredValue = 0xFFFF;   // not defined
static const uint16_t staIllumMaxMeasuredValue = 0xFFFF;  // not defined
#if defined(ZCL_REPORT)
static uint16_t staIllumMeasuredValueOld;
#endif  // ZCL_REPORT
#endif  // LIGHTSENSOR_ENABLED

static uint16_t staDust1umMeasuredValue;
static uint16_t staDust25umMeasuredValue;
static uint16_t staGasNH3MeasuredValue;
static uint16_t staGasCOMeasuredValue;
static uint16_t staGasNO2MeasuredValue;

//*****************************************************************************
// Reportable Attribute Configuration Variables
//*****************************************************************************
#if defined(ZCL_REPORT)
static reportConfig_t staTempReportConfig;
static reportConfig_t staHumidityReportConfig;
static reportConfig_t staPresReportConfig;
#if defined(LIGHTSENSOR_ENABLED)
static reportConfig_t staIllumReportConfig;
#endif
#if defined(DUSTSENSOR_ENABLED)
static reportConfig_t staDustReportConfig;
#endif
#if defined(GASSENSOR_ENABLED)
static reportConfig_t staGasReportConfig;
#endif
static reportConfig_t* staReportConfigList[MAX_REPORT_ATTR];
#endif

//*****************************************************************************
// Local Function Prototypes
//*****************************************************************************

static void SensorTagApp_initialization(void);
static void SensorTagApp_initializeClocks(void);
static void SensorTagApp_registerEndpoints(void);
static void SensorTagApp_setupZStackCallbacks(void);
static void SensorTagApp_writeZStackParameters(void);
static void SensorTagApp_initializeZStack(void);
static void SensorTagApp_process(void);
static void SensorTagApp_processZStackMsgs(zstackmsg_genericReq_t *pMsg);
static void SensorTagApp_processAfIncomingMsgInd(
    zstack_afIncomingMsgInd_t *pInMsg);
static void SensorTag_handleKeys(uint8_t keys);

static bool SensorTagApp_readConvertIRTemp(int16_t *pResult);
static bool SensorTagApp_readConvertHDCTemp(int16_t *pResult);
static bool SensorTagApp_readConvertHumidity(int16_t *pResult);
static bool SensorTagApp_readConvertPressure(int16_t *pResult);
#if defined(LIGHTSENSOR_ENABLED)
static bool SensorTagApp_readConvertIllum(uint16_t *pResult);
#endif
#if defined(DUSTSENSOR_ENABLED)
static bool SensorTagApp_readConvertDust(float *pResult_1um, float *pResult_25um);
#endif
#if defined(GASSENSOR_ENABLED)
static bool SensorTagApp_readConvertGas(float *pResult_NH3, float *pResult_CO, float *pResult_NO2);
#endif

static void SensorTagApp_processKeyChangeCallback(uint8_t keysPressed);

static void SensorTagApp_processIdentifyTimeChange(void);
static void SensorTagApp_processIdentifyQueryResponseCallback(
    zclIdentifyQueryRsp_t *pRsp);
static void SensorTagApp_processIdentifyCallback(zclIdentify_t *pCmd);

static void SensorTagApp_processIdentifyTimeoutCallback(UArg a0);
static void SensorTagApp_setPollRate(uint32_t newPollRate);
static uint8_t SensorTagApp_processZCLMsg(zclIncoming_t *pInMsg);
static ZStatus_t SensorTagApp_processGasCmd(zclIncoming_t *pInMsg);

#if defined(ZCL_REPORT)
static uint8_t SensorTagApp_ProcessInConfigReportCmd(zclIncoming_t *pInMsg);
static void SensorTagApp_processTempSensorMinTimeoutCallback(UArg a0);
static void SensorTagApp_processTempSensorMaxTimeoutCallback(UArg a0);
static void SensorTagApp_setTempSensorTimer(uint16_t minInterval,
                                              uint16_t maxInterval);
static void SensorTagApp_sendTemp(void);
static void SensorTagApp_processHumiditySensorMinTimeoutCallback(UArg a0);
static void SensorTagApp_processHumiditySensorMaxTimeoutCallback(UArg a0);
static void SensorTagApp_setHumiditySensorTimer(uint16_t minInterval,
                                              uint16_t maxInterval);
static void SensorTagApp_sendHumidity(void);
static void SensorTagApp_processPresSensorMinTimeoutCallback(UArg a0);
static void SensorTagApp_processPresSensorMaxTimeoutCallback(UArg a0);
static void SensorTagApp_setPresSensorTimer(uint16_t minInterval,
                                              uint16_t maxInterval);
static void SensorTagApp_sendPressure(void);

#if defined(LIGHTSENSOR_ENABLED)
static void SensorTagApp_processLightSensorMinTimeoutCallback(UArg a0);
static void SensorTagApp_processLightSensorMaxTimeoutCallback(UArg a0);
static void SensorTagApp_setLightSensorTimer(uint16_t minInterval,
                                              uint16_t maxInterval);
static void SensorTagApp_sendIllum(void);
#endif
#if defined(DUSTSENSOR_ENABLED)
static void SensorTagApp_processDustSensorMaxTimeoutCallback(UArg a0);
static void SensorTagApp_setDustSensorTimer(uint16_t minInterval,
                                              uint16_t maxInterval);
static void SensorTagApp_sendDust(void);
#endif
#if defined(GASSENSOR_ENABLED)
static void SensorTagApp_processGasSensorMaxTimeoutCallback(UArg a0);
static void SensorTagApp_setGasSensorTimer(uint16_t minInterval,
                                           uint16_t maxInterval);
static void SensorTagApp_sendGas(void);
#endif
static void SensorTagApp_initReportConfig(void);
static reportConfig_t* SensorTagApp_getReportConfig(zclAttribute_t* pAttr, int* id);
#endif

static void SensorTagApp_applyFactoryImage(void);
static bool SensorTagApp_hasFactoryImage(void);

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t cmdCallbacks =
{
  NULL,                                   // Basic Cluster Reset command
  SensorTagApp_processIdentifyCallback,   // Identify command
#ifdef ZCL_EZMODE
  NULL,                                   // Identify EZ-Mode Invoke command
  NULL,                                   // Identify Update Commission State
#endif
  NULL,                                   // Identify Trigger Effect command
  SensorTagApp_processIdentifyQueryResponseCallback, // Identify Query Rsp
  NULL,        // On/Off cluster commands
  NULL,        // On/Off cluster enhanced command Off with Effect
  NULL,        // On/Off cluster enhanced command On with Recall Global Scene
  NULL,        // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level cmd
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                   // Scene Store Request command
  NULL,                                   // Scene Recall Request command
  NULL,                                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                   // Get Event Log command
  NULL,                                   // Publish Event Log command
#endif
  NULL,                                   // RSSI Location command
  NULL                                    // RSSI Location Response command
};

const zclAttrRec_t ztsAttrs[SENSORTAGAPP_TS_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,      // Cluster IDs - defined in the
                                   //   foundation (ie. zcl.h)
    {
      // Attribute record
      ATTRID_BASIC_HW_VERSION,   // Attribute ID - Found in Cluster
                                 //  Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,        // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,       // Variable access control
                                 //  - found in zcl.h
      (void *)&staHWRevision     // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staDateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staPowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)staLocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staPhysicalEnvironment
    }
  },

  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {
      // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staIdentifyTime
    }
  },

  // *** Temperature Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_TEMPERATURE_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staTempMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_TEMPERATURE_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staTempMinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_TEMPERATURE_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staTempMaxMeasuredValue
    }
  },

  // *** Humidity Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    {
      // Attribute record
      ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staHumidityMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    {
      // Attribute record
      ATTRID_MS_RELATIVE_HUMIDITY_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staHumidityMinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    {
      // Attribute record
      ATTRID_MS_RELATIVE_HUMIDITY_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staHumidityMaxMeasuredValue
    }
  },
};

const zclAttrRec_t zpsAttrs[SENSORTAGAPP_PS_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,      // Cluster IDs - defined in the
                                   //   foundation (ie. zcl.h)
    {
      // Attribute record
      ATTRID_BASIC_HW_VERSION,   // Attribute ID - Found in Cluster
                                 //  Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,        // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,       // Variable access control
                                 //  - found in zcl.h
      (void *)&staHWRevision     // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staDateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staPowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)staLocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staPhysicalEnvironment
    }
  },

  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {
      // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staIdentifyTime
    }
  },

  // *** Pressure Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staPresMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_PRESSURE_MEASUREMENT_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staPresMinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_PRESSURE_MEASUREMENT_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staPresMaxMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_PRESSURE_MEASUREMENT_SCALED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staPresScaledValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_PRESSURE_MEASUREMENT_MIN_SCALED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staPresMinScaledValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_PRESSURE_MEASUREMENT_MAX_SCALED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&staPresMaxScaledValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_PRESSURE_MEASUREMENT_SCALE,
      ZCL_DATATYPE_INT8,
      ACCESS_CONTROL_READ,
      (void *)&staPresScale
    }
  },
};

#if defined(LIGHTSENSOR_ENABLED)
const zclAttrRec_t zlsAttrs[SENSORTAGAPP_LS_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,      // Cluster IDs - defined in the
                                   //   foundation (ie. zcl.h)
    {
      // Attribute record
      ATTRID_BASIC_HW_VERSION,   // Attribute ID - Found in Cluster
                                 //  Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,        // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,       // Variable access control
                                 //  - found in zcl.h
      (void *)&staHWRevision     // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staDateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staPowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)staLocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staPhysicalEnvironment
    }
  },

  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {
      // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staIdentifyTime
    }
  },

  // *** Illuminance Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_ILLUMINANCE_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staIllumMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_ILLUMINANCE_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staIllumMinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_ILLUMINANCE_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staIllumMaxMeasuredValue
    }
  },
};
#endif

#if defined(DUSTSENSOR_ENABLED) || defined(GASSENSOR_ENABLED)
const zclAttrRec_t zgsdsAttrs[SENSORTAGAPP_GSDS_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,      // Cluster IDs - defined in the
                                   //   foundation (ie. zcl.h)
    {
      // Attribute record
      ATTRID_BASIC_HW_VERSION,   // Attribute ID - Found in Cluster
                                 //  Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,        // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,       // Variable access control
                                 //  - found in zcl.h
      (void *)&staHWRevision     // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)staDateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&staPowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)staLocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staPhysicalEnvironment
    }
  },

  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {
      // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&staIdentifyTime
    }
  },

#if defined(DUSTSENSOR_ENABLED)
  // *** Dust Sensor Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_DUST_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_DUST_1UM_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staDust1umMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_DUST_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_DUST_25UM_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staDust25umMeasuredValue
    }
  },
#endif
#if defined(GASSENSOR_ENABLED)
  // *** Gas Sensor Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_GAS_NH3_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staGasNH3MeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_GAS_CO_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staGasCOMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
    {
      // Attribute record
      ATTRID_MS_GAS_NO2_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&staGasNO2MeasuredValue
    }
  },
#endif
};
#endif

/**
 * @internal Clock handler function
 * @param a0 ignored
 */
Void tirtosapp_clock(UArg a0)
{
  /* Wake up the application thread when it waits for clock event */
  Semaphore_post(semaphore0);
}


float CalculateDustMass(uint16_t lpo, float size_um)
{
  float ratio = (float)lpo / 65536.0;
  float count = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;

  if (ratio == 0)
    return 0.0;
  
  return count;
/*  
  double pi = 3.14159;
  double density = 1.65*pow(10,12);
  double K = 3531.5;

    // PM2.5
    double r25 = 0.44*pow(10,-6);
    double vol25 = (4/3)*pi*pow(r25,3);
    double mass25 = density*vol25;
    concentration[PM25] = (count[PM25])*K*mass25;
    // End of mass concentration calculation

  if (size_um == 1)
  {
    PM1.0
    double r10 = 2.6*pow(10,-6);
    double vol10 = (4/3)*pi*pow(r10,3);
    double mass10 = density*vol10;
    concentration[PM10] = (count[PM10])*K*mass10;
  } else if (size_um == 2.5)
  {
  }
  return 0;
  */
}

/*void taskAlertCallback(void)
{
  scifClearAlertIntSource();
  
  // Find the dust sensor values (can access directly since they are single-buffered value, and
  // we always want the latest value)

  // Acknowledge the alert event
  scifAckAlertEvents();
}*/

/******************************************************************************
 * @fn          SensorTagApp_task
 *
 * @brief       Application task entry point for the ZStack HA Temperature
 *              Sensor Application.
 *
 * @param       pfnNV - pointer to the NV functions
 *
 * @return      none
 */
void SensorTagApp_task(NVINTF_nvFuncts_t *pfnNV)
{
  // Save and register the function pointers to the NV drivers
  pfnStaNV = pfnNV;
  zclport_registerNV(pfnStaNV, ZCL_PORT_SCENE_TABLE_NV_ID);

  // Initialize application
  SensorTagApp_initialization();

  // No return from task process
  SensorTagApp_process();
}

/******************************************************************************
 * @fn          SensorTagApp_initialization
 *
 * @brief       Initialize the application
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_initialization(void)
{
  /* Initialize variables */
  staDstAddr.addrMode = afAddrNotPresent;
  staDstAddr.addr.shortAddr = 0;
  staDstAddr.endPoint = 0;
  staDstAddr.panId = 0;

  // Setup I2C for sensors
  bspI2cInit();

  // Initialise the Temperature Sensor driver
  sensorTmp007Init();
  
  // Initialize the Humidity Sensor driver
  sensorHdc1000Init();
  
  // Initialize the Pressure Sensor driver
  sensorBmp280Init();

#if defined(LIGHTSENSOR_ENABLED)
  // Initialise the Light Sensor driver
  sensorOpt3001Init();
#endif

#if defined(GASSENSOR_ENABLED)
  // Initialise the gas sensor driver
  sensorSeeedGasInit();
#endif

  SensorTagApp_initializeClocks();

  /* Initialize keys */
  Board_Key_initialize(SensorTagApp_processKeyChangeCallback);

  /* Initialize the LEDS */
  Board_Led_initialize();

#if defined(EXTERNAL_IMAGE_CHECK)
  /* Initialize the SPI pins */
  hGpioPin = PIN_open(&spiPinState, spiPinTable);
#endif // EXTERNAL_IMAGE_CHECK

#if defined(ZCL_REPORT)
  SensorTagApp_initReportConfig();
#endif

  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&staEntity, &sem);

#if defined(DUSTSENSOR_ENABLED)
  // Initialize and start the Sensor Controller
//  scifOsalRegisterTaskAlertCallback(taskAlertCallback);
  scifOsalInit();
  scifInit(&scifDriverSetup);
  scifStartRtcTicksNow(60); // 65536 ticks per 60 seconds

  // Start the Dust Sensor task
  scifStartTasksNbl(BV(SCIF_DUST_SENSOR_TASK_ID));
#endif

  events |= SENSORTAGAPP_TEMPSENSOR_MIN_TIMEOUT_EVT;
  events |= SENSORTAGAPP_HUMIDITYSENSOR_MIN_TIMEOUT_EVT;
  events |= SENSORTAGAPP_LIGHTSENSOR_MIN_TIMEOUT_EVT;

  // Initialize the ZStack
  SensorTagApp_initializeZStack();
}

/******************************************************************************
 * @fn      SensorTagApp_initializeClocks
 *
 * @brief   Initialize Clocks
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_initializeClocks(void)
{
  // Initialize the timers needed for this application
  identifyClkHandle = Util_constructClock(
    &identifyClkStruct,
    SensorTagApp_processIdentifyTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

#if defined(ZCL_REPORT)
  tempSensorMinClkHandle = Util_constructClock(
    &tempSensorMinClkStruct,
    SensorTagApp_processTempSensorMinTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

  tempSensorMaxClkHandle = Util_constructClock(
    &tempSensorMaxClkStruct,
    SensorTagApp_processTempSensorMaxTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

  humiditySensorMinClkHandle = Util_constructClock(
    &humiditySensorMinClkStruct,
    SensorTagApp_processHumiditySensorMinTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

  humiditySensorMaxClkHandle = Util_constructClock(
    &humiditySensorMaxClkStruct,
    SensorTagApp_processHumiditySensorMaxTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

  presSensorMinClkHandle = Util_constructClock(
    &presSensorMinClkStruct,
    SensorTagApp_processPresSensorMinTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

  presSensorMaxClkHandle = Util_constructClock(
    &presSensorMaxClkStruct,
    SensorTagApp_processPresSensorMaxTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

#if defined(LIGHTSENSOR_ENABLED)
  lightSensorMinClkHandle = Util_constructClock(
    &lightSensorMinClkStruct,
    SensorTagApp_processLightSensorMinTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);

  lightSensorMaxClkHandle = Util_constructClock(
    &lightSensorMaxClkStruct,
    SensorTagApp_processLightSensorMaxTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);
#endif

#if defined(DUSTSENSOR_ENABLED)
  dustSensorMaxClkHandle = Util_constructClock(
    &dustSensorMaxClkStruct,
    SensorTagApp_processDustSensorMaxTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);
#endif

#if defined(GASSENSOR_ENABLED)
  gasSensorMaxClkHandle = Util_constructClock(
    &gasSensorMaxClkStruct,
    SensorTagApp_processGasSensorMaxTimeoutCallback,
    SENSORTAGAPP_INIT_TIMEOUT_VALUE,
    0,
    false,
    0);
#endif

#endif
}

/******************************************************************************
 * @fn      SensorTagApp_registerEndpoints
 *
 * @brief   Setup a Zigbee HA Door Lock Controller Endpoint
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_registerEndpoints(void)
{
  // Initialize the Thermostat Client Simple Descriptor
  staEpDesc[SENSORTAGAPP_TS_EP_IDX].endPoint = SENSORTAGAPP_TS_EP;
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].EndPoint = SENSORTAGAPP_TS_EP;
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].AppProfId = ZCL_HA_PROFILE_ID;
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].AppDeviceId =
      ZCL_HA_DEVICEID_THERMOSTAT;
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].AppDevVer =
      SENSORTAGAPP_DEVICE_VERSION;
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].AppNumInClusters =
      sizeof(tsInputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].pAppInClusterList = tsInputClusters;
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].AppNumOutClusters =
      sizeof(tsOutputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_TS_EP_IDX].pAppOutClusterList = tsOutputClusters;
  staEpDesc[SENSORTAGAPP_TS_EP_IDX].simpleDesc =
      &afSimpleDesc[SENSORTAGAPP_TS_EP_IDX];
  (void)zclport_registerEndpoint(staEntity,
                                 &staEpDesc[SENSORTAGAPP_TS_EP_IDX]);

     // Initialize the Pressure Sensor Simple Descriptor
  staEpDesc[SENSORTAGAPP_PS_EP_IDX].endPoint = SENSORTAGAPP_PS_EP;
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].EndPoint = SENSORTAGAPP_PS_EP;
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].AppProfId = ZCL_HA_PROFILE_ID;
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].AppDeviceId =
      ZCL_HA_DEVICEID_PRESSURE_SENSOR;
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].AppDevVer =
      SENSORTAGAPP_DEVICE_VERSION;
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].AppNumInClusters =
      sizeof(psInputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].pAppInClusterList = psInputClusters;
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].AppNumOutClusters =
      sizeof(psOutputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_PS_EP_IDX].pAppOutClusterList = psOutputClusters;
  staEpDesc[SENSORTAGAPP_PS_EP_IDX].simpleDesc =
      &afSimpleDesc[SENSORTAGAPP_PS_EP_IDX];
  (void)zclport_registerEndpoint(staEntity,
                                 &staEpDesc[SENSORTAGAPP_PS_EP_IDX]);

#if defined(LIGHTSENSOR_ENABLED)
  // Initialize the Light Sensor Simple Descriptor
  staEpDesc[SENSORTAGAPP_LS_EP_IDX].endPoint = SENSORTAGAPP_LS_EP;
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].EndPoint = SENSORTAGAPP_LS_EP;
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].AppProfId = ZCL_HA_PROFILE_ID;
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].AppDeviceId =
      ZCL_HA_DEVICEID_LIGHT_SENSOR;
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].AppDevVer =
      SENSORTAGAPP_DEVICE_VERSION;
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].AppNumInClusters =
      sizeof(lsInputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].pAppInClusterList = lsInputClusters;
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].AppNumOutClusters =
      sizeof(lsOutputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_LS_EP_IDX].pAppOutClusterList = lsOutputClusters;
  staEpDesc[SENSORTAGAPP_LS_EP_IDX].simpleDesc =
      &afSimpleDesc[SENSORTAGAPP_LS_EP_IDX];
  (void)zclport_registerEndpoint(staEntity,
                                 &staEpDesc[SENSORTAGAPP_LS_EP_IDX]);
#endif
#if defined(DUSTSENSOR_ENABLED) || defined(GASSENSOR_ENABLED)

  // Initialize the Gas/Dust Sensor Simple Descriptor
  staEpDesc[SENSORTAGAPP_GSDS_EP_IDX].endPoint = SENSORTAGAPP_GSDS_EP;
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].EndPoint = SENSORTAGAPP_GSDS_EP;
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].AppProfId = ZCL_HA_PROFILE_ID;
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].AppDeviceId =
      ZCL_HA_DEVICEID_GSDS_SENSOR;
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].AppDevVer =
      SENSORTAGAPP_DEVICE_VERSION;
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].AppNumInClusters =
      sizeof(gsdsInputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].pAppInClusterList = gsdsInputClusters;
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].AppNumOutClusters =
      sizeof(gsdsOutputClusters) / sizeof(uint_least16_t);
  afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX].pAppOutClusterList = gsdsOutputClusters;
  staEpDesc[SENSORTAGAPP_GSDS_EP_IDX].simpleDesc =
      &afSimpleDesc[SENSORTAGAPP_GSDS_EP_IDX];
  (void)zclport_registerEndpoint(staEntity,
                                 &staEpDesc[SENSORTAGAPP_GSDS_EP_IDX]);
#endif
}

/******************************************************************************
 * @fn      SensorTagApp_setupZStackCallbacks
 *
 * @brief   Setup the Zstack Callbacks wanted
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_setupZStackCallbacks(void)
{
  zstack_devZDOCBReq_t zdoCBReq = {0};

  // Register for Callbacks, turn on:
  //  Device State Change,
  //  ZDO Match Descriptor Response,
  zdoCBReq.has_devStateChange = true;
  zdoCBReq.devStateChange = true;

  (void)Zstackapi_DevZDOCBReq(staEntity, &zdoCBReq);
}

/******************************************************************************
 * @fn      SensorTagApp_writeZStackParameters
 *
 * @brief   Initialize ZStack Parameters
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_writeZStackParameters(void)
{
  zstack_sysConfigWriteReq_t  writeReq = { 0 };
  uint8_t                     extendedPANID[] = ZNWK_CONFIG_EXTENDED_PAN_ID;

  // HA specifies no Multicast, use group broadcast
  writeReq.has_nwkUseMultiCast = true;
  writeReq.nwkUseMultiCast = false;

  // Update the Default Channel List, defined in znwk_config.h
  writeReq.has_chanList = true;
  writeReq.chanList = ZNWK_DEFAULT_CHANLIST;

  // Update the Extended PAN ID, defined in znwk_config.h
  writeReq.has_extendedPANID = true;
  memcpy(&(writeReq.extendedPANID), extendedPANID, 8);

  // Update the config PAN ID, defined in znwk_config.h
  writeReq.has_panID = true;
  writeReq.panID = ZNWK_CONFIG_PAN_ID;
  (void) Zstackapi_sysConfigWriteReq(staEntity, &writeReq);
}

/******************************************************************************
 * @fn      SensorTagApp_initializeZStack
 *
 * @brief   Initialize ZStack
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_initializeZStack(void)
{
  // Initialize the ZStack Thread
  bool  startDev = true;  // default to auto-start

  // Setup the endpoints
  SensorTagApp_registerEndpoints();

  // Setup indications from ZStack
  SensorTagApp_setupZStackCallbacks();

  if(startDev)
  {
    zstack_devStartReq_t  startReq = { 0 };

    // Start the ZStack Thread
    startReq.startDelay = 0;
    (void) Zstackapi_DevStartReq(staEntity, &startReq);
  }

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks(SENSORTAGAPP_TS_EP, &cmdCallbacks);
  zclGeneral_RegisterCmdCallbacks(SENSORTAGAPP_PS_EP, &cmdCallbacks);
  zclGeneral_RegisterCmdCallbacks(SENSORTAGAPP_LS_EP, &cmdCallbacks);
  zclGeneral_RegisterCmdCallbacks(SENSORTAGAPP_GSDS_EP, &cmdCallbacks);
  zcl_registerPlugin( ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
                      ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
                      SensorTagApp_processGasCmd );

  // Register the callback function for unprocessed ZCL Foundation commands
  zclport_registerZclHandleExternal(SensorTagApp_processZCLMsg);

  // Register the application's attribute list
  zcl_registerAttrList(SENSORTAGAPP_TS_EP,
                       SENSORTAGAPP_TS_MAX_ATTRIBUTES, ztsAttrs);
  zcl_registerAttrList(SENSORTAGAPP_PS_EP,
                       SENSORTAGAPP_PS_MAX_ATTRIBUTES, zpsAttrs);
#if defined(LIGHTSENSOR_ENABLED)
  zcl_registerAttrList(SENSORTAGAPP_LS_EP,
                       SENSORTAGAPP_LS_MAX_ATTRIBUTES, zlsAttrs);
#endif
#if defined(DUSTSENSOR_ENABLED) || defined(GASSENSOR_ENABLED)
  zcl_registerAttrList(SENSORTAGAPP_GSDS_EP,
                       SENSORTAGAPP_GSDS_MAX_ATTRIBUTES, zgsdsAttrs);
#endif
  // Update the ZStack Parameters
  SensorTagApp_writeZStackParameters();
}

/******************************************************************************
 * @fn      SensorTagApp_process
 *
 * @brief   Application task processing start.
 *
 * @param   none
 *
 * @return  void
 */
static void SensorTagApp_process(void)
{
  /* Forever loop */
  for (;;)
  {
    ICall_ServiceEnum       stackid;
    ICall_EntityID          dest;
    zstackmsg_genericReq_t  *pMsg;

    /* Wait for response message */
    if(ICall_wait(ICALL_TIMEOUT_FOREVER) == ICALL_ERRNO_SUCCESS)
    {
      /* Retrieve the response message */
      if(ICall_fetchServiceMsg(&stackid, &dest, (void **) &pMsg) ==
           ICALL_ERRNO_SUCCESS)
      {
        if((stackid == ICALL_SERVICE_CLASS_ZSTACK) && (dest == staEntity))
        {
          if(pMsg)
          {
            SensorTagApp_processZStackMsgs(pMsg);

            // Free any separately allocated memory
            Zstackapi_freeIndMsg(pMsg);
          }
        }

        if(pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      if(events & SENSORTAGAPP_KEY_EVENT)
      {
        // Process Key Presses
        SensorTag_handleKeys(keys);
        keys = 0;
        events &= ~SENSORTAGAPP_KEY_EVENT;
      }

      if(events & SENSORTAGAPP_IDENTIFY_TIMEOUT_EVT)
      {
        // Process the Identify timer expiration
        if(staIdentifyTime > 0)
        {
          staIdentifyTime--;
        }

        SensorTagApp_processIdentifyTimeChange();

        events &= ~SENSORTAGAPP_IDENTIFY_TIMEOUT_EVT;
      }

#if defined(ZCL_REPORT)
      if(events & SENSORTAGAPP_TEMPSENSOR_MIN_TIMEOUT_EVT)
      {
        int16_t result;

        // Read the current temperature
//        if(SensorTagApp_readConvertIRTemp(&result))
        if(SensorTagApp_readConvertHDCTemp(&result))
        {
          staTempMeasuredValue = result;
        }

        // Report if the current temperature is higher or lower than
		// the last reported value by the reportable change
        if(abs(staTempMeasuredValue - staTempMeasuredValueOld) >=
             abs(staTempReportConfig.reportableChange))
        {
          SensorTagApp_sendTemp();
          staTempMeasuredValueOld = staTempMeasuredValue;

          // Reset both min & max timers
          SensorTagApp_setTempSensorTimer(staTempReportConfig.minReportInt,
                                          staTempReportConfig.maxReportInt);
        }
        else
        {
          // Don't reset max timer
          SensorTagApp_setTempSensorTimer(staTempReportConfig.minReportInt,
                                          0);
        }

        events &= ~SENSORTAGAPP_TEMPSENSOR_MIN_TIMEOUT_EVT;
      }

      if(events & SENSORTAGAPP_TEMPSENSOR_MAX_TIMEOUT_EVT)
      {
        int16_t result;

        // Read the current temperature
//        if(SensorTagApp_readConvertIRTemp(&result))
        if(SensorTagApp_readConvertHDCTemp(&result))
        {
          staTempMeasuredValue = result;
        }

        // Timed out. Report anyway.
        SensorTagApp_sendTemp();
        staTempMeasuredValueOld = staTempMeasuredValue;

        // Reset both min & max timers
        SensorTagApp_setTempSensorTimer(staTempReportConfig.minReportInt,
                                         staTempReportConfig.maxReportInt);

        events &= ~SENSORTAGAPP_TEMPSENSOR_MAX_TIMEOUT_EVT;
      }

      if(events & SENSORTAGAPP_HUMIDITYSENSOR_MIN_TIMEOUT_EVT)
      {
        int16_t result;

        // Read the current humidity
        if(SensorTagApp_readConvertHumidity(&result))
        {
          staHumidityMeasuredValue = result;
        }

        // Report if the current humidity is higher or lower than
		// the last reported value by the reportable change
        if(abs(staHumidityMeasuredValue - staHumidityMeasuredValueOld) >=
             abs(staHumidityReportConfig.reportableChange))
        {
          SensorTagApp_sendHumidity();
          staHumidityMeasuredValueOld = staHumidityMeasuredValue;

          // Reset both min & max timers
          SensorTagApp_setHumiditySensorTimer(staHumidityReportConfig.minReportInt,
                                          staHumidityReportConfig.maxReportInt);
        }
        else
        {
          // Don't reset max timer
          SensorTagApp_setHumiditySensorTimer(staHumidityReportConfig.minReportInt,
                                          0);
        }

        events &= ~SENSORTAGAPP_HUMIDITYSENSOR_MIN_TIMEOUT_EVT;
      }

      if(events & SENSORTAGAPP_HUMIDITYSENSOR_MAX_TIMEOUT_EVT)
      {
        int16_t result;

        // Read the current humidity
        if(SensorTagApp_readConvertHumidity(&result))
        {
          staHumidityMeasuredValue = result;
        }

        // Timed out. Report anyway.
        SensorTagApp_sendHumidity();
        staHumidityMeasuredValueOld = staHumidityMeasuredValue;

        // Reset both min & max timers
        SensorTagApp_setHumiditySensorTimer(staHumidityReportConfig.minReportInt,
                                         staHumidityReportConfig.maxReportInt);

        events &= ~SENSORTAGAPP_HUMIDITYSENSOR_MAX_TIMEOUT_EVT;
      }

      if(events & SENSORTAGAPP_PRESSENSOR_MIN_TIMEOUT_EVT)
      {
        int16_t result;

        // Read the current pressure
        if(SensorTagApp_readConvertPressure(&result))
        {
          staPresScaledValue = result;
          // MeasuredValue uses kPa * 10, so divide by 10.  This causes a loss of precision.
          staPresMeasuredValue = result / 10;
        }

        // Report if the current pressure is higher or lower than
		// the last reported value by the reportable change
        if(abs(staPresScaledValue - staPresScaledValueOld) >=
             abs(staPresReportConfig.reportableChange))
        {
          SensorTagApp_sendPressure();
          staPresScaledValueOld = staPresScaledValue;

          // Reset both min & max timers
          SensorTagApp_setPresSensorTimer(staPresReportConfig.minReportInt,
                                          staPresReportConfig.maxReportInt);
        }
        else
        {
          // Don't reset max timer
          SensorTagApp_setPresSensorTimer(staPresReportConfig.minReportInt,
                                          0);
        }

        events &= ~SENSORTAGAPP_PRESSENSOR_MIN_TIMEOUT_EVT;
      }

      if(events & SENSORTAGAPP_PRESSENSOR_MAX_TIMEOUT_EVT)
      {
        int16_t result;

        // Read the current pressure
        if(SensorTagApp_readConvertPressure(&result))
        {
          staPresScaledValue = result;
          // MeasuredValue uses kPa * 10, so divide by 10.  This causes a loss of precision.
          staPresMeasuredValue = result / 10;
        }

        // Timed out. Report anyway.
        SensorTagApp_sendPressure();
        staPresScaledValueOld = staPresScaledValue;

        // Reset both min & max timers
        SensorTagApp_setPresSensorTimer(staPresReportConfig.minReportInt,
                                         staPresReportConfig.maxReportInt);

        events &= ~SENSORTAGAPP_PRESSENSOR_MAX_TIMEOUT_EVT;
      }

#if defined(LIGHTSENSOR_ENABLED)
      if(events & SENSORTAGAPP_LIGHTSENSOR_MIN_TIMEOUT_EVT)
      {
        uint16_t result;
        uint16_t gapUnsigned;

        // Read the current illuminance
        if(SensorTagApp_readConvertIllum(&result))
        {
          staIllumMeasuredValue = result;
        }

        // Report if the current illuminance is higher or lower than
        // the last reported value by the reportable change
        gapUnsigned = (staIllumMeasuredValue > staIllumMeasuredValueOld) ?
                      (staIllumMeasuredValue - staIllumMeasuredValueOld) :
                      (staIllumMeasuredValueOld - staIllumMeasuredValue);
        if(gapUnsigned >= abs(staIllumReportConfig.reportableChange))
        {
          SensorTagApp_sendIllum();
          staIllumMeasuredValueOld = staIllumMeasuredValue;

          // Reset both min & max timers
          SensorTagApp_setLightSensorTimer(staIllumReportConfig.minReportInt,
                                           staIllumReportConfig.maxReportInt);
        }
        else
        {
          // Don't reset max timer
          SensorTagApp_setLightSensorTimer(staIllumReportConfig.minReportInt,
                                           0);
        }

        events &= ~SENSORTAGAPP_LIGHTSENSOR_MIN_TIMEOUT_EVT;
      }

      if(events & SENSORTAGAPP_LIGHTSENSOR_MAX_TIMEOUT_EVT)
      {
        uint16_t result;

        // Read the current illuminance
        if(SensorTagApp_readConvertIllum(&result))
        {
          staIllumMeasuredValue = result;
        }

        // Timed out. Report anyway.
        SensorTagApp_sendIllum();
        staIllumMeasuredValueOld = staIllumMeasuredValue;

        // Reset both min & max timers
        SensorTagApp_setLightSensorTimer(staIllumReportConfig.minReportInt,
                                          staIllumReportConfig.maxReportInt);

        events &= ~SENSORTAGAPP_LIGHTSENSOR_MAX_TIMEOUT_EVT;
      }
#endif  // LIGHTSENSOR_ENABLED
#if defined(DUSTSENSOR_ENABLED)
      if(events & SENSORTAGAPP_DUSTSENSOR_TIMEOUT_EVT)
      {
        float result1, result2;

        // Read the current dust levels
        if(SensorTagApp_readConvertDust(&result1, &result2))
        {
          if (result1 > 6553) result1 = 6553;
          if (result2 > 6553) result2 = 6553;
          staDust1umMeasuredValue = (uint16_t)(result1 * 10.0);
          staDust25umMeasuredValue = (uint16_t)(result2 * 10.0);
        }

        SensorTagApp_sendDust();

        // Reset both min & max timers
        SensorTagApp_setDustSensorTimer(staDustReportConfig.minReportInt,
                                        staDustReportConfig.maxReportInt);

        events &= ~SENSORTAGAPP_DUSTSENSOR_TIMEOUT_EVT;
      }
#endif  // DUSTSENSOR_ENABLED
#if defined(GASSENSOR_ENABLED)
      if(events & SENSORTAGAPP_GASSENSOR_TIMEOUT_EVT)
      {
        float result1, result2, result3;

        // Read the current gas levels
        if(SensorTagApp_readConvertGas(&result1, &result2, &result3))
        {
          if (result1 > 6553) result1 = 6553;
          if (result2 > 6553) result2 = 6553;
          if (result3 > 6553) result3 = 6553;
          staGasNH3MeasuredValue = (uint16_t)(result1 * 10.0);
          staGasCOMeasuredValue = (uint16_t)(result2 * 10.0);
          staGasNO2MeasuredValue = (uint16_t)(result3 * 10.0);
        }

        SensorTagApp_sendGas();

        // Reset both min & max timers
        SensorTagApp_setGasSensorTimer(staGasReportConfig.minReportInt,
                                       staGasReportConfig.maxReportInt);

        events &= ~SENSORTAGAPP_GASSENSOR_TIMEOUT_EVT;
      }
#endif  // DUSTSENSOR_ENABLED
#endif  // ZCL_REPORT
    }
  }
}

/******************************************************************************
 * @fn      SensorTagApp_processZStackMsgs
 *
 * @brief   Process event from Stack
 *
 * @param   pMsg - pointer to incoming ZStack message to process
 *
 * @return  void
 */
static void SensorTagApp_processZStackMsgs(zstackmsg_genericReq_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND:
      {
        // The ZStack Thread is indicating a State change
        zstackmsg_devStateChangeInd_t *pInd =
                                       (zstackmsg_devStateChangeInd_t *) pMsg;

        // Only process the state change if it actually changed.
        if(savedState != pInd->req.state)
        {
          savedState = pInd->req.state;

          if((pInd->req.state == zstack_DevState_DEV_ZB_COORD) ||
             (pInd->req.state == zstack_DevState_DEV_ROUTER) ||
             (pInd->req.state == zstack_DevState_DEV_END_DEVICE))
          {
            // The device is part of a network,
            // get the device's network parameters.
            pNwkInfo = zclport_getDeviceInfo(staEntity);
            if(pNwkInfo == NULL)
            {
              // Something didn't work properly
            }

            // Update the UI with network information
            Board_Led_control(board_led_type_LED1, board_led_state_BLINK);
            Board_Led_control(board_led_type_LED2, board_led_state_BLINK);

            if(pInd->req.state == zstack_DevState_DEV_END_DEVICE)
            {
              // Change the default poll rate from 1 second to
              // the config setting in znwk_config.h
              SensorTagApp_setPollRate(ZNWK_POLL_RATE);
            }
          }
          else
          {
            Board_Led_control(board_led_type_LED1, board_led_state_BLINKING);
            Board_Led_control(board_led_type_LED2, board_led_state_OFF);
          }
        }
      }
      break;

    case zstackmsg_CmdIDs_AF_INCOMING_MSG_IND:
      {
        // Process incoming data messages
        zstackmsg_afIncomingMsgInd_t *pInd =
                                     (zstackmsg_afIncomingMsgInd_t *) pMsg;
        SensorTagApp_processAfIncomingMsgInd(&(pInd->req));
      }
      break;

    /*
     * These are messages/indications from ZStack that this
     * application doesn't process.  These message can be
     * processed by your application, remove from this list and
     * process them here in this switch statement.
     */
    case zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND:
    case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE:
    case zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP:
    case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP:
    case zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP:
    case zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP:
    case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP:
    case zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP:
    case zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_RSP:
    case zstackmsg_CmdIDs_ZDO_USER_DESC_RSP:
    case zstackmsg_CmdIDs_ZDO_USER_DESC_SET_RSP:
    case zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP:
    case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP:
    case zstackmsg_CmdIDs_ZDO_BIND_RSP:
    case zstackmsg_CmdIDs_ZDO_UNBIND_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP:
    case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY:
    case zstackmsg_CmdIDs_ZDO_SRC_RTG_IND:
    case zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND:
    case zstackmsg_CmdIDs_ZDO_NWK_DISC_CNF:
    case zstackmsg_CmdIDs_ZDO_BEACON_NOTIFY_IND:
    case zstackmsg_CmdIDs_ZDO_JOIN_CNF:
    case zstackmsg_CmdIDs_ZDO_LEAVE_CNF:
    case zstackmsg_CmdIDs_ZDO_LEAVE_IND:
    case zstackmsg_CmdIDs_SYS_RESET_IND:
    case zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND:
    case zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND:
    case zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND:
      break;

    default:
      break;
  }
}

/******************************************************************************
 *
 * @fn          SensorTagApp_processAfIncomingMsgInd
 *
 * @brief       Process AF Incoming Message Indication message
 *
 * @param       pInMsg - pointer to the incoming message
 *
 * @return      none
 *
 */
static void SensorTagApp_processAfIncomingMsgInd(
    zstack_afIncomingMsgInd_t *pInMsg)
{
  afIncomingMSGPacket_t afMsg;

  // All incoming messages are passed to the ZCL message processor
  // Convert from ZStack API message to ZStack AF incoming data
  // Structure (ZCL needs this structure).
  afMsg.groupId = pInMsg->groupID;
  afMsg.clusterId = pInMsg->clusterId;
  afMsg.srcAddr.endPoint = pInMsg->srcAddr.endpoint;
  afMsg.srcAddr.panId = pInMsg->srcAddr.panID;
  afMsg.srcAddr.addrMode = (afAddrMode_t)pInMsg->srcAddr.addrMode;
  if((afMsg.srcAddr.addrMode == afAddr16Bit)
      || (afMsg.srcAddr.addrMode == afAddrGroup)
      || (afMsg.srcAddr.addrMode == afAddrBroadcast))
  {
    afMsg.srcAddr.addr.shortAddr = pInMsg->srcAddr.addr.shortAddr;
  }
  else if(afMsg.srcAddr.addrMode == afAddr64Bit)
  {
    memcpy(afMsg.srcAddr.addr.extAddr, &(pInMsg->srcAddr.addr.extAddr),
           Z_EXTADDR_LEN);
  }
  afMsg.macDestAddr = pInMsg->macDestAddr;
  afMsg.endPoint = pInMsg->endpoint;
  afMsg.wasBroadcast = pInMsg->wasBroadcast;
  afMsg.LinkQuality = pInMsg->linkQuality;
  afMsg.correlation = pInMsg->correlation;
  afMsg.rssi = pInMsg->rssi;
  afMsg.SecurityUse = pInMsg->securityUse;
  afMsg.timestamp = pInMsg->timestamp;
  afMsg.nwkSeqNum = pInMsg->nwkSeqNum;
  afMsg.macSrcAddr = pInMsg->macSrcAddr;
  afMsg.radius = pInMsg->radius;
  afMsg.cmd.TransSeqNumber = pInMsg->transSeqNum;
  afMsg.cmd.DataLength = pInMsg->n_payload;
  afMsg.cmd.Data = pInMsg->pPayload;

  zcl_ProcessMessageMSG(&afMsg);
}


/******************************************************************************
 *
 * @fn          SensorTagApp_processZCLMsg
 *
 * @brief       Process ZCL foundation commands
 *              not processed by zcl_ProcessMessageMSG()
 *
 * @param       pInMsg - pointer to the incoming message
 *
 * @return      TRUE
 *
 */
static uint8_t SensorTagApp_processZCLMsg(zclIncoming_t *pInMsg)
{
  switch (pInMsg->hdr.commandID)
  {
#if defined(ZCL_REPORT)
    case ZCL_CMD_CONFIG_REPORT:
      SensorTagApp_ProcessInConfigReportCmd(pInMsg);
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      // SensorTagApp_ProcessInReadReportCfgCmd(pInMsg);
      break;
#endif // ZCL_REPORT

    default:
      break;
  }

  if(pInMsg->attrCmd != NULL)
  {
    // free the parsed command
    ICall_free(pInMsg->attrCmd);
    pInMsg->attrCmd = NULL;
  }

  return TRUE;
}

static ZStatus_t SensorTagApp_processGasCmd(zclIncoming_t *pInMsg)
{
  if (!zcl_ClusterCmd(pInMsg->hdr.fc.type)) return ZFailure;
  if (pInMsg->msg->endPoint != SENSORTAGAPP_GSDS_EP) return ZFailure;
  
  if (pInMsg->hdr.commandID == 1)
  {
    sensorSeeedGasCalibrate();
  }

  return ZSuccess;
}

/******************************************************************************
 * @fn      SensorTag_handleKeys
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - keys that were pressed
 *
 * @return  void
 */
static void SensorTag_handleKeys(uint8_t keys)
{
  // Check for an indication that the Factory Reset is supposed to happen
  if(keys == (KEY_SELECT | KEY_UP | KEY_DOWN | KEY_LEFT | KEY_RIGHT))
  {
    zstack_sysResetReq_t req;
    req.type = zstack_ResetTypes_DEVICE;
    req.newNwkState = true;
    (void) Zstackapi_sysResetReq(staEntity, &req); // Perform a device reset and ignore NV
//    SensorTagApp_applyFactoryImage();
  }
}

/*********************************************************************
 * @fn      SensorTagApp_processKeyChangeCallback
 *
 * @brief   Key event handler function
 *
 * @param   keysPressed - ignored
 *
 * @return  none
 */
static void SensorTagApp_processKeyChangeCallback(uint8_t keysPressed)
{
  // Save the key press information
  keys = keysPressed;

  // set the key press event
  events |= SENSORTAGAPP_KEY_EVENT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SensorTagApp_processIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_processIdentifyTimeChange(void)
{
  // Stop the Identify timer
  if(Util_isClockActive(&identifyClkStruct) == true)
  {
    Util_stopClock(&identifyClkStruct);
  }

  // Are we still identifying?
  if(staIdentifyTime > 0)
  {
    // Continue with another timer
    Clock_setTimeout(identifyClkHandle,
          ((staIdentifyTime * SENSORTAGAPP_1SEC_MSEC) * TIMER_MS_ADJUSTMENT));
    Util_startClock(&identifyClkStruct);

    Board_Led_control(board_led_type_LED2, board_led_state_BLINKING);
  }
  else
  {
    Board_Led_control(board_led_type_LED2, board_led_state_OFF);
  }
}

/*********************************************************************
 * @fn      SensorTagApp_processIdentifyCallback
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   pCmd - pointer to Identify command
 *
 * @return  none
 */
static void SensorTagApp_processIdentifyCallback(zclIdentify_t *pCmd)
{
  // Save the incoming time and setup a timer
  staIdentifyTime = pCmd->identifyTime;
  SensorTagApp_processIdentifyTimeChange();
}

/*********************************************************************
 * @fn      SensorTagApp_processIdentifyQueryResponseCallback
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command
 *          for this application.
 *
 * @param   pRsp - pointer to the incoming ZCL Response
 *
 * @return  none
 */
static void SensorTagApp_processIdentifyQueryResponseCallback(
    zclIdentifyQueryRsp_t *pRsp)
{
  // If you don't have EZMode enabled, add your own code
  // to Process this response message
  (void) pRsp;
}

/******************************************************************************
 * @fn      SensorTagApp_processIdentifyTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processIdentifyTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_IDENTIFY_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn      SensorTagApp_setPollRate
 *
 * @brief   Set the ZStack Thread Poll Rate
 *
 * @param   newPollRate - new poll rate in milliseconds
 *
 * @return  none
 */
static void SensorTagApp_setPollRate(uint32_t newPollRate)
{
  zstack_sysConfigWriteReq_t  writeReq = { 0 };

  // Set the new poll rate
  writeReq.has_pollRate = true;
  writeReq.pollRate = newPollRate;
  (void) Zstackapi_sysConfigWriteReq(staEntity, &writeReq);
}

/******************************************************************************
 * @fn          SensorTagApp_readConvertIRTemp
 *
 * @brief       Call function to read temp sensor.
 *
 * @param       pResult - pointer to result
 *
 * @return      true if temp read
 */
static bool SensorTagApp_readConvertIRTemp(int16_t *pResult)
{
  bool        ret;
/*  uint16_t rawTemp, rawHum;

  // Read data from the HDC1000 Humidity sensor
  sensorHdc1000Start();
  delay_ms(15);
  ret = sensorHdc1000Read(&rawTemp, &rawHum);

  if(ret)
  {
    float tTmp; // result - temperature
    float tHum; // result - humidity

    // Convert from raw temp and humidity to Celcius and % RH
    sensorHdc1000Convert(rawTemp, rawHum, &tTmp, &tHum);

    // Return value is deg C * 100 for ZigBee.
    *pResult = (int16_t) (tTmp * 100.0);
  }*/

  TempData_t  tmpData;

  // Read data from the Tmp007 Temperature sensor
  sensorTmp007Enable(true);
  delay_ms(SENSORTAGAPP_TEMP_MEAS_DELAY);
  ret = sensorTmp007Read(&tmpData.v.tempLocal, &tmpData.v.tempTarget);
  sensorTmp007Enable(false);

  if(ret)
  {
    float tObj; // result - object temperature
    float tTgt; // result - ambience temperature

    // Convert from raw temp to Celcius
    sensorTmp007Convert(tmpData.v.tempTarget, tmpData.v.tempLocal,
                        &tObj, &tTgt);

    // We are using the ambience temperature from the chip.
    // If you want to use the object temperature, use tObj below.
    // Convert to pResult, multiply by 100,
    // use the 24.75 C will look like 2475
    *pResult = (int16_t) (tTgt * 100);
  }

  return ret;
}

/******************************************************************************
 * @fn          SensorTagApp_readConvertHDCTemp
 *
 * @brief       Call function to read temp sensor.
 *
 * @param       pResult - pointer to result
 *
 * @return      true if temp read
 */
static bool SensorTagApp_readConvertHDCTemp(int16_t *pResult)
{
  bool        ret;
  uint16_t rawTemp, rawHum;

  // Read data from the HDC1000 Humidity sensor
  sensorHdc1000Start();
  delay_ms(SENSORTAGAPP_HUMIDITY_MEAS_DELAY);
  ret = sensorHdc1000Read(&rawTemp, &rawHum);

  if(ret)
  {
    float tTmp; // result - temperature
    float tHum; // result - humidity

    // Convert from raw temp and humidity to Celcius and % RH
    sensorHdc1000Convert(rawTemp, rawHum, &tTmp, &tHum);

    // Return value is deg C * 100 for ZigBee.
    *pResult = (int16_t) ((tTmp - CAL_SELFHEATING) * 100.0);
  }

  return ret;
}

/******************************************************************************
 * @fn          SensorTagApp_readConvertHumidity
 *
 * @brief       Call function to read humidity sensor.
 *
 * @param       pResult - pointer to result
 *
 * @return      true if humidity read
 */
static bool SensorTagApp_readConvertHumidity(int16_t *pResult)
{
  bool        ret;
  uint16_t rawTemp, rawHum;

  // Read data from the HDC1000 Humidity sensor
  sensorHdc1000Start();
  delay_ms(SENSORTAGAPP_HUMIDITY_MEAS_DELAY);
  ret = sensorHdc1000Read(&rawTemp, &rawHum);

  if(ret)
  {
    float tTmp; // result - temperature
    float tHum; // result - humidity

    // Convert from raw temp and humidity to Celcius and % RH
    sensorHdc1000Convert(rawTemp, rawHum, &tTmp, &tHum);

    // Return value is % RH * 100 for ZigBee.
    *pResult = (int16_t) (tHum * 100.0);
  }

  return ret;
}


/******************************************************************************
 * @fn          SensorTagApp_readConvertPressure
 *
 * @brief       Call function to read pressure sensor.
 *
 * @param       pResult - pointer to result
 *
 * @return      true if pressure read
 */
static bool SensorTagApp_readConvertPressure(int16_t *pResult)
{
  bool        ret;
  uint8_t     rawData[BMP_DATA_SIZE];

  // Read data from the BMP280 Pressure sensor
  sensorBmp280Enable(true);
  delay_ms(SENSORTAGAPP_PRES_MEAS_DELAY);
  ret = sensorBmp280Read(rawData);
  sensorBmp280Enable(false);

  if(ret)
  {
    int32_t temp;
    uint32_t pres;

    // Convert from raw data to temperature and pressure
    sensorBmp280Convert(rawData, &temp, &pres);
    
    // Pressure is in kPa * 100.
    *pResult = pres / 10;
  }

  return ret;
}

#if defined(LIGHTSENSOR_ENABLED)
/******************************************************************************
 * @fn          SensorTagApp_readConvertIllum
 *
 * @brief       Call function to read light sensor.
 *
 * @param       pResult - pointer to result
 *
 * @return      true if illuminance read
 */
static bool SensorTagApp_readConvertIllum(uint16_t *pResult)
{
  bool      ret;
  uint16_t  illumData;

  // Read data from the OPT3001 light sensor
  sensorOpt3001Enable(true);
  delay_ms(SENSORTAGAPP_ILLUM_MEAS_DELAY);
  ret = sensorOpt3001Read(&illumData);
  sensorOpt3001Enable(false);

  if(ret)
  {
    float fLux, fResult;

    // Convert from raw illuminance to Lux and then
    // to Illuminance Measurement cluster value
    fLux = sensorOpt3001Convert(illumData);
    fResult = 10000.0 * log10(fLux) + 1;

    if(fResult > 65534)
    {
      *pResult = 65534;
    }
    else
    {
      *pResult = (uint_t) fResult;
    }
  }

  return ret;
}
#endif

#if defined(DUSTSENSOR_ENABLED)
/******************************************************************************
 * @fn          SensorTagApp_readConvertDust
 *
 * @brief       Call function to read dust sensor.
 *
 * @param       pResult_1um - pointer to 1um result
 *
 * @param       pResult_25um - pointer to 2.5um result
 *
 * @return      true if dust read
 */
static bool SensorTagApp_readConvertDust(float *pResult_1um, float *pResult_25um)
{
  // Read data from the dust sensor
  *pResult_1um = CalculateDustMass(scifTaskData.dustSensor.output.lpo1um, 1.0);
  *pResult_25um = CalculateDustMass(scifTaskData.dustSensor.output.lpo25um, 2.5);

  return true;
}
#endif

#if defined(GASSENSOR_ENABLED)
/******************************************************************************
 * @fn          SensorTagApp_readConvertGas
 *
 * @brief       Call function to read gas sensor.
 *
 * @param       pResult_NH3 - pointer to NH3 result
 *
 * @param       pResult_CO - pointer to CO result
 *
 * @param       pResult_NO2 - pointer to NO2 result
 *
 * @return      true if gas read
 */
static bool SensorTagApp_readConvertGas(float *pResult_NH3, float *pResult_CO, float *pResult_NO2)
{
  if (!sensorSeeedGasRead())
    return false;

  *pResult_NH3 = sensorSeeedGasConvert(GAS_NH3);
  *pResult_CO = sensorSeeedGasConvert(GAS_CO);
  *pResult_NO2 = sensorSeeedGasConvert(GAS_NO2);

  return true;
}
#endif

#if defined(ZCL_REPORT)
/*********************************************************************
 * @fn      SensorTagApp_ProcessInConfigReportCmd
 *
 * @brief   Process the Configure Reporting Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  TRUE if attribute was found in the Attribute list,
 *          FALSE if not
 */
static uint8_t SensorTagApp_ProcessInConfigReportCmd(zclIncoming_t *pInMsg)
{
  zclCfgReportCmd_t *cfgReportCmd;
  zclCfgReportRec_t *reportRec;
  zclCfgReportRspCmd_t *cfgReportRspCmd;
  zclAttrRec_t attrRec;
  uint8 status;
  uint8 i, j = 0;

  cfgReportCmd = (zclCfgReportCmd_t *)pInMsg->attrCmd;

  // Allocate space for the response command
  cfgReportRspCmd = (zclCfgReportRspCmd_t *) ICall_malloc(
                     sizeof(zclCfgReportRspCmd_t) +
                     sizeof (zclCfgReportStatus_t) * cfgReportCmd->numAttr);
  if(cfgReportRspCmd == NULL)
  {
    return FALSE; // EMBEDDED RETURN
  }

  // Process each Attribute Reporting Configuration record
  for (i = 0; i < cfgReportCmd->numAttr; i++)
  {
    reportRec = &(cfgReportCmd->attrList[i]);

    status = ZCL_STATUS_SUCCESS;

    if(zclFindAttrRec(pInMsg->msg->endPoint, pInMsg->msg->clusterId,
                         reportRec->attrID, &attrRec))
    {
      if(reportRec->direction == ZCL_SEND_ATTR_REPORTS)
      {
        if(reportRec->dataType == attrRec.attr.dataType)
        {
          reportConfig_t* pReportConfig;
          int reportId;

          // This the attribute that is to be reported
          if(pReportConfig = SensorTagApp_getReportConfig(&(attrRec.attr), &reportId))
          {
/*            if(reportRec->minReportInt < MIN_HA_MIN_REPORTING_INT ||
                 (reportRec->maxReportInt != 0 &&
                   (reportRec->maxReportInt < reportRec->minReportInt ||
                     reportRec->maxReportInt > MIN_HA_MAX_REPORTING_INT)))
            {
              // Invalid fields
              status = ZCL_STATUS_INVALID_VALUE;
            }
            else*/
            {
              // Set the Min and Max Reporting Intervals and Reportable Change
              pReportConfig->minReportInt = reportRec->minReportInt;
              pReportConfig->maxReportInt = reportRec->maxReportInt;
              pReportConfig->reportableChange = *(int16_t*)reportRec->reportableChange;
//              memcpy(pReportConfig->pReportableChange,
//                     reportRec->reportableChange,
//                     zclGetDataTypeLength(reportRec->dataType));
              pReportConfig->configTimer(pReportConfig->minReportInt,
                                         pReportConfig->maxReportInt);
              
              zclport_writeNV(ZCL_PORT_REPORT_NV_ID, reportId, 0, 2*3, pReportConfig);

              status = ZCL_STATUS_SUCCESS;
            }
          }
          else
          {
            // Attribute cannot be reported
            status = ZCL_STATUS_UNREPORTABLE_ATTRIBUTE;
          }
        }
        else
        {
          // Attribute data type is incorrect
          status = ZCL_STATUS_INVALID_DATA_TYPE;
        }
      }
      else
      {
        // ZCL_EXPECT_ATTR_REPORTS direction is not supported
        status = ZCL_STATUS_UNSUPPORTED_ATTRIBUTE; // for now
      }
    }
    else
    {
      // Attribute is not supported
      status = ZCL_STATUS_UNSUPPORTED_ATTRIBUTE;
    }

    // If not successful then record the status
    if(status != ZCL_STATUS_SUCCESS)
    {
      cfgReportRspCmd->attrList[j].status = status;
      cfgReportRspCmd->attrList[j++].attrID = reportRec->attrID;
    }
  } // for loop

  if(j == 0)
  {
    // Since all attributes were configured successfully, include a single
    // attribute status record in the response command with the status field
    // set to SUCCESS and the attribute ID field omitted.
    cfgReportRspCmd->attrList[0].status = ZCL_STATUS_SUCCESS;
    cfgReportRspCmd->numAttr = 1;
  }
  else
  {
    cfgReportRspCmd->numAttr = j;
  }

  // Send the response back
  zcl_SendConfigReportRspCmd(pInMsg->msg->endPoint, &(pInMsg->msg->srcAddr),
                             pInMsg->msg->clusterId, cfgReportRspCmd,
                             ZCL_FRAME_SERVER_CLIENT_DIR,
                             TRUE, pInMsg->hdr.transSeqNum);
  ICall_free(cfgReportRspCmd);

  return TRUE ;
}

/******************************************************************************
 * @fn      SensorTagApp_processTempSensorMinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processTempSensorMinTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_TEMPSENSOR_MIN_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn      SensorTagApp_processTempSensorMaxTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processTempSensorMaxTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_TEMPSENSOR_MAX_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn          SensorTagApp_setTempTimer
 *
 * @brief       Call function to Setup the next Temperature reading
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_setTempSensorTimer(uint16_t minInterval,
                                            uint16_t maxInterval)
{
  // If the Temp timers are running, stop them
  if(Util_isClockActive(&tempSensorMinClkStruct) == true)
  {
      Util_stopClock(&tempSensorMinClkStruct);
  }

  // If maxInterval is 0, don't stop currently running max timer
  if((maxInterval != 0) &&
      (Util_isClockActive(&tempSensorMaxClkStruct) == true))
  {
      Util_stopClock(&tempSensorMaxClkStruct);
  }

  // Reconfigure Temp Sensor timers in 10 usec increments
  if(maxInterval != ZCL_REPORTING_OFF)
  {
    if(minInterval != maxInterval)
    {
      // Don't run min timer if min int is the same as max int
      Clock_setTimeout(tempSensorMinClkHandle,
                       (minInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&tempSensorMinClkStruct);
    }

    if(maxInterval != 0)
    {
      // Restart the max timer if maxInterval is not 0
      Clock_setTimeout(tempSensorMaxClkHandle,
                       (maxInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&tempSensorMaxClkStruct);
    }
  }
}

/*********************************************************************
 * @fn      SensorTagApp_sendTemp
 *
 * @brief   Called to send current temperature information to the thermostat
 *
 * @param   none
 *
 * @return  none
 */
  zclReportCmd_t  *pReportCmd;
static void SensorTagApp_sendTemp(void)
{
//  zclReportCmd_t  *pReportCmd;

  // Build and send a ZCL temperature reading to the matched device
  pReportCmd = ICall_malloc(sizeof(zclReportCmd_t) + sizeof(zclReport_t));
  if(pReportCmd != NULL)
  {
    // Fill in the single attribute information for the temperature reading
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_TEMPERATURE_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *) (&staTempMeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_TS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR, true, staTransID++);

    ICall_free(pReportCmd);
  }
}

/******************************************************************************
 * @fn      SensorTagApp_processHumiditySensorMinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processHumiditySensorMinTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_HUMIDITYSENSOR_MIN_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn      SensorTagApp_processHumiditySensorMaxTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processHumiditySensorMaxTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_HUMIDITYSENSOR_MAX_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn          SensorTagApp_setHumidityTimer
 *
 * @brief       Call function to Setup the next humidity reading
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_setHumiditySensorTimer(uint16_t minInterval,
                                            uint16_t maxInterval)
{
  // If the humidity timers are running, stop them
  if(Util_isClockActive(&humiditySensorMinClkStruct) == true)
  {
      Util_stopClock(&humiditySensorMinClkStruct);
  }

  // If maxInterval is 0, don't stop currently running max timer
  if((maxInterval != 0) &&
      (Util_isClockActive(&humiditySensorMaxClkStruct) == true))
  {
      Util_stopClock(&humiditySensorMaxClkStruct);
  }

  // Reconfigure Humidity Sensor timers in 10 usec increments
  if(maxInterval != ZCL_REPORTING_OFF)
  {
    if(minInterval != maxInterval)
    {
      // Don't run min timer if min int is the same as max int
      Clock_setTimeout(humiditySensorMinClkHandle,
                       (minInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&humiditySensorMinClkStruct);
    }

    if(maxInterval != 0)
    {
      // Restart the max timer if maxInterval is not 0
      Clock_setTimeout(humiditySensorMaxClkHandle,
                       (maxInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&humiditySensorMaxClkStruct);
    }
  }
}

/*********************************************************************
 * @fn      SensorTagApp_sendHumidity
 *
 * @brief   Called to send current humidity information to the thermostat
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_sendHumidity(void)
{
  zclReportCmd_t  *pReportCmd;

  // Build and send a ZCL humidity reading to the matched device
  pReportCmd = ICall_malloc(sizeof(zclReportCmd_t) + sizeof(zclReport_t));
  if(pReportCmd != NULL)
  {
    // Fill in the single attribute information for the humidity reading
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *) (&staHumidityMeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_TS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY, pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR, true, staTransID++);

    ICall_free(pReportCmd);
  }
}

/******************************************************************************
 * @fn      SensorTagApp_processPresSensorMinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processPresSensorMinTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_PRESSENSOR_MIN_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn      SensorTagApp_processPresSensorMaxTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processPresSensorMaxTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_PRESSENSOR_MAX_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn          SensorTagApp_setPresTimer
 *
 * @brief       Call function to Setup the next Pressure reading
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_setPresSensorTimer(uint16_t minInterval,
                                            uint16_t maxInterval)
{
  // If the Pres timers are running, stop them
  if(Util_isClockActive(&presSensorMinClkStruct) == true)
  {
      Util_stopClock(&presSensorMinClkStruct);
  }

  // If maxInterval is 0, don't stop currently running max timer
  if((maxInterval != 0) &&
      (Util_isClockActive(&presSensorMaxClkStruct) == true))
  {
      Util_stopClock(&presSensorMaxClkStruct);
  }

  // Reconfigure Pres Sensor timers in 10 usec increments
  if(maxInterval != ZCL_REPORTING_OFF)
  {
    if(minInterval != maxInterval)
    {
      // Don't run min timer if min int is the same as max int
      Clock_setTimeout(presSensorMinClkHandle,
                       (minInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&presSensorMinClkStruct);
    }

    if(maxInterval != 0)
    {
      // Restart the max timer if maxInterval is not 0
      Clock_setTimeout(presSensorMaxClkHandle,
                       (maxInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&presSensorMaxClkStruct);
    }
  }
}

/*********************************************************************
 * @fn      SensorTagApp_sendPressure
 *
 * @brief   Called to send current pressure information to the thermostat
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_sendPressure(void)
{
  zclReportCmd_t  *pReportCmd;

  // Build and send a ZCL pressure reading to the matched device
  pReportCmd = ICall_malloc(sizeof(zclReportCmd_t) + sizeof(zclReport_t));
  if(pReportCmd != NULL)
  {
    // Fill in the single attribute information for the pressure reading
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *) (&staPresMeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_PS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT, pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR, true, staTransID++);

    // Fill in the single attribute information for the pressure reading
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_PRESSURE_MEASUREMENT_SCALED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *) (&staPresScaledValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_PS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT, pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR, true, staTransID++);

    ICall_free(pReportCmd);
  }
}

#if defined(LIGHTSENSOR_ENABLED)
/******************************************************************************
 * @fn      SensorTagApp_processLightSensorMinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processLightSensorMinTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_LIGHTSENSOR_MIN_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn      SensorTagApp_processLightSensorMinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processLightSensorMaxTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_LIGHTSENSOR_MAX_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn          SensorTagApp_setIllumTimer
 *
 * @brief       Call function to Setup the next Illuminance reading
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_setLightSensorTimer(uint16_t minInterval,
                                             uint16_t maxInterval)
{
  // If the Light timers are running, stop them
  if(Util_isClockActive(&lightSensorMinClkStruct) == true)
  {
    Util_stopClock(&lightSensorMinClkStruct);
  }
  
  // If maxInterval is 0, don't stop currently running max timer
  if((maxInterval != 0) &&
      (Util_isClockActive(&lightSensorMaxClkStruct) == true))
  {
    Util_stopClock(&lightSensorMaxClkStruct);
  }
  
  // Reconfigure Light Sensor timers in 10 usec increments
  if(maxInterval != ZCL_REPORTING_OFF)
  {
    if(minInterval != maxInterval)
    {
      // Don't run min timer if min int is the same as max int
      Clock_setTimeout(lightSensorMinClkHandle,
                       (minInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&lightSensorMinClkStruct);
    }
    
    if(maxInterval != 0)
    {
      // Restart the max timer if maxInterval is not 0
      Clock_setTimeout(lightSensorMaxClkHandle,
                       (maxInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&lightSensorMaxClkStruct);
    }
  }
}

/*********************************************************************
 * @fn      SensorTagApp_sendIllum
 *
 * @brief   Called to send current illuminance information to the gateway
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_sendIllum(void)
{
  zclReportCmd_t *pReportCmd;

  // Build and send a ZCL illuminance reading to the matched device
  pReportCmd = ICall_malloc(sizeof(zclReportCmd_t) + sizeof(zclReport_t));
  if(pReportCmd != NULL)
  {
    // Fill in the single attribute information
    // for the illuminance reading
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_ILLUMINANCE_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&staIllumMeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_LS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
                      pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR,
                      true,
                      staTransID++);

    ICall_free(pReportCmd);
  }
}
#endif  // LIGHTSENSOR_ENABLED

#if defined(DUSTSENSOR_ENABLED)
/******************************************************************************
 * @fn      SensorTagApp_processDustSensorTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processDustSensorMaxTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_DUSTSENSOR_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn          SensorTagApp_setDustSensorTimer
 *
 * @brief       Call function to Setup the next Dust reading
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_setDustSensorTimer(uint16_t minInterval,
                                             uint16_t maxInterval)
{
  // If maxInterval is 0, don't stop currently running max timer
  if((maxInterval != 0) &&
      (Util_isClockActive(&dustSensorMaxClkStruct) == true))
  {
    Util_stopClock(&dustSensorMaxClkStruct);
  }
  
  // Reconfigure Dust Sensor timer in 10 usec increments
  if(maxInterval != ZCL_REPORTING_OFF)
  {
    if(maxInterval != 0)
    {
      // Restart the max timer if maxInterval is not 0
      Clock_setTimeout(dustSensorMaxClkHandle,
                       (maxInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&dustSensorMaxClkStruct);
    }
  }
}

/*********************************************************************
 * @fn      SensorTagApp_sendDust
 *
 * @brief   Called to send current dust information to the gateway
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_sendDust(void)
{
  zclReportCmd_t *pReportCmd;

  // Build and send a ZCL illuminance reading to the matched device
  pReportCmd = ICall_malloc(sizeof(zclReportCmd_t) + sizeof(zclReport_t));
  if(pReportCmd != NULL)
  {
    // Fill in the single attribute information
    // for the dust reading
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_DUST_1UM_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&staDust1umMeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_GSDS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_DUST_MEASUREMENT,
                      pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR,
                      true,
                      staTransID++);

    pReportCmd->attrList[0].attrID = ATTRID_MS_DUST_25UM_MEASURED_VALUE;
    pReportCmd->attrList[0].attrData = (void *)(&staDust25umMeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_GSDS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_DUST_MEASUREMENT,
                      pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR,
                      true,
                      staTransID++);

    ICall_free(pReportCmd);
  }
}
#endif  // DUSTSENSOR_ENABLED

#if defined(GASSENSOR_ENABLED)
/******************************************************************************
 * @fn      SensorTagApp_processGasSensorMaxTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_processGasSensorMaxTimeoutCallback(UArg a0)
{
  (void) a0;  // Parameter is not used

  events |= SENSORTAGAPP_GASSENSOR_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/******************************************************************************
 * @fn          SensorTagApp_setGasSensorTimer
 *
 * @brief       Call function to Setup the next Gas reading
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_setGasSensorTimer(uint16_t minInterval,
                                             uint16_t maxInterval)
{
  // If maxInterval is 0, don't stop currently running max timer
  if((maxInterval != 0) &&
      (Util_isClockActive(&gasSensorMaxClkStruct) == true))
  {
    Util_stopClock(&gasSensorMaxClkStruct);
  }
  
  // Reconfigure Gas Sensor timer in 10 usec increments
  if(maxInterval != ZCL_REPORTING_OFF)
  {
    if(maxInterval != 0)
    {
      // Restart the max timer if maxInterval is not 0
      Clock_setTimeout(gasSensorMaxClkHandle,
                       (maxInterval * 1000 * TIMER_MS_ADJUSTMENT));
      Util_startClock(&gasSensorMaxClkStruct);
    }
  }
}

/*********************************************************************
 * @fn      SensorTagApp_sendGas
 *
 * @brief   Called to send current gas information to the gateway
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagApp_sendGas(void)
{
  zclReportCmd_t *pReportCmd;

  // Build and send a ZCL illuminance reading to the matched device
  pReportCmd = ICall_malloc(sizeof(zclReportCmd_t) + sizeof(zclReport_t));
  if(pReportCmd != NULL)
  {
    // Fill in the single attribute information
    // for the dust reading
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_GAS_NH3_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&staGasNH3MeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_GSDS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
                      pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR,
                      true,
                      staTransID++);

    pReportCmd->attrList[0].attrID = ATTRID_MS_GAS_CO_MEASURED_VALUE;
    pReportCmd->attrList[0].attrData = (void *)(&staGasCOMeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_GSDS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
                      pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR,
                      true,
                      staTransID++);

    pReportCmd->attrList[0].attrID = ATTRID_MS_GAS_NO2_MEASURED_VALUE;
    pReportCmd->attrList[0].attrData = (void *)(&staGasNO2MeasuredValue);

    // Call ZCL function to send the report
    zcl_SendReportCmd(SENSORTAGAPP_GSDS_EP, &staDstAddr,
                      ZCL_CLUSTER_ID_MS_GAS_MEASUREMENT,
                      pReportCmd,
                      ZCL_FRAME_SERVER_CLIENT_DIR,
                      true,
                      staTransID++);

    ICall_free(pReportCmd);
  }
}
#endif  // GASSENSOR_ENABLED

/******************************************************************************
 * @fn      SensorTagApp_initReportConfig
 *
 * @brief   report configurations initialization function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SensorTagApp_initReportConfig(void)
{
  int i;
  
  staTempMeasuredValueOld = 0;
  staTempReportConfig.pAttr = &staTempMeasuredValue;
  staTempReportConfig.pAttrOld = &staTempMeasuredValueOld;
  staTempReportConfig.configTimer = SensorTagApp_setTempSensorTimer;
  staReportConfigList[0] = &staTempReportConfig;

 
  staHumidityMeasuredValueOld = 0;
  staHumidityReportConfig.pAttr = &staHumidityMeasuredValue;
  staHumidityReportConfig.pAttrOld = &staHumidityMeasuredValueOld;
  staHumidityReportConfig.configTimer = SensorTagApp_setHumiditySensorTimer;
  staReportConfigList[1] = &staHumidityReportConfig;
  
  staPresScaledValueOld = 0;
  staPresReportConfig.pAttr = &staPresScaledValue;
  staPresReportConfig.configTimer = SensorTagApp_setPresSensorTimer;
  staReportConfigList[2] = &staPresReportConfig;
  
#if defined(LIGHTSENSOR_ENABLED)
  staIllumMeasuredValueOld = 0;
  staIllumReportConfig.pAttr = &staIllumMeasuredValue;
  staIllumReportConfig.pAttrOld = &staIllumMeasuredValueOld;
  staIllumReportConfig.configTimer = SensorTagApp_setLightSensorTimer;
  staReportConfigList[3] = &staIllumReportConfig;
#endif  // LIGHTSENSOR_ENABLED

#if defined(DUSTSENSOR_ENABLED)
  staDustReportConfig.pAttr = &staDust1umMeasuredValue;
  staDustReportConfig.configTimer = SensorTagApp_setDustSensorTimer;
  staReportConfigList[4] = &staDustReportConfig;
#endif  // DUSTSENSOR_ENABLED

#if defined(GASSENSOR_ENABLED)
  staGasReportConfig.pAttr = &staGasNH3MeasuredValue;
  staGasReportConfig.configTimer = SensorTagApp_setGasSensorTimer;
  staReportConfigList[5] = &staGasReportConfig;
#endif  // GASSENSOR_ENABLED

  for (i = 0; i < MAX_REPORT_ATTR; i ++)
  {
    // Set default values.  initializeNVItem will only write if not in NVRAM
    staReportConfigList[i]->minReportInt = MIN_HA_MAX_REPORTING_INT;
    staReportConfigList[i]->maxReportInt = MIN_HA_MAX_REPORTING_INT;
    staReportConfigList[i]->reportableChange = 0;
    zclport_initializeNVItem(ZCL_PORT_REPORT_NV_ID, i, 2*3, staReportConfigList[i]);
    zclport_readNV(ZCL_PORT_REPORT_NV_ID, i, 0, 2*3, staReportConfigList[i]);
    staReportConfigList[i]->configTimer(staReportConfigList[i]->minReportInt, staReportConfigList[i]->maxReportInt);
  }
}

/******************************************************************************
 * @fn        SensorTagApp_getReportConfig
 *
 * @brief     Call function to get the matching reporting configuration entry
 *
 * @param     pAttr, id
 *
 * @return    reporting configuration entry
 */
static reportConfig_t* SensorTagApp_getReportConfig(zclAttribute_t* pAttr, int* id)
{
  uint8_t i;

  for (i = 0; i < MAX_REPORT_ATTR; i++)
  {
    if(pAttr->dataPtr == staReportConfigList[i]->pAttr)
    {
      *id = i;
      return staReportConfigList[i];
    }
  }

  return NULL;
}
#endif  // ZCL_REPORT

/******************************************************************************
 * @fn          SensorTagApp_applyFactoryImage
 *
 * @brief       Load the factory image for the SensorTag from external flash
 *              and reboot.
 *
 * @param       none
 *
 * @return      none
 */
static void SensorTagApp_applyFactoryImage(void)
{
  Power_setConstraint(Power_SB_DISALLOW);

  if(SensorTagApp_hasFactoryImage())
  {
    // Launch factory image; page 31 must be ommitted
    ((void (*)(uint32_t, uint32_t, uint32_t))SENSORTAGAPP_BL_OFFSET)
                          (EFL_ADDR_RECOVERY,EFL_SIZE_RECOVERY-0x1000, 0);
  }

  Power_releaseConstraint(Power_SB_DISALLOW);
}

/******************************************************************************
 * @fn          SensorTagApp_hasFactoryImage
 *
 * @brief       Is the factory image available?
 *
 * @param       none
 *
 * @return      none
 */
static bool SensorTagApp_hasFactoryImage(void)
{
#if defined(EXTERNAL_IMAGE_CHECK)
  bool  valid;
  valid = extFlashOpen();

  if(valid)
  {
    uint16_t  buffer[2];

    // 1. Check reset vector
    valid = extFlashRead(EFL_ADDR_RECOVERY, sizeof(buffer),
                         (uint8_t *) buffer);
    if(valid)
    {
      valid = (buffer[0] != 0xFFFF && buffer[1] != 0xFFFF) &&
              (buffer[0] != 0x0000 && buffer[1] != 0x0000);
    }

    extFlashClose();
  }

  return valid;
#else
  return (true);
#endif
}

/******************************************************************************
 ******************************************************************************/
