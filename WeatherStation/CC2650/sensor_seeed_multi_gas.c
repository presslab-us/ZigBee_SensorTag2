/*******************************************************************************
*  Filename:       sensor_seeed_multi_gas.c
*  Revised:        $Date: 2014-02-05 10:47:02 +0100 (on, 05 feb 2014) $
*  Revision:       $Revision: 12066 $
*
*  Description:    Driver for SEEED Studio multichannel gas sensor
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* -----------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------
*/
#include "sensor_seeed_multi_gas.h"
#include "sensor.h"
#include "bsp_i2c.h"
#include "math.h"

/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/
// Sensor I2C address
#define SENSOR_I2C_ADDRESS         0x04

// Registers
#define SENSOR_REG_R0_NH3          0x11 // R0 (calibrated zero) of NH3 sensor
#define SENSOR_REG_R0_RED          0x12 // R0 (calibrated zero) of RED sensor
#define SENSOR_REG_R0_OX           0x13 // R0 (calibrated zero) of OX sensor

#define SENSOR_REG_R_NH3           0x01 // R (measurement) of NH3 sensor
#define SENSOR_REG_R_RED           0x02 // R (measurement) of RED sensor
#define SENSOR_REG_R_OX            0x03 // R (measurement) of OX sensor

#define SENSOR_REG_POWER_OFF       0x20 // Power off sensor
#define SENSOR_REG_POWER_ON        0x21 // Power on sensor
#define SENSOR_REG_CALIBRATE       0x22 // Start calibration
#define SENSOR_REG_CHANGE_ADDR     0x23 // Change I2C address

// Sensor selection/deselection
bool SENSOR_SELECT(void)
{
  bool status = bspI2cSelect(BSP_I2C_INTERFACE_2,SENSOR_I2C_ADDRESS);
  bspI2cWriteSingle(0); // write dummy byte
  return status;
}

#define SENSOR_DESELECT()   bspI2cDeselect()

/* -----------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------
*/
typedef struct
{
  uint16_t r0_nh3;
  uint16_t r0_red;
  uint16_t r0_ox;
  uint16_t r_nh3;
  uint16_t r_red;
  uint16_t r_ox;
} SensorData_t;

/* -----------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------
*/

static bool sensorSeeedGasGetWord(uint8_t addr, void *pBuf);

/* -----------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------
*/
static bool success;
static SensorData_t data;

/*******************************************************************************
* @fn          sensorSeeedGasGetWord
*
* @brief       Reads a 16 bit word from the sensor
*
* @param       addr - address to read
*
* @param       pBuf - pointer to buffer to receive word
*
* @return      True if I2C operation successful
*******************************************************************************/
bool sensorSeeedGasGetWord(uint8_t addr, void *pBuf)
{
  uint8_t checksum;
  uint8_t buffer[4];
  
  success = bspI2cWriteSingle(addr);
  if (!success) return success;

  delay_ms(2);

  success = bspI2cRead((uint8_t*)&buffer,sizeof(buffer));
  if (!success) return success;
  
  checksum = (uint8_t)buffer[0] + buffer[1] + buffer[2];
  success = (checksum == buffer[3]);
  if (!success) return success;
  
  ((uint8_t *)pBuf)[1] = buffer[1];
  ((uint8_t *)pBuf)[0] = buffer[2];

   return success;
}

/*******************************************************************************
* @fn          sensorSeeedGasInit
*
* @brief       Initialise the SEEED gas sensor driver
*
* @return      True if I2C operation successful
*******************************************************************************/
bool sensorSeeedGasInit(void)
{
  if (!SENSOR_SELECT())
    return false;

  success = sensorWriteReg(SENSOR_REG_POWER_ON, 0, 0);
  if (!success) goto exit;
  success = sensorSeeedGasGetWord(SENSOR_REG_R0_NH3, &data.r0_nh3);
  if (!success) goto exit;
  success = sensorSeeedGasGetWord(SENSOR_REG_R0_RED, &data.r0_red);
  if (!success) goto exit;
  success = sensorSeeedGasGetWord(SENSOR_REG_R0_OX, &data.r0_ox);
  if (!success) goto exit;

  success = (data.r0_red != 0) && (data.r0_ox != 0) && (data.r0_nh3 != 0);

exit:
  SENSOR_DESELECT();

  return success;
}

/*******************************************************************************
* @fn          sensorSeeedGasCalibrate
*
* @brief       Zeros the calibration of the SEEED gas sensor driver
*
* @return      True if I2C operation successful
*******************************************************************************/
bool sensorSeeedGasCalibrate(void)
{
  if (!SENSOR_SELECT())
    return false;

  success = sensorWriteReg(SENSOR_REG_CALIBRATE, 0, 0);
  SENSOR_DESELECT();

  if (!success) goto exit;

  delay_ms(8000);

  success = sensorSeeedGasInit();

exit:
  return success;
}

/*******************************************************************************
* @fn          sensorSeeedGasRead
*
* @brief       Get sensor data
*
* @return      true of I2C operations successful
*/
bool sensorSeeedGasRead(void)
{
  if (!success)
    return success;

  success = SENSOR_SELECT();

  if (!success) goto exit;
  success = sensorSeeedGasGetWord(SENSOR_REG_R_NH3, &data.r_nh3);
  if (!success) goto exit;
  success = sensorSeeedGasGetWord(SENSOR_REG_R_RED, &data.r_red);
  if (!success) goto exit;
  success = sensorSeeedGasGetWord(SENSOR_REG_R_OX, &data.r_ox);
  if (!success) goto exit;

exit:
  SENSOR_DESELECT();

  return success;
}

/*******************************************************************************
* @fn          sensorSeeedGasConvert
*
* @brief       Convert raw data to desired gas
*
* @param       gasType - type of gas to convert sensor results into
*
* @return      float - result
*******************************************************************************/
float sensorSeeedGasConvert(enum gasType_t gasType)
{
  if (!success)
    return success;

  float ratioNH3 = (float)data.r_nh3 / data.r0_nh3;
  float ratioRED = (float)data.r_red / data.r0_red;
  float ratioOX = (float)data.r_ox / data.r0_ox;

  switch (gasType)
  {
  case GAS_NH3:
    if(ratioNH3 < 0.04) ratioNH3 = 0.04;
    if(ratioNH3 > 1.1) ratioNH3 = 1.1;
    //return 1 / (ratioNH3 * ratioNH3 * pow(10, 0.4));
    return pow(ratioNH3, -1.67)/1.47;  //modi by jack
  case GAS_CO:
    if(ratioRED < 0.01) ratioRED = 0.01;
    if(ratioRED > 3) ratioRED = 3;
    //return pow(10, 0.6) / pow(ratioRED, 1.2);
    //return pow(ratioRED, -1.179)*4.385;  //mod by jack
    return pow(ratioRED, -1.179)*0.2;  //mod by ryan
  case GAS_NO2:
    if(ratioOX < 0.07) ratioOX = 0.07;
    if(ratioOX > 70) ratioOX = 70;
    //return ratioOX / pow(10, 0.8);
    //return pow(ratioOX, 1.007)/6.855;  //mod by jack
    return pow(ratioOX, 1.007)/20.0;  //mod by ryan
  }
/*
        case C3H8:  //add by jack
        {
            if(ratio0 < 0.23) ratio0 = 0.23;
            if(ratio0 > 0.8) ratio0 = 0.8;
            c = pow(ratio0, -2.518)*570.164;
            break;
        }
        case C4H10:  //add by jack
        {
            if(ratio0 < 0.15) ratio0 = 0.15;
            if(ratio0 > 0.65) ratio0 = 0.65;
            c = pow(ratio0, -2.138)*398.107;
            break;
        }
        case CH4:  //add by jack
        {
            if(ratio1 < 0.5) ratio1 = 0.5;
            if(ratio1 > 0.7) ratio1 = 0.7;
            c = pow(ratio1, -4.363)*630.957;
            break;
        }
        case H2:  //add by jack
        {
            if(ratio1 < 0.04) ratio1 = 0.04;
            if(ratio1 > 0.8) ratio1 = 0.8;
            c = pow(ratio1, -1.8)*0.73;
            break;
        }
        case C2H5OH:  //add by jack
        {
            if(ratio1 < 0.04) ratio1 = 0.04;
            if(ratio1 > 1.1) ratio1 = 1.1;
            c = pow(ratio1, -1.552)*1.622;
            break;
        }
        default:
*/
  return -1;
}
