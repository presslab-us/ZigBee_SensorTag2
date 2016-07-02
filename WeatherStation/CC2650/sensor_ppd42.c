/*******************************************************************************
*  Filename:       sensor_ppd42.c
*/


/* -----------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------
*/
#include "sensor_ppd42.h"
#include "sensor.h"

/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------
*/
typedef struct
{
  uint16_t ppm25;
  uint16_t ppm10;
} SensorData_t;

/* -----------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------
*/

/*******************************************************************************
* @fn          sensorPpd42Init
*
* @brief       Initialise the PPD42 dust sensor driver
*
* @return      True if operations successful
*******************************************************************************/
bool sensorPpd42Init(void)
{
  return true;
}

/*******************************************************************************
* @fn          sensorPpd42Start
*
* @brief       Start measurement
*
* @return      none
*/
void sensorPpd42Start(void)
{
}


/*******************************************************************************
* @fn          sensorPpd42Read
*
* @brief       Get dust sensor data
*
* @param       lpo10 - low pulse occupancy 1.0 micron
*
* @param       lpo25 - low pulse occupancy 2.5 micron
*
* @return      true if operations successful
*/
bool sensorPpd42Read(uint16_t *lpo10, uint16_t *lpo25)
{
  bool valid;

  // Store lpo of 2.5
  *lpo25 = 0;

  // Store lpo of 1.0
  *lpo10 = 0;

  valid = true;

  return valid;
}

/*******************************************************************************
* @fn          sensorPpd42Convert
*
* @brief       Convert raw data to dust PPM
*
* @param       lpo10 - raw lpo 10 value
*
* @param       lpo25 - raw lpo 25 value
*
* @param       ppm10 - converted PPM 1.0 micron
*
* @param       ppm25 - converted PPM 2.5 micron
*
* @return      none
*******************************************************************************/
void sensorPpd42Convert(uint16_t lpo10, uint16_t lpo25,
                        uint16_t *ppm10, uint16_t *ppm25)
{
  *ppm10 = *ppm25 = 0;
}
