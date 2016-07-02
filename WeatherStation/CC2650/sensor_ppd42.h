/*******************************************************************************
*  Filename:       sensor_ppd42.h
*/

#ifndef SENSOR_PPD42_H
#define SENSOR_PPD42_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "stdbool.h"
#include "stdint.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * FUNCTIONS
 */
bool sensorPpd42Init(void);
void sensorPpd42Start(void);
bool sensorPpd42Read(uint16_t *lpo10, uint16_t *lpo25);
void sensorPpd42Convert(uint16_t lpo10, uint16_t lpo25, uint16_t *ppm10, uint16_t *ppm25);

/*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_PPD42_H */
