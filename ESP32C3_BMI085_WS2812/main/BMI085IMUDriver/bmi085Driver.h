/**
*
* @file       bmi085Driver.h
* @date       2023-07-2
* @version    v1.0.0
*
*/

#ifndef _ACCEL_GYRO_OIS_H
#define _ACCEL_GYRO_OIS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/* header files */
/*********************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "BMI085IMUDriver/BMI08x-Sensor-API-master/bmi08_defs.h"

/*********************** BMI085 function prototypes ************************/

/**
 * @details Sensor initialization
 */
void InitializeBMI085(void);

/**
 * @details Sensor deinitialization
 */
void DeInitializeBMI085(void);

/**
 * @details This API reads the gyro data from the sensor,
 * store it in the bmi08x_sensor_data structure instance
 * passed by the user.
 */
int ReadBMI085SensorValues(void);

#ifdef __cplusplus
}
#endif

#endif /* _ACCEL_GYRO_OIS_H */
