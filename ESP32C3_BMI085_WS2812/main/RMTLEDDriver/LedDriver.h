/**
*
* @file       LedDriver.h
* @date       2023-07-2
* @version    v1.0.0
*
*/

#ifndef _LED_DRIVER_H
#define _LED_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/* header files */

#include <stdio.h>
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"


/*********************** BMI085 function prototypes ************************/

/**
 * @details Led initialization
 */
void InitializeLEDDriver(void);

/**
 * @brief Function toggels LED On and off
 */
void ToggelLed(void);

/**
 * @brief Function sets LED color
 */
void SetLedColor(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);

/**
 * @brief Function turn off LED
 */
void TurnOffLed(void);

#ifdef __cplusplus
}
#endif

#endif /* _LED_DRIVER_H */
