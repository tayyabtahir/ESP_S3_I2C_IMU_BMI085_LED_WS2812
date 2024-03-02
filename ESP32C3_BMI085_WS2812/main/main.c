//==============================================================================
//
//  Main.c
//
//  This is a main file for the project.
//
//  (c) 2023 
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
//
//==============================================================================
//  FILE INFORMATION
//==============================================================================
//
//  Source:        main.c
//
//  Project:       ESP32S3
//
//  Author:        
//
//  Date:          2023/07/02
//
//  Revision:      1.0
//
//==============================================================================
//  FILE DESCRIPTION
//==============================================================================
//
//! \file
//! This is main module for ESP32S3 IMU Project build01. The main functionality
//! of this module is initialization, managing activities and deinitialization
//
//==============================================================================
//  REVISION HISTORY
//==============================================================================
//  Revision: 1.0  2023/07/02
//
//==============================================================================
//  INCLUDES
//==============================================================================

#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "BMI085IMUDriver/bmi085Driver.h"
#include "RMTLEDDriver/LedDriver.h"

static const char *TAG = "MAIN";

#define LOOP_ITERATIONS                     100

void app_main(void)
{
    
    uint32_t counter;

    InitializeBMI085();
    ESP_LOGI(TAG, "I2C initialized successfully");

    InitializeLEDDriver();
    ESP_LOGI(TAG, "SK6812 initialized successfully");

    for(counter = 0; counter < LOOP_ITERATIONS; counter++)
    {
        /* Read Sensor Data*/
        ReadBMI085SensorValues();
        /* Toggle Led*/
        ToggelLed();
        /* Sleep for 2 seconds */
        esp_rom_delay_us(2000000);
    }
    /* TODO: ADD functionality as per user requirement*/
    
    DeInitializeBMI085();
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
