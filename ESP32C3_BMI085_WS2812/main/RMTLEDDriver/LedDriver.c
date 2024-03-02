//==============================================================================
//
//  LedDriver.c
//
//  This is a driver file for the SK6812 Led drive Integration.
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
//  Source:        LedDriver.c
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
//! This is driver file for SK6812 LED. The main functionality
//! of this module is provide initialization and deinitialization of the controlling
//! GPIO and color changing functionlity
//
//==============================================================================
//  REVISION HISTORY
//==============================================================================
//  Revision: 1.0  2023/07/02
//
//==============================================================================


/******************************************************************************/
/*!                 Header Files                                              */

#include "RMTLEDDriver/LedDriver.h"

/******************************************************************************/
/*!                Macro definition                                           */

static const char *TAG = "LED";
#define LED_STRIP_BLINK_GPIO  2             /* GPIO assignment */
#define LED_STRIP_LED_NUMBERS 1             /* Numbers of the LED in the strip */
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000) /* 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution) */


/******************************************************************************/
/*!                Global variables                                           */
/******************************************************************************/

led_strip_handle_t led_strip;
bool led_on_off = false;

/******************************************************************************/
/*!           Static Function Declaration                                     */
/******************************************************************************/

/******************************************************************************/
/*!            Functions                                                      */
/******************************************************************************/

/**
 * @brief Function to initiliaze the LED driver
 */
void InitializeLEDDriver(void)
{
    /* LED strip general initialization, according to your led board design */
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_SK6812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };

    // LED Strip object handle
    led_strip_handle_t localLed_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &localLed_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    led_strip = localLed_strip;
}

/**
 * @brief Function toggels LED On and off
 */
void ToggelLed(void)
{
    if (led_on_off) {
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 5, 5, 5));
    }
    /* Refresh the strip to send data */
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    ESP_LOGI(TAG, "LED ON!");
    } else {
        /* Set all LED off to clear all pixels */
        ESP_ERROR_CHECK(led_strip_clear(led_strip));
        ESP_LOGI(TAG, "LED OFF!");
    }

    led_on_off = !led_on_off;
}

/**
 * @brief Function sets LED color
 */
void SetLedColor(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    led_on_off = true;
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, index, red, green, blue));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    ESP_LOGI(TAG, "LED Set to color!");
}

/**
 * @brief Function turn off LED
 */
void TurnOffLed(void)
{
    ESP_LOGI(TAG, "LED OFF!");
    led_on_off = false;
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
}