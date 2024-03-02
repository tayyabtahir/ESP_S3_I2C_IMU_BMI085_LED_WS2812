# ESP32_S3_I2C_IMU_BMI085_LED_WS2812

This sample will integrate an BMI085 IMU sensor and WS2812 LED with ESP32_S3.



## How To Compile the sample code.
To compile the sample you first need to install ESP IDF utility.

**Please follow the link here for furthur details**
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

Then procced to the code folder **.\GitHub\ESP32_S3_I2C_IMU_BMI085_LED_WS2812\ESP32S3_BMI085_WS2812>** and execute **idf.py build** command to compile.
## Connections

Make the following connections:

* **I2C Pins**
    * CONFIG_I2C_MASTER_SCL 19
    * CONFIG_I2C_MASTER_SDA 18

* **Led Pins**
    * LED_STRIP_BLINK_GPIO  2

## Application Details
A brief description on the applications involved

### ESP32_S3_I2C_IMU_BMI085_LED_WS2812
This sample code drives BMI085 IMU to spit out accelerometer and groscope data while toggling the led SK6812 via ESP32S3.
