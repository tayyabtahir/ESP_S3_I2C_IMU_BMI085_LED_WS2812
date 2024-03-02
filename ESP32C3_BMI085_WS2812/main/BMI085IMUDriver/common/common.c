//==============================================================================
//
//  common.c
//
//  This is a driver file for the IMU sensor Integration.
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
//  Source:        common.c
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
//! This is driver file for BMI085 IMU. The main functionality
//! of this module is provide initialization and deinitialization of the sensor and
//! manage reading data from it as well.
//
//==============================================================================
//  REVISION HISTORY
//==============================================================================
//  Revision: 1.0  2023/07/02
//
//==============================================================================

#include "common.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

static const char* TAG = "BMI085 COMMON";

#define BMI08_READ_WRITE_LEN  UINT8_C(46)
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_REG_ADD_BYTE_SIZE       1
#define I2C_REG_ADD_BYTE_LOCATION   0
#define I2C_REG_DATA_LOCATION       1


/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address for gyro */
uint8_t gyro_dev_add;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to ESP platform
 */
BMI08_INTF_RET_TYPE bmi08_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    // esp_err_t i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_address, uint8_t *read_buffer, size_t read_size, TickType_t ticks_to_wait)
    // esp_err_t i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, TickType_t ticks_to_wait)
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg_addr, 1, reg_data, len, I2C_MASTER_TIMEOUT_MS);

    //return coines_read_i2c(COINES_I2C_BUS_0, dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to ESP platform
 */
BMI08_INTF_RET_TYPE bmi08_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    if (len > BMI08_READ_WRITE_LEN)
        return -1;

    /* Buffer to store the register address to write + the data to be written*/
    uint8_t write_buf[I2C_REG_ADD_BYTE_SIZE + BMI08_READ_WRITE_LEN] = {0};
    write_buf[I2C_REG_ADD_BYTE_LOCATION] = reg_addr;
    memcpy(&write_buf[I2C_REG_DATA_LOCATION], reg_data, len);

    // i2c_master_write_to_device(I2C_MASTER_NUM, BMI085_SENSOR_ACC_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, write_buf, I2C_REG_ADD_BYTE_SIZE + len, I2C_MASTER_TIMEOUT_MS);
    //return coines_write_i2c(COINES_I2C_BUS_0, dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * Delay function map to ESP platform
 */
void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

/*!
 *  @brief Function to select the interface on I2C.
 *  Also to initialize coines platform
 */
int8_t bmi08_interface_init(struct bmi08_dev *bmi08, uint8_t intf, uint8_t variant)
{
    int8_t rslt = BMI08_OK;

    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = I2C_MASTER_SDA_IO,
                .scl_io_num = I2C_MASTER_SCL_IO,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = I2C_MASTER_FREQ_HZ,
            };

    if (bmi08 != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI08_I2C_INTF)
        {
            ESP_LOGI(TAG,"I2C Interface \n");

            /* To initialize the user I2C function */
            acc_dev_add = BMI08_ACCEL_I2C_ADDR_PRIMARY;
            gyro_dev_add = BMI08_GYRO_I2C_ADDR_PRIMARY;
            bmi08->intf = BMI08_I2C_INTF;
            bmi08->read = bmi08_i2c_read;
            bmi08->write = bmi08_i2c_write;

            i2c_param_config(i2c_master_port, &conf);
            rslt = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

            esp_rom_delay_us(1000*10);
        }
        /* Bus configuration : SPI */
        else if (intf == BMI08_SPI_INTF)
        {
            ESP_LOGE(TAG,"SPI Interface Not supported\n");
        }

        /* Selection of bmi085 or bmi088 sensor variant */
        bmi08->variant = (enum bmi08_variant)variant;

        /* Assign accel device address to accel interface pointer */
        bmi08->intf_ptr_accel = &acc_dev_add;

        /* Assign gyro device address to gyro interface pointer */
        bmi08->intf_ptr_gyro = &gyro_dev_add;

        /* Configure delay in microseconds */
        bmi08->delay_us = bmi08_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi08->read_write_len = BMI08_READ_WRITE_LEN;

        esp_rom_delay_us(10000);

        esp_rom_delay_us(10000);
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi08_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMI08_OK)
    {
        ESP_LOGI(TAG,"%s\t", api_name);
        if (rslt == BMI08_E_NULL_PTR)
        {
            ESP_LOGI(TAG,"Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMI08_E_COM_FAIL)
        {
            ESP_LOGI(TAG,"Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMI08_E_DEV_NOT_FOUND)
        {
            ESP_LOGI(TAG,"Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMI08_E_OUT_OF_RANGE)
        {
            ESP_LOGI(TAG,"Error [%d] : Out of Range\r\n", rslt);
        }
        else if (rslt == BMI08_E_INVALID_INPUT)
        {
            ESP_LOGI(TAG,"Error [%d] : Invalid input\r\n", rslt);
        }
        else if (rslt == BMI08_E_CONFIG_STREAM_ERROR)
        {
            ESP_LOGI(TAG,"Error [%d] : Config stream error\r\n", rslt);
        }
        else if (rslt == BMI08_E_RD_WR_LENGTH_INVALID)
        {
            ESP_LOGI(TAG,"Error [%d] : Invalid Read write length\r\n", rslt);
        }
        else if (rslt == BMI08_E_INVALID_CONFIG)
        {
            ESP_LOGI(TAG,"Error [%d] : Invalid config\r\n", rslt);
        }
        else if (rslt == BMI08_E_FEATURE_NOT_SUPPORTED)
        {
            ESP_LOGI(TAG,"Error [%d] : Feature not supported\r\n", rslt);
        }
        else if (rslt == BMI08_W_FIFO_EMPTY)
        {
            ESP_LOGI(TAG,"Warning [%d] : FIFO empty\r\n", rslt);
        }
        else
        {
            ESP_LOGI(TAG,"Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Deinitializes imu sensor channel
 *
 *  @return void.
 */
void bmi08_deinit(void)
{
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
