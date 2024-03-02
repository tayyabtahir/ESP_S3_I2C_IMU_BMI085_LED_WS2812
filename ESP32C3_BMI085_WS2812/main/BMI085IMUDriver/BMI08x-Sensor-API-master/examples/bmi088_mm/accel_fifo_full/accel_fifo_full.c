/**
 * Copyright (C) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "bmi088_mm.h"
#include "common.h"

/*********************************************************************/
/*                            Macros                                 */
/*********************************************************************/

/* Buffer size allocated to store raw FIFO data */
#define BMI08_FIFO_RAW_DATA_BUFFER_SIZE        UINT16_C(1024)

/* Length of data to be read from FIFO */
#define BMI08_FIFO_RAW_DATA_USER_LENGTH        UINT16_C(1024)

/* Number of Gyro frames to be extracted from FIFO */
#define BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(100)

/* Sensortime resolution in seconds */
#define SENSORTIME_RESOLUTION                  0.0000390625f

/*********************************************************************/
/*                       Function Declarations                       */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi08 sensor
 */
static void init_bmi08(struct bmi08_dev *bmi08dev);

/*********************************************************************/
/*                             Functions                             */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08 sensor
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* Initialize bmi08 sensors (accel & gyro)*/
    if (bmi088_mma_init(bmi08dev) == BMI08_OK && bmi08g_init(bmi08dev) == BMI08_OK)
    {
        printf("BMI08 initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08dev->accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08dev->gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(bmi08dev);
    }
    else
    {
        printf("BMI08 initialization failure!\n");
        exit(COINES_E_FAILURE);
    }

    if (rslt == BMI08_OK)
    {
        /*! Max read/write length (maximum supported length is 32).
         * To be set by the user */
        bmi08dev->read_write_len = 32;

        /* Set accel power mode */
        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
        bmi08dev->accel_cfg.odr = BMI08_ACCEL_ODR_200_HZ;
        bmi08dev->accel_cfg.range = BMI088_MM_ACCEL_RANGE_6G;
        rslt = bmi088_mma_set_meas_conf(bmi08dev);
        bmi08_check_rslt("bmi088_mma_set_meas_conf", rslt);
    }
}

static void configure_bmi08_fifo_full_interrupt(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi08_accel_fifo_config config;
    struct bmi08_accel_int_channel_cfg int_config;

    /* Configure the Interrupt configurations */
    int_config.int_channel = BMI08_INT_CHANNEL_2;
    int_config.int_type = BMI088_MM_ACCEL_INT_FIFO_FULL;
    int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Set the interrupt configuration */
    rslt = bmi088_mma_set_int_config(&int_config, BMI088_MM_ACCEL_INT_FIFO_FULL, bmi08dev);
    bmi08_check_rslt("bmi088_mma_set_int_config", rslt);

    config.accel_en = BMI08_ENABLE;
    config.int2_en = BMI08_ENABLE;

    /* Set FIFO configuration by enabling accelerometer */
    rslt = bmi08a_set_fifo_config(&config, bmi08dev);
    bmi08_check_rslt("bmi08a_set_fifo_config", rslt);
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 */
int main(void)
{
    struct bmi08_dev bmi08;
    int8_t rslt;
    uint8_t status = 0;
    uint8_t try = 1;
    uint16_t idx = 0;
    uint16_t accel_length = BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT;
    uint16_t fifo_length = 0;

    /* Variable to store sensor time value */
    uint32_t sensor_time;

    uint8_t fifo_data[BMI08_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };
    struct bmi08_fifo_frame fifo_frame = { 0 };
    struct bmi08_sensor_data bmi08_accel[BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT] = { { 0 } };

    /* Interface reference is given as a parameter
     *         For I2C : BMI08_I2C_INTF
     *         For SPI : BMI08_SPI_INTF
     */
    rslt = bmi08_interface_init(&bmi08, BMI08_I2C_INTF);
    bmi08_check_rslt("bmi08_interface_init", rslt);

    /* Initialize the sensors */
    init_bmi08(&bmi08);

    configure_bmi08_fifo_full_interrupt(&bmi08);

    /* Update FIFO structure */
    fifo_frame.data = fifo_data;
    fifo_frame.length = BMI08_FIFO_RAW_DATA_USER_LENGTH;

    while (try <= 10)
    {
        rslt = bmi08a_get_data_int_status(&status, &bmi08);
        bmi08_check_rslt("bmi08a_get_data_int_status", rslt);

        if (status & BMI08_ACCEL_FIFO_FULL_INT)
        {
            printf("\nIteration : %d\n", try);

            rslt = bmi08a_get_fifo_length(&fifo_length, &bmi08);
            bmi08_check_rslt("bmi08a_get_fifo_length", rslt);

            printf("Requested data frames before parsing: %d\n", accel_length);
            printf("FIFO length available : %d\n", fifo_length);

            if (rslt == BMI08_OK)
            {
                /* Read FIFO data */
                rslt = bmi08a_read_fifo_data(&fifo_frame, &bmi08);
                bmi08_check_rslt("bmi08a_read_fifo_data", rslt);

                /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
                rslt = bmi08a_extract_accel(bmi08_accel, &accel_length, &fifo_frame, &bmi08);
                bmi08_check_rslt("bmi08a_extract_accel", rslt);

                printf("Parsed accelerometer frames: %d\n", accel_length);

                /* Print the parsed accelerometer data from the FIFO buffer */
                for (idx = 0; idx < accel_length; idx++)
                {
                    printf("ACCEL[%d] X : %d\t Y : %d\t Z : %d\n",
                           idx,
                           bmi08_accel[idx].x,
                           bmi08_accel[idx].y,
                           bmi08_accel[idx].z);
                }

                rslt = bmi08a_get_sensor_time(&bmi08, &sensor_time);
                bmi08_check_rslt("bmi08a_get_sensor_time", rslt);

                printf("Sensor time : %.4lf   s\n", (sensor_time * SENSORTIME_RESOLUTION));
            }

            try++;
        }
    }

    bmi08_coines_deinit();

    return rslt;
}
