 /** motion_sensor_hw.c
 *
 * This file contains the APIs related to the lsm9ds1 sensor
 * The lsm9ds1 sensor libraries can be found at:
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm9ds1_STdC
 * Commit ID: bb6d11f6fea38a08f680cb9f6cb5ed3f11f6e060
 *
 */

/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"
#include "lsm9ds1_drivers/lsm9ds1_reg.h"
#include "motion_sensor_hw.h"
#include "wiced_rtos.h"
#include "GeneratedSource/cycfg_gatt_db.h"

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                            Functions prototype
 ******************************************************************************/

static void init_LSM9DS1_ACC_GYRO(void);

static void init_LSM9DS1_MAG(void);

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/* Initialize magnetic sensor driver interface */
uint8_t i2c_add_mag = LSM9DS1_MAG_I2C_ADD_H >> 1;
lsm9ds1_ctx_t dev_ctx_mag = {platform_write, platform_read, (void*)&i2c_add_mag};

/* Initialize accelerometer and gyroscope driver interface */
uint8_t i2c_add_imu = LSM9DS1_IMU_I2C_ADD_L >> 1;
lsm9ds1_ctx_t dev_ctx_imu = {platform_write, platform_read, (void*)&i2c_add_imu};

 /******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/**
 * Function         init_LSM9DS1_ACC_GYRO
 *
 * @brief           This function initializes accelerometer and gyroscope
 *
 * @return          : None
 */
void init_LSM9DS1_ACC_GYRO(void)
{
    int32_t response;
    uint8_t value;
    lsm9ds1_id_t dev_id;
    lsm9ds1_xl_trshld_en_t enable_xl_threshold = {0, 1, 0, 1, 0, 1};
    uint8_t axis_threshold_value[3] = {XL_THRESHOLD, XL_THRESHOLD, XL_THRESHOLD};

  /* Gyro ODR and full scale */
  response = lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_119Hz);
  if(WICED_SUCCESS != response)
  {
      WICED_BT_TRACE("Failed to set Gyro Data Rate\r\n");
  }

  response = lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_245dps);
  if(WICED_SUCCESS != response)
  {
      WICED_BT_TRACE("Failed to set Gyro full scale\r\n");
  }

  response = lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_8g);
  if(WICED_SUCCESS != response)
  {
      WICED_BT_TRACE("Failed to set Acc full scale\r\n");
  }

  /* BDU Enable */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  if(WICED_SUCCESS != response)
  {
      WICED_BT_TRACE("Failed to set BDU\r\n");
  }

  /* Enable Acc interrupts to send periodic notifications */
  lsm9ds1_pin_logic_set(&dev_ctx_imu, LSM9DS1_LOGIC_OR);
  lsm9ds1_xl_trshld_axis_set(&dev_ctx_imu, enable_xl_threshold);
  lsm9ds1_pin_polarity_set(&dev_ctx_mag, &dev_ctx_imu, LSM9DS1_ACTIVE_HIGH);

  /* Set thresholds for interrupts */
  lsm9ds1_xl_trshld_set(&dev_ctx_imu, axis_threshold_value);

}

/**
 * Function         init_LSM9DS1_MAG
 *
 * @brief           This function initializes magnetometer
 *
 * @return          : None
 */
void init_LSM9DS1_MAG(void)
{
    int32_t response;

    response = lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
  if(WICED_SUCCESS != response)
  {
      WICED_BT_TRACE("Failed to set Mag full scale\r\n");
  }

  response = lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_HP_10Hz);
  if(WICED_SUCCESS != response)
  {
      WICED_BT_TRACE("Failed to set Mag ODR\r\n");
  }
}

/**
 * Function         motion_sensor_init
 *
 * @brief           This function initializes motion sensors ( accelerometer,
 *                  gyro and magnetic sensor)
 *
 * @return          : None
 */
void motion_sensor_init (void)
{
    uint8_t rst;
    /* Restore default configuration */
    lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
    do
    {
        lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
    } while (rst);

    init_LSM9DS1_ACC_GYRO();
    init_LSM9DS1_MAG();
}

/**
 * Function         get_accel_val
 *
 * @brief           This function gets accelerometer value
 *
 * @param[in] accel_struct       : Pointer to store accelerometer value
 *
 * @return                       : None
 */
void get_accel_val (axis3bit16_t* accel_struct)
{
    uint8_t value_XL;
    float            final_value = 0;
    char             sign = '\0';

    /*Read ACC output only if new ACC value is available */
    lsm9ds1_xl_flag_data_ready_get(&dev_ctx_imu, &value_XL);

    if (DATA_READY == value_XL)
    {
        lsm9ds1_acceleration_raw_get(&dev_ctx_imu, accel_struct->u8bit);

        final_value = lsm9ds1_from_fs8g_to_mg(accel_struct->i16bit[0]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("\r\nAcceleration X-Axis: %c%d.%02d mg\r\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));

      final_value = lsm9ds1_from_fs8g_to_mg(accel_struct->i16bit[1]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("Acceleration Y-Axis: %c%d.%02d mg\r\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));

      final_value = lsm9ds1_from_fs8g_to_mg(accel_struct->i16bit[2]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("Acceleration Z-Axis: %c%d.%02d mg\r\n\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));


    }
    else
    {
        WICED_BT_TRACE("Data Not Available\r\n");
    }
}

/**
 * Function         get_gyro_val
 *
 * @brief           This function gets gyroscope value
 *
 * @param[in] gyro_struct        : Pointer to store gyroscope value
 *
 * @return                       : None
 */
void get_gyro_val (axis3bit16_t* gyro_struct)
{
    uint8_t value_G;
    int32_t response;
    float            final_value = 0;
    char             sign = '\0';

    response = lsm9ds1_gy_flag_data_ready_get(&dev_ctx_imu, &value_G);
    if(WICED_SUCCESS != response)
    {
        WICED_BT_TRACE("Failed to get Acc data ready flag\r\n");
    }

    if (DATA_READY==value_G)
    {
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, gyro_struct->u8bit);

      final_value = lsm9ds1_from_fs245dps_to_mdps(gyro_struct->i16bit[0]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("\r\nGyro X-Axis: %c%d.%02d dps\r\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));

      final_value = lsm9ds1_from_fs245dps_to_mdps(gyro_struct->i16bit[1]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("Gyro Y-Axis: %c%d.%02d dps\r\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));

      final_value = lsm9ds1_from_fs245dps_to_mdps(gyro_struct->i16bit[2]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("Gyro Z-Axis: %c%d.%02d dps\r\n\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));

    }
}

/**
 * Function         get_mag_val
 *
 * @brief           This function gets magnetometer value
 *
 * @param[in] gyro_struct        : Pointer to store magnetometer value
 *
 * @return                       : None
 */
void get_mag_val (axis3bit16_t* mag_struct)
{
    uint8_t value_M;
    int32_t response;
    float            final_value = 0;
    char             sign = '\0';

/*Read MAG output only if new value is available */
response =  lsm9ds1_mag_flag_data_ready_get(&dev_ctx_mag, &value_M);
if(WICED_SUCCESS != response)
{
      WICED_BT_TRACE("Failed to get Gyro data ready flag\r\n");
}

if (DATA_READY==value_M)
    {
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, mag_struct->u8bit);

      final_value = lsm9ds1_from_fs16gauss_to_mG(mag_struct->i16bit[0]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("\r\nMag X-Axis: %c%d.%02d mG\r\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));

      final_value = lsm9ds1_from_fs16gauss_to_mG(mag_struct->i16bit[1]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("Mag Y-Axis: %c%d.%02d mG\r\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));

      final_value = lsm9ds1_from_fs16gauss_to_mG(mag_struct->i16bit[2]);

      if(final_value < 0)
      {
          sign = '-';
      }
      else
      {
          sign = '+';
      }

      WICED_BT_TRACE("Mag Z-Axis: %c%d.%02d mG\r\n\n", sign, ABS((int16_t)final_value),
                                               ABS((int)(final_value*100)%100));
    }
}

/**
 * Function         motion_sensor_update_gatt
 *
 * @brief           This function reads motion sensors data and updates the
 *                  value in Gatt DB
 *
 * @return                       : None
 */
void motion_sensor_update_gatt()
{
    get_accel_val(&((axis3bit16_t *)(app_sensor_hub_motion_sensor_notify))[0]);

    get_gyro_val(&((axis3bit16_t *)(app_sensor_hub_motion_sensor_notify))[1]);

    get_mag_val(&((axis3bit16_t *)(app_sensor_hub_motion_sensor_notify))[2]);
}

/**
 * Function          platform_write
 *
 * @brief            Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
    uint8_t reg_data_bytes[len + 1];
    uint8_t result;
    uint8_t *i2c_address = handle;

    reg_data_bytes[0] = reg;
    memcpy(&reg_data_bytes[1], bufp, len);

    result = wiced_hal_i2c_write(reg_data_bytes, sizeof(reg_data_bytes), *i2c_address);
    return result;
}

/**
 * Function          platform_read
 *
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    UINT8 result;
    uint8_t *i2c_address = handle;

    result = wiced_hal_i2c_combined_read(bufp, len, &reg, sizeof(reg), *i2c_address);
    return result;
}
