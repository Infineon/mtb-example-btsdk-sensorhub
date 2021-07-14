/*******************************************************************************
* File Name:   sensors.c
*
* Description: This file shows the implementation of  sensor and timer.
*                              
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
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
*******************************************************************************/

#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_gatt.h"
#include "wiced_result.h"
#include "wiced_timer.h"
#include "wiced_rtos.h"
#include "wiced_hal_i2c.h"
#include "wiced_hal_adc.h"
#include "bt_types.h"
#include "ble_server.h"
#include "sensors.h"

/*******************************************************************************
 *                                Structures
 ******************************************************************************/

/*******************************************************************************
 *                            Functions prototype
 ******************************************************************************/

static void motion_timer_callback(uint32_t arg);

static void mag3d_timer_callback(uint32_t arg);

static void temp_timer_callback(uint32_t arg);
/*******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

extern host_info_t ble_server_hostinfo;

/* Timer handles */
static wiced_timer_t motion_timer_handle;
static wiced_timer_t mag3d_timer_handle;
static wiced_timer_t temp_timer_handle;

/*******************************************************************************
 * Function Name: sensors_init
 *******************************************************************************
 * Summary   : Initialize the necessary peripherals I2C , 
 *                                           ADC and Timers for sensors
 * Parameter : None
 * Return    : None
 ******************************************************************************/
void sensors_init(void)
{
   /* Initialize I2C */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    /* Initialize the necessary peripherals (ADC) */
    wiced_hal_adc_init();

    /* Initialize Motion sensor */
    motion_sensor_init();

    /* Initialize Magnetic sensor */
    mag3d_sensor_init();

    if(mag3d_sensor_isConnected())
    {
        /* Initialize 3D magnetic  sense timer*/
        if(WICED_BT_SUCCESS != wiced_init_timer(&mag3d_timer_handle,
               &mag3d_timer_callback, 0u, WICED_MILLI_SECONDS_PERIODIC_TIMER))
        {
            WICED_BT_TRACE("3D Magnetic sensor timer initialize failed\r\n");
        }
    }

    /* Initialize motion sensor timer */
    if(WICED_BT_SUCCESS != wiced_init_timer(&motion_timer_handle,
           &motion_timer_callback, 0u, WICED_MILLI_SECONDS_PERIODIC_TIMER))
    {
        WICED_BT_TRACE("Motion sensor timer initialize failed\r\n");
    }

    /* Initialize temperature read timer */
    if(WICED_BT_SUCCESS != wiced_init_timer(&temp_timer_handle,
           &temp_timer_callback, 0u, WICED_MILLI_SECONDS_PERIODIC_TIMER))
    {
        WICED_BT_TRACE("Temperature timer initialize failed\r\n");
    }

}

/*******************************************************************************
 * Function Name: motion_timer_start
 *******************************************************************************
 *
 * Summary  : This function starts the motion sensor timer
 *
 * Return   : None
 ******************************************************************************/
void motion_timer_start(void)
{
    if(WICED_BT_SUCCESS != wiced_start_timer(&motion_timer_handle,
                                            MOTION_SENSOR_TIME_MS))
    {
        WICED_BT_TRACE("Motion sensor timer start failed\r\n");
    }
}

/*******************************************************************************
 * Function Name: motion_timer_stop
 *******************************************************************************
 *
 * Summary  :  This function stops the motion sensor timer
 *
 * Return   : None
 ******************************************************************************/
void motion_timer_stop(void)
{
    if(WICED_BT_SUCCESS != wiced_stop_timer(&motion_timer_handle))
    {
        WICED_BT_TRACE("Motion sensor timer stop failed\r\n");
    }
}

/*******************************************************************************
 * Function Name: mag3d_timer_start
 *******************************************************************************
 *
 * Summary  : This function starts the 3D magnetic sensor timer 
 *
 * Return   : None
 ******************************************************************************/
void mag3d_timer_start(void)
{
    if(WICED_BT_SUCCESS != wiced_start_timer(&mag3d_timer_handle, MAG3D_TIME_MS))
    {
        WICED_BT_TRACE("3D Magnetic sensor timer start failed\r\n");
    }
}

/*******************************************************************************
 * Function Name: mag3d_timer_stop
 *******************************************************************************
 *
 * Summary   :This function stops the 3D magnetic sensor timer 
 *
 * Return    : None
 ******************************************************************************/
void mag3d_timer_stop(void)
{
    if(WICED_BT_SUCCESS != wiced_stop_timer(&mag3d_timer_handle))
    {
        WICED_BT_TRACE("3D Magnetic sensor timer stop failed\r\n");
    }
}

/*******************************************************************************
 * Function Name: temp_timer_start
 *******************************************************************************
 *
 * Summary   :This function starts the temperature timer
 *
 * Return    : None
 ******************************************************************************/
void temp_timer_start(void)
{
    if(WICED_BT_SUCCESS != wiced_start_timer(&temp_timer_handle, TEMPERATURE_TIME_MS))
    {
        WICED_BT_TRACE("Temperature timer start failed\r\n");
    }
}

/*******************************************************************************
 * Function Name: temp_timer_stop
 *******************************************************************************
 *
 * Summary   :This function stops the temperature timer 
 *
 * Return    : None
 ******************************************************************************/
void temp_timer_stop(void)
{
    if(WICED_BT_SUCCESS != wiced_stop_timer(&temp_timer_handle))
    {
        WICED_BT_TRACE("Temperature timer stop failed\r\n");
    }
}

/*******************************************************************************
 * Function Name: motion_sensor_update_gatt
 *******************************************************************************
 *
 * Summary  : This function gets motion sensors data 
 *            (accelerometer,gyroscope and magnetic)and update the value in Gatt 
 *
 * Return   : None
 ******************************************************************************/
void motion_sensor_update_gatt(void)
{
    get_accel_val(&((axis3bit16_t *)(app_sensor_hub_motion_sensor_notify))[0]);

    get_gyro_val(&((axis3bit16_t *)(app_sensor_hub_motion_sensor_notify))[1]);

    get_mag_val(&((axis3bit16_t *)(app_sensor_hub_motion_sensor_notify))[2]);
}


/*******************************************************************************
 * Function Name: mag3d_sensor_update_gatt
 *******************************************************************************
 *
 * Summary   : This function reads 3D magnetic sensors value and Update 
 *                                     the value in Gatt 
 *
 * Return     : None
 ******************************************************************************/
void mag3d_sensor_update_gatt(void)
{
    mag3d_sensor_read_val((mag3axis_t*)&app_sensor_hub_magnetic_3dsensor_notify[0]);
}


/*******************************************************************************
 * Function Name: temp_sensor_update_gatt
 *******************************************************************************
 *
 * Summary  :This function will update the temperature sensor value in gatt
 *
 * Return   : None
 ******************************************************************************/
void temp_sensor_update_gatt(void)
{
    int16_t temp_val = 0;

    temp_val = (uint16_t) get_temperature();

    app_sensor_hub_temp_sensor_notify[0] = ABS(temp_val/100) & 0xFF;
    app_sensor_hub_temp_sensor_notify[1] = ABS(temp_val%100) & 0xFF;
}

/*******************************************************************************
 * Function Name: motion_timer_callback
 *******************************************************************************
 *
 * Summary          : This function works on Motion timer callback
 * @param[in] arg   : not used
 *
 * Return           : None
 ******************************************************************************/
void motion_timer_callback(uint32_t arg)
{
     if((GATT_CLIENT_CONFIG_NOTIFICATION ==
             ble_server_hostinfo.sensor_value_cc_config[0])
                                   && (0 != ble_server_hostinfo.connection_id ))
        {
           WICED_BT_TRACE("Sending motion sensor notification\r\n");
           motion_sensor_update_gatt();

           if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(
                                             ble_server_hostinfo.connection_id ,
                                      HDLC_SENSOR_HUB_MOTION_SENSOR_NOTIFY_VALUE,
                                      app_gatt_db_ext_attr_tbl[2].cur_len,
                                      app_gatt_db_ext_attr_tbl[2].p_data))
           {
            WICED_BT_TRACE("Sending motion sensor notification failed\r\n");
           }
        }
}

/*******************************************************************************
 * Function Name: mag3d_timer_callback
 *******************************************************************************
 *
 * Summary             :This function works on 3D magnetic sensor timer callback
 * @param[in] arg      : not used
 *
 * Return              : None
 ******************************************************************************/
void mag3d_timer_callback(uint32_t arg)
{

    if((GATT_CLIENT_CONFIG_NOTIFICATION ==
            ble_server_hostinfo.sensor_value_cc_config[1])
                                   && (0 != ble_server_hostinfo.connection_id ))
    {

        WICED_BT_TRACE("Sending 3D magnetic sensor notification\r\n");

        mag3d_sensor_update_gatt();

        if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(
                          ble_server_hostinfo.connection_id ,
                          HDLC_SENSOR_HUB_MAGNETIC_3DSENSOR_NOTIFY_VALUE,
                          app_gatt_db_ext_attr_tbl[4].cur_len,
                          app_gatt_db_ext_attr_tbl[4].p_data))
        {
            WICED_BT_TRACE("Sending 3D magnetic sensor notification failed\r\n");
        }

    }
}

/*******************************************************************************
 * Function Name: temp_timer_callback
 *******************************************************************************
 *
 * Summary           :This function works on temperature sensor timer callback
 * @param[in] arg    : not used
 *
 * Return            : None
 ******************************************************************************/
void temp_timer_callback(uint32_t arg)
{

    if((GATT_CLIENT_CONFIG_NOTIFICATION ==
            ble_server_hostinfo.sensor_value_cc_config[2])
                                       && (0 != ble_server_hostinfo.connection_id ))
    {
       WICED_BT_TRACE("Sending temperature sensor notification\r\n");

       temp_sensor_update_gatt();

       if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(
                                                  ble_server_hostinfo.connection_id ,
                                  HDLC_SENSOR_HUB_TEMP_SENSOR_NOTIFY_VALUE,
                                  app_gatt_db_ext_attr_tbl[6].cur_len,
                                  app_gatt_db_ext_attr_tbl[6].p_data))
       {
         WICED_BT_TRACE("Sending temperature sensor notification failed\r\n");
       }
    }
}
