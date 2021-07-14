/*******************************************************************************
* File Name:   ble_server.c
*
* Description: This file contains the Bluetooth LE server implementation and handles
*                Bluetooth LE events.
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
#include "bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_l2c.h"
#include "wiced_rtos.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_aclk.h"
#include "ble_server.h"
#include "ble_cfg.h"
#include "sensors.h"
#include "wiced_result.h"
#include "wiced_hal_gpio.h"


/******************************************************************************
 *                   External Variables and Functions
 ******************************************************************************/

static wiced_bt_gatt_status_t ble_server_gatt_callback(wiced_bt_gatt_evt_t event,
                                        wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t ble_server_gatt_conn_status_cb(
                                    wiced_bt_gatt_connection_status_t *p_status);

static wiced_bt_gatt_status_t ble_server_gatt_connection_up(
                                    wiced_bt_gatt_connection_status_t *p_status);

static wiced_bt_gatt_status_t ble_server_gatt_connection_down(
                                    wiced_bt_gatt_connection_status_t *p_status);

static wiced_bt_gatt_status_t ble_server_gatt_req_cb( uint16_t conn_id,
                                            wiced_bt_gatt_request_type_t type,
                                            wiced_bt_gatt_request_data_t *p_data);

static wiced_bt_gatt_status_t ble_server_gatt_get_value( uint16_t attr_handle,
                                            uint16_t conn_id, uint8_t *p_val,
                                            uint16_t max_len, uint16_t *p_len );

static wiced_bt_gatt_status_t ble_server_gatt_set_value( uint16_t attr_handle,
                                                uint16_t conn_id, uint8_t *p_val,
                                                uint16_t len );

static wiced_bt_gatt_status_t ble_server_gatt_req_read_handler(uint16_t conn_id,
                                                wiced_bt_gatt_read_t * p_read_data);

static wiced_bt_gatt_status_t ble_server_gatt_req_write_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_write_t * p_data);

static void ble_app_init(void);

static void ble_app_adv_config(void);

static void app_set_status_led(uint8_t status, uint8_t frequency, uint8_t dutycycle);

static void app_button_interrupt_cb(void* user_data, uint8_t port_pin);
/******************************************************************************
 *                                Macros
 ******************************************************************************/

#define PWM_CHANNEL                             PWM0
#define PWM_INP_CLK_IN_HZ                       (32*1000) /* PWM Input Clock Frequency*/

#define PWM_FREQ_IN_HZ                          (2)
#define PWM_DUTY_CYCLE                          (50)

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/* Holds the host information*/
host_info_t ble_server_hostinfo;

wiced_pwm_config_t pwm_config;
/******************************************************************************
*                                Function Definitions
******************************************************************************/
/******************************************************************************
 * Function         ble_app_management_cback
 *
 *                  This function is invoked after the controller's
 *                  initialization is complete
 *
 * @param[in] event                : Callback Event number
 * @param[in] p_event_data         : Event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 ******************************************************************************/
wiced_result_t ble_app_management_cback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    uint16_t conn_interval = 0;

    WICED_BT_TRACE("Sensor hub management cback: %d\r\n", event);

    switch(event)
    {
        case BTM_ENABLED_EVT:
            /*initialize the drivers */
            sensors_init();
            /* Initialize the application */
            ble_app_init();
            break;

        case BTM_DISABLED_EVT:
            /* Bluetooth Controller and Host Stack Disabled */
            WICED_BT_TRACE("Bluetooth Disabled \r\n");
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* No IO capabilities on this platform */
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data     = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req     = BTM_LE_AUTH_REQ_NO_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = MAX_SECURITY_KEY_SIZE;
            p_event_data->pairing_io_capabilities_ble_request.init_keys    = 0;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys    = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
            break;

         case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

             if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF)
             {
                 WICED_BT_TRACE("Advertisement State Change: OFF\r\n");
             }
             else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_HIGH)
             {
                 WICED_BT_TRACE("Advertisement State Change: Undirected High\r\n");
             }
             else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_LOW)
             {
                 WICED_BT_TRACE("Advertisement State Change: Undirected Low\r\n");
             }
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            /* Connection parameters updated */
            if(WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                conn_interval = (p_event_data->ble_connection_param_update.conn_interval) * CONN_INTERVAL_MULTIPLIER;
                WICED_BT_TRACE("New connection parameters:"
                               "\n\rConnection interval: %d.%dms\r\n",
                                  CONN_INTERVAL_MAJOR(conn_interval),
                                 CONN_INTERVAL_MINOR(conn_interval));
            }
            else
            {
                WICED_BT_TRACE("Connection parameters update failed\r\n");
            }
            break;

        default:
            WICED_BT_TRACE("Unhandled Bluetooth Management Event: %d\r\n", event);
            break;
    }

    return result;
}

/******************************************************************************
 * Function         ble_app_init(void)
 *
 * @brief           This function Inlitilize the Bluetooth LE app  
 *
 * @return    None
 ******************************************************************************/
void ble_app_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_device_address_t  local_device_bd_addr;

    /* Configure the Button GPIO as an input with a resistive pull up and falling edge interrupt */
    wiced_hal_gpio_register_pin_for_interrupt( SW3, app_button_interrupt_cb, NULL );
    wiced_hal_gpio_configure_pin( USER_BUTTON,
                                ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ),
                                GPIO_PIN_OUTPUT_HIGH );

    /* Enable PWM Clock*/
    wiced_hal_aclk_enable(PWM_INP_CLK_IN_HZ, WICED_ACLK1, WICED_ACLK_FREQ_24_MHZ);

    app_set_status_led(1u, PWM_FREQ_IN_HZ, PWM_DUTY_CYCLE);

    wiced_bt_dev_read_local_addr(local_device_bd_addr);

    WICED_BT_TRACE("Bluetooth Device Address: [ %B] \r\n\n",
                    local_device_bd_addr);

    /* Not Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0u);

    WICED_BT_TRACE("\r\nDiscover this device with the name: \"%s\"\r\n",
                    app_gap_device_name);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(ble_server_gatt_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\r\n", gatt_status);

    /*  Tell stack to use our GATT database */
    WICED_BT_TRACE("GATT DB length: %d\r\n", gatt_database_len);
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\r\n", gatt_status);

    /* Set the advertising params and make the device discoverable */
    ble_app_adv_config();

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         0u, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\r\n", result);

}

/*******************************************************************************
 * Function         ble_server_gatt_callback
 *
 * @brief           This function is invoked on GATT event
 *
 * @param[in] event                : GATT event
 * @param[in] p_event_data         : GATT event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 *******************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_callback(wiced_bt_gatt_evt_t event,
                                             wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_PDU;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = ble_server_gatt_conn_status_cb(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = ble_server_gatt_req_cb(p_attr_req->conn_id, p_attr_req->request_type, &p_attr_req->data);
        break;

    default:
        break;
    }
    return status;
}

/*******************************************************************************
 * Function         ble_server_gatt_conn_status_cb
 *
 * @brief           This function is invoked on GATT connection event
 *
 * @param[in] p_status               : GATT connection status
 *
 * @return                           : Connection status from connection up/down function
 *******************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_conn_status_cb(
                                    wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        return ble_server_gatt_connection_up(p_status);
    }

    return ble_server_gatt_connection_down(p_status);
}

/*******************************************************************************
 * Function         ble_server_gatt_connection_up
 *
 * @brief           This function is invoked when connection is established
 *
 * @param[in] p_status               : GATT connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 *******************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_connection_up(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result = WICED_BT_ERROR;

    WICED_BT_TRACE("Sensor hub connected with [ %B] id:%d\r\n", p_status->bd_addr,
                                                             p_status->conn_id);
    app_set_status_led(0u, PWM_FREQ_IN_HZ, PWM_DUTY_CYCLE);

    /* Update the connection handler.  Save address of the connected device. */
    ble_server_hostinfo.connection_id= p_status->conn_id;

    /* Stop advertising */
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC,
                                                                          NULL);

    WICED_BT_TRACE("Stopping Advertisements %d\r\n", result);

    /* Update connection parameters to 100 ms for SDS */
    wiced_bt_l2cap_update_ble_conn_params(p_status->bd_addr, CONN_INTERVAL,
                                      CONN_INTERVAL, CONN_LATENCY, SUP_TIMEOUT);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
 * Function         ble_server_gatt_connection_down
 *
 * @brief           This function is invoked when connection is lost
 *
 * @param[in] p_status               : GATT connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 ******************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_connection_down(
                                    wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    BD_ADDR bdaddr;
    memcpy(bdaddr, ble_server_hostinfo.link_keys.bd_addr, sizeof(bdaddr));

    WICED_BT_TRACE("Sensor hub disconnected from [ %B] conn_id:%d reason:%d\r\n",
            bdaddr, p_status->conn_id, p_status->reason);

    app_set_status_led(1u, PWM_FREQ_IN_HZ, PWM_DUTY_CYCLE);

    /* Resetting the device info */
    ble_server_hostinfo.connection_id = 0;

    /*Undirected advertisement */

    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                                     0u, NULL);

    WICED_BT_TRACE("Advertisement started, status:%d\r\n", result);


    return WICED_BT_SUCCESS;
}

/*******************************************************************************
 * Function         ble_server_gatt_req_cb
 *
 * @brief           Process GATT request from the peer
 *
 * @param[in] p_data                 : GATT attribute request data
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 ********************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_req_cb( uint16_t conn_id, wiced_bt_gatt_request_type_t type,
                                                                wiced_bt_gatt_request_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_ERROR;

    /* Check the type of request and service it by function calls */
    switch (type)
    {
    case GATTS_REQ_TYPE_READ:
        WICED_BT_TRACE("GATT read request. connection handle is %d\r\n", p_data->handle);
        result = ble_server_gatt_req_read_handler(conn_id, &p_data->read_req);
        break;
    case GATTS_REQ_TYPE_WRITE:
        WICED_BT_TRACE("GATT write request. connection handle is %d\r\n", p_data->handle);
        result = ble_server_gatt_req_write_handler(conn_id, &p_data->write_req);
        break;
    default:
        WICED_BT_TRACE("GATT request unhandled..\r\n");
        break;
    }
    return result;
}

/*******************************************************************************
 * Function         ble_server_gatt_get_value
 *
 * @brief           This function handles reading of the attribute value from the GATT database and passing the
 *                   data to the BT stack.
 *
 * @param[in] attr_handle               : Attribute handle for read operation
 * @param[in] conn_id                     : Connection ID
 * @param[in] *p_val                     : Pointer to the buffer to store read data
 * @param[in] max_len                     : Maximum buffer length available to store
 * @param[in] *p_len                     : Actual length of data copied to the buffer
 *
 * @return                               : wiced_bt_gatt_status_e
 *******************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_get_value( uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val,
                                                                         uint16_t max_len, uint16_t *p_len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Detected a matching handle in the external lookup table
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= max_len)
            {
                // Value fits within the supplied buffer; copy over the value
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data, app_gatt_db_ext_attr_tbl[i].cur_len);
                status = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                // Value to read will not fit within the buffer
                status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // Add code to read value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully read then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
        default:
            // The read operation was not performed for the indicated handle
            WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\r\n", attr_handle);
            status = WICED_BT_GATT_READ_NOT_PERMIT;
            break;
        }
    }

    return status;
}

/*******************************************************************************
 * Function         ble_server_gatt_set_value
 *
 * @brief           This function handles writing to the attribute value in the GATT database
 *                   using the data passed from the BT stack.
 *
 * @param[in] attr_handle               : Attribute handle for write operation
 * @param[in] conn_id                     : Connection ID
 * @param[in] *p_val                     : Pointer to the buffer to store the data to be written
 * @param[in] len                         : Length of data to be written
 *
 * @return                               : wiced_bt_gatt_status_e
 ********************************************************************************/

wiced_bt_gatt_status_t ble_server_gatt_set_value( uint16_t attr_handle, uint16_t conn_id,
                                                            uint8_t *p_val, uint16_t len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Verify that size constraints have been met
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                // Value fits within the supplied buffer; copy over the value
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                status = WICED_BT_GATT_SUCCESS;

                switch ( attr_handle )
                {
                case HDLD_SENSOR_HUB_MOTION_SENSOR_NOTIFY_CLIENT_CHAR_CONFIG:
                    if ( len != 2 )
                    {
                        return WICED_BT_GATT_INVALID_ATTR_LEN;
                    }
                    ble_server_hostinfo.sensor_value_cc_config[0] = p_val[0] | ( p_val[1] << 8 );

                    /* If notifications enabled then start the notification timer.
                     * Send notifications at 100 ms interval.
                     */
                    if(GATT_CLIENT_CONFIG_NOTIFICATION ==
                               app_sensor_hub_motion_sensor_notify_client_char_config[0])
                    {
                        motion_timer_start();
                    }
                    else
                    {
                        motion_timer_stop();
                    }

                    break;
                case HDLD_SENSOR_HUB_MAGNETIC_3DSENSOR_NOTIFY_CLIENT_CHAR_CONFIG:

                    if ( len != 2 )
                    {
                        return WICED_BT_GATT_INVALID_ATTR_LEN;
                    }
                    ble_server_hostinfo.sensor_value_cc_config[1] = p_val[0] | ( p_val[1] << 8 );

                     /* If notifications enabled then start notification and idle timer.
                      * Send notifications at 500 ms interval and check for interrupt from
                      * sensor after 10s. If interrupt not active then got to SDS
                      */

                     if(mag3d_sensor_isConnected())
                     {

                         if(GATT_CLIENT_CONFIG_NOTIFICATION ==
                                    app_sensor_hub_magnetic_3dsensor_notify_client_char_config[0])
                         {
                             mag3d_timer_start();
                         }
                         else
                         {
                             mag3d_timer_stop();
                         }
                     }

                     break;
                 case HDLD_SENSOR_HUB_TEMP_SENSOR_NOTIFY_CLIENT_CHAR_CONFIG:

                     if ( len != 2 )
                     {
                         return WICED_BT_GATT_INVALID_ATTR_LEN;
                     }
                     ble_server_hostinfo.sensor_value_cc_config[2] = p_val[0] | ( p_val[1] << 8 );

                     /* If notifications enabled then start notification and idle timer.
                     * Send notifications at 1000 ms interval
                     */
                     if(GATT_CLIENT_CONFIG_NOTIFICATION ==
                             app_sensor_hub_temp_sensor_notify_client_char_config[0])
                     {
                         temp_timer_start();
                     }
                     else
                     {
                         temp_timer_stop();
                     }

                     break;

                default:
                    WICED_BT_TRACE("Write is not supported \r\n");
                }
            }
            else
            {
                // Value to write does not meet size constraints
                status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        switch ( attr_handle )
        {
        default:
            // The write operation was not performed for the indicated handle
            WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\r\n", attr_handle);
            status = WICED_BT_GATT_WRITE_NOT_PERMIT;
            break;
        }
    }

    return status;
}

/*******************************************************************************
 * Function         ble_server_gatt_req_read_handler
 *
 * @brief           Process Read request or command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from GATT read request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 *******************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_req_read_handler(uint16_t conn_id,
                                             wiced_bt_gatt_read_t * p_read_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = ble_server_gatt_get_value(p_read_data->handle, conn_id, p_read_data->p_val,
                                        *p_read_data->p_val_len, p_read_data->p_val_len);

    if(WICED_BT_GATT_SUCCESS == status)
    {
        /* If Sensor value requested then update the GATT DB with latest value */
        if(HDLC_SENSOR_HUB_MOTION_SENSOR_NOTIFY_VALUE == p_read_data->handle)
        {
            motion_sensor_update_gatt();
        }
        if((HDLC_SENSOR_HUB_MAGNETIC_3DSENSOR_NOTIFY_VALUE == p_read_data->handle)
                                                        && mag3d_sensor_isConnected())
        {
            mag3d_sensor_update_gatt();
        }
        if(HDLC_SENSOR_HUB_TEMP_SENSOR_NOTIFY_VALUE == p_read_data->handle)
        {
            temp_sensor_update_gatt();
        }
    }
    return status;
}

/*******************************************************************************
 * Function         ble_server_gatt_req_write_handler
 *
 * @brief           Process write request or write command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from GATT write request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 *******************************************************************************/
wiced_bt_gatt_status_t ble_server_gatt_req_write_handler(uint16_t conn_id,
                                                 wiced_bt_gatt_write_t * p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    status = ble_server_gatt_set_value(p_data->handle, conn_id,
                                        p_data->p_val, p_data->val_len);

    return status;
}

/*******************************************************************************
 * Function         ble_app_adv_config
 *
 * @brief           Configure advertisement data
 *
 * @return                          : None
 ******************************************************************************/
void ble_app_adv_config(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3] = {0};
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t ble_server_service_uuid[LEN_UUID_128] = { __UUID_SERVICE_SENSOR_HUB };

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)app_gap_device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)app_gap_device_name;
    num_elem++;

    /* Advertisement Element for Sensor Hub Service */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = ble_server_service_uuid;
    num_elem++;

    if(WICED_BT_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem))
    {
        WICED_BT_TRACE("Set advertisement data failed\r\n");
    }
}




/*******************************************************************************
 * Function        app_set_status_led
 *
 *                 Helper function to Control LED with PWM for advertisement status.
 * @param[in] status              : LED status value to set (ON/OFF)
 * @param[in] frequency           : PWM frequency value
 * @param[in] dutycycle           : PWM Dutycycle value
 * @return                        : None
 *******************************************************************************/
void app_set_status_led(uint8_t status, uint8_t frequency, uint8_t dutycycle)
{

    if (status)
    {
        wiced_hal_gpio_select_function(LED1, WICED_PWM0);

        wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ,
                                 dutycycle,
                                 frequency,
                                 &pwm_config);

        wiced_hal_pwm_start(PWM_CHANNEL,
                            PMU_CLK,
                            pwm_config.toggle_count,
                            pwm_config.init_count,
                            0);

        wiced_hal_pwm_enable(PWM_CHANNEL);
    }
    else
    {
        wiced_hal_pwm_disable(PWM_CHANNEL);
        wiced_hal_gpio_select_function(LED1, WICED_GPIO);
        wiced_hal_gpio_set_pin_output(LED1, 0u);
    }
}


/*******************************************************************************
 * Function         button_interrupt_cb
 *
 * @brief           Button callback
 * @param[in] user_data      : related to the interrupt, populated internally.
 * @param[in] port_pin       : pin number on which interrupt was received.
 *
 * @return                   : None
 *******************************************************************************/
void app_button_interrupt_cb(void* user_data, uint8_t port_pin)
{
    wiced_result_t result = WICED_BT_ERROR;
    uint16_t written_byte = 0u;

    if(0 == ble_server_hostinfo.connection_id)
    {
        app_set_status_led(1u, PWM_FREQ_IN_HZ, PWM_DUTY_CYCLE);

        /* Config the advertising params and make the device discoverable */
        ble_app_adv_config();

        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         0u, NULL);
        WICED_BT_TRACE("Advertisement started, status:%d\r\n", result);
    }

    /* Clear the GPIO interrupt */
    wiced_hal_gpio_clear_pin_interrupt_status( port_pin );

}

