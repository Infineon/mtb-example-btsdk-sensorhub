/*
 * $ Copyright Cypress Semiconductor $
 */

 /** @file
 *
 * This file implements all the BLE related functionality, button callbacks and
 * PUART Rx
 *
 */

#include "wiced_bt_gatt.h"
#include "cycfg_gatt_db.h"
#include "cycfg_pins.h"
#include "motion_sensor_hw.h"
#include "motion_sensor_ble.h"
#include "app_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_i2c.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "bt_types.h"
#include "wiced_bt_l2c.h"


/******************************************************************************
 *                   External Variables and Functions
 ******************************************************************************/

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
/* Notification timer and Idle timer handles */
wiced_timer_t notification_timer_handle;
wiced_timer_t idle_timer_handle;

/* Variables for passcode input */
uint8_t user_input_state;
uint32_t passkey = 0;
uint8_t passkey_itr = 0;

/* Holds the host info saved in the NVRAM */
host_info_t motion_sensor_hostinfo;

/* Holds the connection ID */
uint8_t conn_id = 0;
/******************************************************************************
*                                Function Definitions
******************************************************************************/
/**
 * Function         motion_sensor_management_cback
 *
 *                  This function is invoked after the controller's
 *                  initialization is complete
 *
 * @param[in] event                : Callback Event number
 * @param[in] p_event_data         : Event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 */
wiced_result_t motion_sensor_management_cback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    uint8_t                             bytes_written = 0;
    wiced_result_t                      rc = WICED_ERROR;

    WICED_BT_TRACE("Motion Sensor management cback: %d\r\n", event);

    switch(event)
    {
        case BTM_ENABLED_EVT:
            /* Initialize the application */
            motion_sensor_application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("numeric_value: %d \r\n",
                    p_event_data->user_confirmation_request.numeric_value);

            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                    p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA [ %B], Key %d \r\n",
                    p_event_data->user_passkey_notification.bd_addr,
                    p_event_data->user_passkey_notification.passkey);

            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                    p_event_data->user_passkey_notification.bd_addr);

            break;

        case BTM_PASSKEY_REQUEST_EVT:
            WICED_BT_TRACE("PassKey requested\r\n",
                    p_event_data->user_passkey_notification.bd_addr,
                    p_event_data->user_passkey_notification.passkey);

            user_input_state = GET_PASSKEY;

            WICED_BT_TRACE ("Enter Six Digit Passkey\r\n");

            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Grant security */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                        WICED_BT_SUCCESS);

            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
                             BTM_IO_CAPABILITIES_BLE_DISPLAY_AND_KEYBOARD_INPUT;

            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;

            p_event_data->pairing_io_capabilities_ble_request.auth_req =
                                                   BTM_LE_AUTH_REQ_SC_MITM_BOND;

            p_event_data->pairing_io_capabilities_ble_request.max_key_size =
                                                          MAX_SECURITY_KEY_SIZE;

            p_event_data->pairing_io_capabilities_ble_request.init_keys =
                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;

            p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;

            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;

            WICED_BT_TRACE("Pairing Complete: %d\r\n", p_info->reason);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            motion_sensor_hostinfo.link_keys = p_event_data->paired_device_link_keys_update;

            WICED_BT_TRACE("Link keys update Address: [ %B], Address type: %d\n", motion_sensor_hostinfo.link_keys.bd_addr,
                                                       motion_sensor_hostinfo.link_keys.key_data.static_addr_type);

            bytes_written = wiced_hal_write_nvram(MOTION_SENSOR_VS_ID,
            sizeof(motion_sensor_hostinfo), (uint8_t*)&motion_sensor_hostinfo, &rc);

            if(0 == bytes_written)
            {
                WICED_BT_TRACE("0 Bytes written to NVRAM\r\n");
            }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            wiced_hal_read_nvram(MOTION_SENSOR_VS_ID, sizeof(motion_sensor_hostinfo),
                                   (uint8_t*)&motion_sensor_hostinfo, &result);

            p_event_data->paired_device_link_keys_request = motion_sensor_hostinfo.link_keys;

            WICED_BT_TRACE("keys read from NVRAM [ %B] result: %d \n",
                           (uint8_t*)&motion_sensor_hostinfo.link_keys, result);

            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            motion_sensor_hostinfo.local_keys = p_event_data->local_identity_keys_update;

            bytes_written = wiced_hal_write_nvram(MOTION_SENSOR_VS_ID,
              sizeof(motion_sensor_hostinfo), (uint8_t*)&motion_sensor_hostinfo, &rc);

            if(0 == bytes_written)
            {
                WICED_BT_TRACE("0 Bytes written to NVRAM\r\n");
            }
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            wiced_hal_read_nvram(MOTION_SENSOR_VS_ID, sizeof(motion_sensor_hostinfo),
                                   (uint8_t*)&motion_sensor_hostinfo, &result);

            /* read keys from NVRAM */
            if (1 == motion_sensor_hostinfo.dev_prebonded)
            {
                p_event_data->local_identity_keys_request =
                                              motion_sensor_hostinfo.local_keys;

                WICED_BT_TRACE("local keys read from NVRAM result: %d \r\n", result);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Device not bonded\r\n");
            }

            break;

        case BTM_ENCRYPTION_STATUS_EVT:

            WICED_BT_TRACE("Encryption Status Event: bd [ %B] res %d\r\n",
                    p_event_data->encryption_status.bd_addr,
                    p_event_data->encryption_status.result);

            /* Check if the encrypted device was prebonded and then restore the CCCD value */
            if(WICED_SUCCESS == p_event_data->encryption_status.result && 0 != motion_sensor_hostinfo.dev_prebonded)
            {
                /* Restore CCCD value for bonded devices */
                app_motion_sensor_motion_sensor_notify_client_char_config[0] = 
                    (motion_sensor_hostinfo.sensor_value_characteristic_client_configuration & 0x00FF);

                app_motion_sensor_motion_sensor_notify_client_char_config[1] = 
                    ((motion_sensor_hostinfo.sensor_value_characteristic_client_configuration >> 8) & 0x00FF);

                if(GATT_CLIENT_CONFIG_NOTIFICATION ==
                   app_motion_sensor_motion_sensor_notify_client_char_config[0])
                {
                    motion_sensor_acc_interrupt_enable();
                }
            }

            /* Process SMP bond result */
            motion_sensor_smp_bond_result(p_info->status);

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            WICED_BT_TRACE("Advertisement State Change: %d\r\n", p_event_data->ble_advert_state_changed);

            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE("Connection parameters status:%d\r\n"
                    "Connection Interval: %d intervals\r\n"
                    "Connection Latency: %d intervals\r\nConnection"
                    "Timeout: %d ms\r\n",
                    p_event_data->ble_connection_param_update.status,
                    p_event_data->ble_connection_param_update.conn_interval,
                    p_event_data->ble_connection_param_update.conn_latency,
                    ((p_event_data->ble_connection_param_update.supervision_timeout) * 10));

            break;

        default:
            WICED_BT_TRACE("Unhandled Bluetooth Management Event: %d\r\n", event);
            break;
    }

    return result;
}

/**
 * Function         motion_sensor_application_init
 *
 * @brief           This function is invoked after ENABLED event from controller
 *
 * @return    None
 */
void motion_sensor_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result, timer_result;
    wiced_bt_device_address_t  local_device_bd_addr;

    WICED_BT_TRACE("\r\nDiscover this device with the name: \"%s\"\r\n",
                    wiced_bt_cfg_settings.device_name);

    wiced_bt_dev_read_local_addr(local_device_bd_addr);

    WICED_BT_TRACE("Bluetooth Device Address: [ %B] \r\n\n",
                    local_device_bd_addr);

    /* Initialize PUART for input */
    motion_sensor_set_console_input();

    /* Initialize I2C */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    /* Register GPIO for interrupt and button */
    wiced_hal_gpio_register_pin_for_interrupt(MOTION_SENSOR_INTERRUPT,
                                         motion_sensor_interrupt_handler, NULL);

    wiced_platform_register_button_callback(SW3, button_cb,
                                      NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);

    /* Initialize Idle timer and Notification timer */
    if(WICED_BT_SUCCESS != wiced_init_timer(&notification_timer_handle,
           &notification_timer_callback, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER))
    {
        WICED_BT_TRACE("Notification timer failed\r\n");
    }

    if(WICED_BT_SUCCESS != wiced_init_timer(&idle_timer_handle, &idle_timer_callback,
                                                        0, WICED_SECONDS_TIMER))
    {
        WICED_BT_TRACE("Idle timer failed\r\n");
    }

    /* Read data from NVRAM in case data is available */
    if(0 == wiced_hal_read_nvram(MOTION_SENSOR_VS_ID, sizeof(motion_sensor_hostinfo),
                                    (uint8_t*)&motion_sensor_hostinfo, &result))
    {
        WICED_BT_TRACE("NVRAM read failed\r\n");
    }

    WICED_BT_TRACE("Host info read from NVRAM result: %d \r\n",  result);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(motion_sensor_gatts_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\r\n", gatt_status);

    /*  Tell stack to use our GATT database */
    WICED_BT_TRACE("GATT DB length: %d\r\n", gatt_database_len);
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\r\n", gatt_status);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Initialize GPIO and LSM9DS1 sensor */

        ENABLE_GPIO_INTERRUPT;

        initialize_all_sensors();

  /* If Device not currently connected and no bonded device do undirected advertisement */
   if(0 == motion_sensor_hostinfo.dev_prebonded)
  {
     /* Set the advertising params and make the device discoverable */
       motion_sensor_set_advertisement_data();
       result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         NULL, NULL);
       WICED_BT_TRACE("wiced_bt_start_advertisements %d\r\n", result);
  }
 /* If Device not currently connected but bonded device present do directed advertisement */
   else
    {
        /* Set the advertising params and make the device discoverable */
        WICED_BT_TRACE("Address: [ %B], Address type: %d\r\n", motion_sensor_hostinfo.link_keys.bd_addr,
                                           motion_sensor_hostinfo.link_keys.key_data.static_addr_type);

        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_LOW,
                motion_sensor_hostinfo.link_keys.key_data.static_addr_type,  motion_sensor_hostinfo.link_keys.bd_addr);

        WICED_BT_TRACE("wiced_bt_start_advertisements Directed %d\r\n", result);
    }
}

/**
 * Function         motion_sensor_gatts_callback
 *
 * @brief           This function is invoked on GATT event
 *
 * @param[in] event                : Gatt event
 * @param[in] p_data               : Gatt event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 */
wiced_bt_gatt_status_t motion_sensor_gatts_callback(wiced_bt_gatt_evt_t event,
                                             wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("GATT EVT: %d\r\n", event);
    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = motion_sensor_gatts_conn_status_cb(&p_data->connection_status);
        if(WICED_BT_GATT_SUCCESS != result)
        {
            WICED_BT_TRACE("GATT Connection CB failed: %d", result);
        }
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = motion_sensor_gatts_req_cb(&p_data->attribute_request);
        if(WICED_BT_GATT_SUCCESS != result)
        {
            WICED_BT_TRACE("GATT Req CB failed: %d", result);
        }
        break;

    default:
        break;
    }
    return result;
}

/**
 * Function         motion_sensor_gatts_conn_status_cb
 *
 * @brief           This function is invoked on GATT connection event
 *
 * @param[in] p_status               : Gatt connection status
 *
 * @return                           : Connection status from connection up/down function
 */
wiced_bt_gatt_status_t motion_sensor_gatts_conn_status_cb(
                                    wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        return motion_sensor_gatts_connection_up(p_status);
    }

    return motion_sensor_gatts_connection_down(p_status);
}

/**
 * Function         motion_sensor_gatts_connection_up
 *
 * @brief           This function is invoked when connection is established
 *
 * @param[in] p_status               : Gatt connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t motion_sensor_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    wiced_result_t rc;

    WICED_BT_TRACE("Motion Sensor connected with [ %B] id:%d\r\n", p_status->bd_addr,
                                                             p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    conn_id = p_status->conn_id;

    /* Stop advertising */
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC,
                                                                          NULL);

    WICED_BT_TRACE("Stopping Advertisements %d\r\n", result);

    /* Update connection parameters to 100 ms for SDS */
    wiced_bt_l2cap_update_ble_conn_params(p_status->bd_addr, CONN_INTERVAL,
                                      CONN_INTERVAL, CONN_LATENCY, SUP_TIMEOUT);

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function         motion_sensor_gatts_connection_down
 *
 * @brief           This function is invoked when connection is lost
 *
 * @param[in] p_status               : Gatt connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t motion_sensor_gatts_connection_down(
                                    wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    BD_ADDR bdaddr;
    memcpy(bdaddr, motion_sensor_hostinfo.link_keys.bd_addr, sizeof(bdaddr));

    WICED_BT_TRACE("Motion Sensor disconnected from [ %B] conn_id:%d reason:%d\r\n",
            bdaddr, p_status->conn_id, p_status->reason);

    /* Resetting the device info */
    conn_id = 0;

    /* If device bonded then do directed advertisement */
    if(1 == motion_sensor_hostinfo.dev_prebonded)
    {
        /* Set the advertising params and make the device discoverable */
        WICED_BT_TRACE("Address: [ %B], Address type: %d\r\n",
           motion_sensor_hostinfo.link_keys.bd_addr, motion_sensor_hostinfo.link_keys.key_data.static_addr_type);

        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH,
           motion_sensor_hostinfo.link_keys.key_data.static_addr_type, bdaddr);

        WICED_BT_TRACE("wiced_bt_start_advertisements Directed %d\r\n", result);
    }
    /* If device not bonded then do undirected advertisement */
    else
    {
        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                                         NULL, NULL);

        WICED_BT_TRACE("wiced_bt_start_advertisements %d\r\n", result);
    }

    /* Disable the interrupt */
    motion_sensor_acc_interrupt_disable();

    return WICED_BT_SUCCESS;
}

/**
 * Function         motion_sensor_gatts_req_cb
 *
 * @brief           Process GATT request from the peer
 *
 * @param[in] p_data                 : Gatt attribute request data
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t motion_sensor_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("wiced_gatts_req_cb. connection %d, type %d\r\n",
                                         p_data->conn_id, p_data->request_type);

    /* Check the type of request and service it by function calls */
    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = motion_sensor_gatts_req_read_handler(p_data->conn_id,
                                                      &(p_data->data.read_req));
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = motion_sensor_gatts_req_write_handler(p_data->conn_id,
                                                     &(p_data->data.write_req));
        break;

    case GATTS_REQ_TYPE_MTU:
        result = motion_sensor_gatts_req_mtu_handler(p_data->conn_id,
                                                              p_data->data.mtu);
        break;

   default:
        break;
    }
    return result;
}

/**
 * Function         motion_sensor_get_attribute
 *
 * @brief           Find the right data from Gatt DB
 *
 * @param[in] handle                 : Gatt handle
 *
 * @return                           : Correct attribute pointer from Gatt DB
 */
gatt_db_lookup_table_t * motion_sensor_get_attribute(uint16_t handle)
            {
    int array_index;
    for (array_index = 0; array_index < app_gatt_db_ext_attr_tbl_size; array_index++)
    {
        if (app_gatt_db_ext_attr_tbl[array_index].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[array_index]);
        }
    }
    WICED_BT_TRACE("attr not found:%x\r\n", handle);
    return NULL;
}


/**
 * Function         motion_sensor_gatts_req_read_handler
 *
 * @brief           Process Read request or command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from Gatt read request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t motion_sensor_gatts_req_read_handler(uint16_t conn_id,
                                             wiced_bt_gatt_read_t * p_read_data)
{
    gatt_db_lookup_table_t *puAttribute;
    int          attr_len_to_copy;

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = motion_sensor_get_attribute(p_read_data->handle)))
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\r\n", p_read_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\r\n",
           conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy);

    /* If Sensor value requested then update the Gatt DB with latest value */
    if(HDLC_MOTION_SENSOR_MOTION_SENSOR_NOTIFY_VALUE == p_read_data->handle)
    {
        read_all_sensors_update_gatt();
    }

    if (p_read_data->offset >= puAttribute->cur_len)
    {
        attr_len_to_copy = 0;
    }

    if (attr_len_to_copy != 0)
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if (to_copy > *p_read_data->p_val_len)
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_data) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy(p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function         motion_sensor_gatts_req_write_handler
 *
 * @brief           Process write request or write command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from Gatt write request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t motion_sensor_gatts_req_write_handler(uint16_t conn_id,
                                                 wiced_bt_gatt_write_t * p_data)
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    uint8_t bytes_written = 0;
    wiced_result_t rc;

    WICED_BT_TRACE("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\r\n",
        conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len);

    switch (p_data->handle)
    {
    /* By writing into Characteristic Client Configuration descriptor
     * peer can enable or disable notification or indication */
    case HDLD_MOTION_SENSOR_MOTION_SENSOR_NOTIFY_CLIENT_CHAR_CONFIG:
        if (p_data->val_len != 2)
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        app_motion_sensor_motion_sensor_notify_client_char_config[0] = p_attr[0];
        app_motion_sensor_motion_sensor_notify_client_char_config[1] = p_attr[1];

        motion_sensor_hostinfo.sensor_value_characteristic_client_configuration =
            app_motion_sensor_motion_sensor_notify_client_char_config[0] |
            app_motion_sensor_motion_sensor_notify_client_char_config[1] << 1;

        /* If notifications enabled then start notification and idle timer.
         * Send notifications at 500 ms interval and check for interrupt from
         * sensor after 10s. If interrupt not active then got to SDS
         */
        if(GATT_CLIENT_CONFIG_NOTIFICATION ==
                   app_motion_sensor_motion_sensor_notify_client_char_config[0])
        {
            motion_sensor_acc_interrupt_enable();
            if(WICED_BT_SUCCESS != wiced_start_timer(&notification_timer_handle,
                                                           NOTFICATION_TIME_MS))
            {
                WICED_BT_TRACE("Notification timer start failed\r\n");
            }
            if(WICED_BT_SUCCESS != wiced_start_timer(&idle_timer_handle, IDLE_TIME_S))
            {
                WICED_BT_TRACE("Idle timer start failed\r\n");
            }
        }
        else
        {
            motion_sensor_acc_interrupt_disable();
        }

        bytes_written = wiced_hal_write_nvram(MOTION_SENSOR_VS_ID,
        sizeof(motion_sensor_hostinfo), (uint8_t*)&motion_sensor_hostinfo, &rc);
        if(0 == bytes_written)
        {
            WICED_BT_TRACE("0 Bytes written to NVRAM\r\n");
        }
        break;

        default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    return  result;
}

void motion_sensor_passkey_reply (uint32_t passkey)
{
   wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS,
                                   motion_sensor_hostinfo.link_keys.bd_addr ,passkey);
}

/**
 * Function         motion_sensor_smp_bond_result
 *
 * @brief           Process SMP bonding result. If we successfully paired with the
 *                  central device, save its BDADDR in the NVRAM and initialize
 *                  associated data
 *
 * @param[in] result                : Bonding result
 *
 * @return                          : None
 */
void motion_sensor_smp_bond_result(uint8_t result)
{
    wiced_result_t status;
    uint8_t bytes_written = 0;
    wiced_result_t rc;

    /* Bonding success */
    if (WICED_BT_SUCCESS == result)
    {
        /* Pack the data to be stored into the hostinfo structure */
        motion_sensor_hostinfo.dev_prebonded = 1;

        /* Write to NVRAM */
        bytes_written = wiced_hal_write_nvram(MOTION_SENSOR_VS_ID,
        sizeof(motion_sensor_hostinfo), (uint8_t*)&motion_sensor_hostinfo, &rc);

        if(0 == bytes_written)
        {
            WICED_BT_TRACE("0 Bytes written to NVRAM\r\n");
        }
    }
}

/**
 * Function         motion_sensor_gatts_req_mtu_handler
 *
 * @brief           Process MTU request from the peer
 *
 * @return                          : None
 */
wiced_bt_gatt_status_t motion_sensor_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\r\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function         motion_sensor_set_advertisement_data
 *
 * @brief           Set Advertisement data
 *
 * @return                          : None
 */
void motion_sensor_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t motion_sensor_service_uuid[LEN_UUID_128] = { __UUID_SERVICE_MOTION_SENSOR };

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = motion_sensor_service_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)wiced_bt_cfg_settings.device_name;
    num_elem++;

    if(WICED_BT_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem))
    {
        WICED_BT_TRACE("Set advertisement data failed\r\n");
    }
}

/**
 * Function         send_sensor_value_notification
 *
 * @brief           Send sensor value notification if CCCD == 1 and device connected
 *
 * @return                          : None
 */

void send_sensor_value_notification()
{
    if((GATT_CLIENT_CONFIG_NOTIFICATION ==
        motion_sensor_hostinfo.sensor_value_characteristic_client_configuration)
                                       && (0 != conn_id))
    {
       WICED_BT_TRACE("Sending Notification\r\n");
       read_all_sensors_update_gatt();

       if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(
                                  conn_id,
                                  HDLC_MOTION_SENSOR_MOTION_SENSOR_NOTIFY_VALUE,
                                  app_gatt_db_ext_attr_tbl[2].cur_len,
                                  app_gatt_db_ext_attr_tbl[2].p_data))
       {
           WICED_BT_TRACE("Sending sensor value notification failed\r\n");
       }
    }
}

/**
 * Function         motion_sensor_interrupt_handler
 *
 * @brief           Interrupt handler
 * @param[in] user_data                : User data (not used)
 * @param[in] value                    : value (not used)
 *
 * @return                          : None
 */
void motion_sensor_interrupt_handler(void* user_data, uint8_t value)
{
    WICED_BT_TRACE("Interrupt Handler\r\n");

    /* Disable the GPIO interrupt */
    DISABLE_GPIO_INTERRUPT;

    /* Check if device is still connected and CCCD==1. If not then disable the
     * interrupt on the sensor */
    if((0 == motion_sensor_hostinfo.sensor_value_characteristic_client_configuration)
                                         || (0 == conn_id))
    {
        motion_sensor_acc_interrupt_disable();
    }
    /* Start notification and idle timer */
    else
    {
       if(WICED_BT_SUCCESS != wiced_start_timer(&notification_timer_handle,
                                                           NOTFICATION_TIME_MS))
       {
           WICED_BT_TRACE("Notification timer start failed\r\n");
       }
       if(WICED_BT_SUCCESS != wiced_start_timer(&idle_timer_handle, IDLE_TIME_S))
       {
           WICED_BT_TRACE("Idle timer start failed\r\n");
       }
    }
}

/**
 * Function         notification_timer_callback
 *
 * @brief           Notification timer callback
 * @param[in] user_data                : arg (not used)
 *
 * @return                          : None
 */
void notification_timer_callback(uint32_t arg)
{
    send_sensor_value_notification();
}

/**
 * Function         idle_timer_callback
 *
 * @brief           Idle timer callback
 * @param[in] user_data                : arg (not used)
 *
 * @return                          : None
 */
void idle_timer_callback(uint32_t arg)
{
    UINT32 int_status;
    wiced_result_t timer_result;

    /* Check the current status of the motion sensor interrupt signal */
    int_status = wiced_hal_gpio_get_pin_input_status(MOTION_SENSOR_INTERRUPT);

    /* If interrupt signal not high stop timers, enable interrupt */
    if(0 == int_status)
    {
        if(WICED_BT_SUCCESS != wiced_stop_timer(&notification_timer_handle))
        {
            WICED_BT_TRACE("Notification timer stop failed\r\n");
        }
        if(WICED_BT_SUCCESS != wiced_stop_timer(&idle_timer_handle))
        {
            WICED_BT_TRACE("Idle timer stop failed\r\n");
        }
        ENABLE_GPIO_INTERRUPT;
    }
    /* If interrupt signal high restart Idle timer */
    else
    {
        if(wiced_is_timer_in_use(&idle_timer_handle))
        {
            if(WICED_BT_SUCCESS != wiced_stop_timer(&idle_timer_handle))
            {
                WICED_BT_TRACE("Idle timer stop failed\r\n");
            }
        }
        if(WICED_BT_SUCCESS != wiced_start_timer(&idle_timer_handle, 10))
        {
            WICED_BT_TRACE("Idle timer start failed\r\n");
        }
        else
        {
            WICED_BT_TRACE("idle timer re started \r\n");
        }
    }
}

/**
 * Function         button_cb
 *
 * @brief           Button callback
 *
 * @return                          : None
 */
void button_cb (void* user_data, uint8_t value)
{
    wiced_result_t result;
    uint16_t written_byte;

    if(0 == conn_id)
    {
        memset(&motion_sensor_hostinfo, 0, sizeof(motion_sensor_hostinfo));
        wiced_hal_write_nvram(MOTION_SENSOR_VS_ID, sizeof(motion_sensor_hostinfo),
                (uint8_t *)&motion_sensor_hostinfo, &result);

        WICED_BT_TRACE("Peer bond data removed\r\n");

        /* Disable the interrupt */
        motion_sensor_acc_interrupt_disable();

        /* Set the advertising params and make the device discoverable */
        motion_sensor_set_advertisement_data();

        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         NULL, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements %d\r\n", result);
    }
}

/**
 * Function         puart_rx_interrupt_callback
 *
 * @brief           Callback function for handling Serial terminal inputs from the user
 *
 * @return                       : None
 */
void puart_rx_interrupt_callback(void* unused)
{
    /* There can be at most 16 bytes in the HW FIFO. */
    uint8_t  readbyte;
    wiced_result_t result;

    wiced_hal_puart_read(&readbyte);

    switch (user_input_state)
    {
        case NO_ACTION:

            break;

        case GET_PASSKEY:
            if ((readbyte>='0')&&(readbyte<='9'))
            {
                passkey = passkey * 10 + readbyte - '0';
                passkey_itr++;
                if (passkey_itr == 6)
                {
                    user_input_state = NO_ACTION;
                    motion_sensor_passkey_reply(passkey);
                    passkey_itr = 0;
                    passkey = 0;
                }
            }
    }

    wiced_hal_puart_reset_puart_interrupt();
}

/**
 * Function         motion_sensor_set_console_input
 *
 * @brief           For initializing the console input via PUART RX Interrupt
 *
 * @return                       : None
 */
void motion_sensor_set_console_input(void)
{
    /* Turn off flow control */
    wiced_hal_puart_flow_off();  /* call  to turn on flow control */

    /* BEGIN - puart interrupt */
    wiced_hal_puart_register_interrupt(puart_rx_interrupt_callback);

    /* set watermak level to 1 to receive interrupt up on receiving each byte */
    wiced_hal_puart_set_watermark_level(1);

    /* Turn on Tx */
    wiced_hal_puart_enable_tx();
}
