/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
*
* This file provides definitions and function prototypes for Motion Sensor
* device
*
*/

#ifndef _MOTION_SENSOR_BLE_H_
#define _MOTION_SENSOR_BLE_H_

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define MAX_SECURITY_KEY_SIZE   (0x10)

#define MOTION_SENSOR_INTERRUPT (CYBSP_D6) /* Motion Sensor interrupt*/

#define NOTFICATION_TIME_MS     (500) /* 500 ms interval for notification */
#define IDLE_TIME_S             (10) /* 10 s interval for idle timeout */

/* VSIDs for NVRAM storage */
#define MOTION_SENSOR_VS_ID    WICED_NVRAM_VSID_START

/* Macros to disable abd enable interrupt */
#define ENABLE_GPIO_INTERRUPT wiced_hal_gpio_configure_pin(MOTION_SENSOR_INTERRUPT, \
                              (GPIO_INPUT_ENABLE|GPIO_PULL_DOWN|GPIO_EN_INT_RISING_EDGE), \
                              GPIO_PIN_OUTPUT_LOW)

#define DISABLE_GPIO_INTERRUPT wiced_hal_gpio_configure_pin(MOTION_SENSOR_INTERRUPT, \
                               (GPIO_INPUT_ENABLE|GPIO_PULL_DOWN|GPIO_INTERRUPT_DISABLE), \
                               GPIO_PIN_OUTPUT_LOW)

/* PUART related macro */
#define NO_ACTION (0)
#define GET_PASSKEY (6)

/* Connection parameters related macro */
#define CONN_INTERVAL (80)
#define CONN_LATENCY  (0)
#define SUP_TIMEOUT   (512)
/******************************************************************************
 *                                Typedefs
 ******************************************************************************/
#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
	uint16_t  conn_id;                  /* connection ID referenced by the stack */
    uint8_t   dev_prebonded;      /* stay connected or disconnect after all messages are sent */
    uint16_t  sensor_value_characteristic_client_configuration;  /* Current value of the client configuration descriptor */
    wiced_bt_device_link_keys_t link_keys; /* Value of the link keys */
    wiced_bt_local_identity_keys_t local_keys; /* value of local keys */
} host_info_t;
#pragma pack()
/******************************************************************************
*                             Function prototypes
******************************************************************************/
wiced_result_t motion_sensor_management_cback(wiced_bt_management_evt_t event,
		                          wiced_bt_management_evt_data_t *p_event_data);

void motion_sensor_application_init(void);

void motion_sensor_passkey_reply (uint32_t passkey);

void motion_sensor_smp_bond_result(uint8_t result);

wiced_bt_gatt_status_t motion_sensor_gatts_callback(wiced_bt_gatt_evt_t event,
		                                    wiced_bt_gatt_event_data_t *p_data);

void motion_sensor_set_advertisement_data(void);

wiced_bt_gatt_status_t motion_sensor_gatts_conn_status_cb(
		                           wiced_bt_gatt_connection_status_t *p_status);

wiced_bt_gatt_status_t motion_sensor_gatts_connection_up(
		                           wiced_bt_gatt_connection_status_t *p_status);

wiced_bt_gatt_status_t motion_sensor_gatts_connection_down(
		                           wiced_bt_gatt_connection_status_t *p_status);

wiced_bt_gatt_status_t motion_sensor_gatts_req_cb(
		                             wiced_bt_gatt_attribute_request_t *p_data);

gatt_db_lookup_table_t * motion_sensor_get_attribute(uint16_t handle);

wiced_bt_gatt_status_t motion_sensor_gatts_req_read_handler(uint16_t conn_id,
		                                    wiced_bt_gatt_read_t * p_read_data);

wiced_bt_gatt_status_t motion_sensor_gatts_req_write_handler(uint16_t conn_id,
		                                        wiced_bt_gatt_write_t * p_data);

wiced_bt_gatt_status_t motion_sensor_gatts_req_mtu_handler(uint16_t conn_id,
		                                                          uint16_t mtu);

void send_sensor_value_notification();

void motion_sensor_interrupt_handler(void* user_data, uint8_t value);

void notification_timer_callback(uint32_t arg);

void idle_timer_callback(uint32_t arg);

void button_cb (void* user_data, uint8_t value);

void shutdown_cb(void);

void motion_sensor_set_console_input(void);

#endif /* _MOTION_SENSOR_BLE_H_ */
