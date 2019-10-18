/*
 * $ Copyright Cypress Semiconductor $
 */

 /** @file
 * File name: motion_sensor.c
 *
 * Description: Main file which is the entry point to the application.
 *              The application_start function initializes UART for trace
 *              messages and initializes the BT stack.
 *
 * Features demonstrated:
 *              I2C communication,BLE advertisement, BLE connection and
 *              notifications.
 *
 *  Controls:
 *  - Remove bond data with button press
 *
 */

#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "app_bt_cfg.h"
#include "wiced_platform.h"
#include "wiced_bt_stack.h"

/******************************************************************************
 *                   External Variables and Functions
 ******************************************************************************/
extern wiced_result_t motion_sensor_management_cback(wiced_bt_management_evt_t event,
                                 wiced_bt_management_evt_data_t *p_event_data);

/******************************************************************************
 *                                Constants
 ******************************************************************************/
/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Sets trace to PUART and register BLE management event callback
*
* Parameters:
*   None
*
* Return:
*  None
*
*******************************************************************************/
void application_start(void)
{
    /* Route Trace messages to PUART */
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    WICED_BT_TRACE("\n\r-------------------------------------------------\r\n\n"
                       "                  Motion Sensor                  \r\n\n"
                       "-------------------------------------------------\n\r");

    /* Register call back and configuration with stack */
     if(WICED_BT_SUCCESS != wiced_bt_stack_init(motion_sensor_management_cback ,
                     &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools))
     {
         WICED_BT_TRACE("Stack Init failed\n");
     }
}
