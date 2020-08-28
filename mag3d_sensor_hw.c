/** mag3d_sensor_hw.c
 *
 * Magnetic sensor implementaion
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

#include "math.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"
#include "wiced.h"
#include "wiced_platform.h"
#include "mag3d_sensor_hw.h"
#include "wiced_rtos.h"
#include "GeneratedSource/cycfg_gatt_db.h"


/******************************************************************************
 *                                Macros
 ******************************************************************************/
#define MAG3D_BITS_LENGTH           (12u)
#define MAG3D_SIGNED_BIT_MASK       (10u)
/* 0.13 from magnetic conversion (12bit) */
#define MAG3D_FLUX_CONV_CONST       (0.13f)

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                            Functions prototype
 ******************************************************************************/


/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
/* magnetic sensor shield connection status*/

volatile uint8_t mag3d_shield_connected = 0;

/******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/**
 * Function         mag3d_sensor_init
 *
 * @brief           This function initializes the 3D magnetic sensor
 *
 * @return          : None
 */
void mag3d_sensor_init(void)
{
    int32_t response = 0;
    uint8_t config_data[MAG3D_CONFIG_WRITE_LEN] = { 0 };
    uint8_t read_data[MAG3D_DATA_READ_LEN] = { 0 };

    /* Read the sensor */
    response = wiced_hal_i2c_read(read_data, MAG3D_DATA_READ_LEN, MAG3D_SLAVE_ADDRESS);

    if(WICED_SUCCESS == response)
    {
        config_data[1] |= (read_data[6] & MAG3D_CONFIG_REG1_MASK) | MAG3D_MOD1_IIC_ADDR_0 |
                                                     MAG3D_MOD1_LP_MODE; /* Config MOD1 Registers */
        config_data[2] |= (read_data[7] & MAG3D_CONFIG_REG2_MASK);
        config_data[3] |= (read_data[8] & MAG3D_CONFIG_REG3_MASK) | MAG3D_MOD2_TEMP_DISABLE; /* Config MOD2 Registers */

        /* Config the sensor */
        response = wiced_hal_i2c_write(config_data, MAG3D_CONFIG_WRITE_LEN, MAG3D_SLAVE_ADDRESS);

        if(WICED_SUCCESS != response)
        {
            WICED_BT_TRACE("Failed to reset the communication\r\n");
        }
        mag3d_shield_connected = 1u;
    }
    else
    {
        WICED_BT_TRACE("Failed to read data\r\n");
    }

}


/**
 * Function         mag3d_sensor_read_val
 *
 * @brief           This function gets 3d magnetic sensor value
 *
 * @param[in] sensorpack_t        : Structure to store 3D magnetic sensor value
 *
 * @return                       : None
 */
void mag3d_sensor_read_val (sensorpack_t *sense)
{

    int32_t response = 0;
    tlv493d_t magData;
    float final_value = 0.0f;

    response = wiced_hal_i2c_read((uint8_t *)&magData,MAG3D_DATA_READ_LEN,
                                                MAG3D_SLAVE_ADDRESS);

    if(WICED_SUCCESS != response)
    {
        WICED_BT_TRACE("Failed to get magnetic 3D sensor data Res= %d \r\n",response);
    }

    sense->bx = magData.bx1;
    sense->bx = sense->bx<<4u;
    sense->bx = sense->bx | (magData.bx0 & 0xFu);

    sense->by = magData.by1;
    sense->by = sense->by<<4u;
    sense->by = sense->by | (magData.by0 & 0xFu);

    sense->bz = magData.bz1;
    sense->bz = sense->bz<<4u;
    sense->bz = sense->bz | (magData.bz0 & 0xFu);

    final_value = mag3dsense_calc_flux(sense->bx);
    WICED_BT_TRACE("\r\nMag3D Bx: %c%d.%02d mT \r\n",(final_value<0)? '-':'+', ABS((int16_t)final_value),
                                            ABS((int)(final_value*100)%100));
    final_value = mag3dsense_calc_flux(sense->by);
    WICED_BT_TRACE("Mag3D By: %c%d.%02d mT \r\n",(final_value<0)? '-':'+', ABS((int16_t)final_value),
                                            ABS((int)(final_value*100)%100));
    final_value = mag3dsense_calc_flux(sense->bz);
    WICED_BT_TRACE("Mag3D Bz: %c%d.%02d mT \r\n\n",(final_value<0)? '-':'+', ABS((int16_t)final_value),
                                            ABS((int)(final_value*100)%100));

}


/**
 * Function         mag3d_sensor_update_gatt
 *
 * @brief           This function reads all sensors and updates the value in Gatt DB
 *
 * @return                       : None
 */
void mag3d_sensor_update_gatt()
{

    mag3d_sensor_read_val((sensorpack_t*)&app_sensor_hub_magnetic_3dsensor_notify[0]);
}


/*******************************************************************************
* Function Name: mag3dsense_calc_flux
********************************************************************************
* Summary:
*  calculate the magnetic flux value from raw data.
*
* Parameters:
*  uint16_t val : input data
*
*******************************************************************************/


float mag3dsense_calc_flux(uint16_t val){

    int16_t bflux = 0;
    int16_t bit = 0;
    int16_t k = 1;

    for(bit=0;bit < MAG3D_BITS_LENGTH;bit++)
    {
        bflux += k*((val>>bit) & 0x01);
        k=(bit == MAG3D_SIGNED_BIT_MASK)?(k*2)*(-1):(k*2);

    }
    return (float)bflux*MAG3D_FLUX_CONV_CONST;
}

