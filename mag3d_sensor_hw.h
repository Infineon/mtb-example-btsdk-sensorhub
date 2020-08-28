/** mag3d_sensor_hw.h
 *
 * This file provides definitions and function prototypes for Magnetic 3DSensor
 * device
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


#ifndef MAG3D_SENSOR_HW_H_
#define MAG3D_SENSOR_HW_H_

/******************************************************************************
 *                                Constants
 ******************************************************************************/

#define MAG3D_SLAVE_ADDRESS             (0x5E)      /* Sensor slave address */
#define MAG3D_INT_SCL                   (WICED_P28) /* GPIO for I2C clock */
#define MAG3D_ADD_SDA                   (WICED_P29) /* GPIO for I2C data */

/* Register masks for write registers */
#define MAG3D_CONFIG_REG1_MASK          (0x18u)
#define MAG3D_CONFIG_REG2_MASK          (0xFFu)
#define MAG3D_CONFIG_REG3_MASK          (0x1Fu)

/* Read and write data length for sensor registers */
#define MAG3D_DATA_READ_LEN             (0x9u)
#define MAG3D_CONFIG_WRITE_LEN          (0x4u)

/* MOD1 write registers for slave address select*/
#define MAG3D_MOD1_IIC_ADDR_0           (0x0<<5u)
#define MAG3D_MOD1_IIC_ADDR_1           (0x1<<5u)
#define MAG3D_MOD1_IIC_ADDR_2           (0x2<<5u)
#define MAG3D_MOD1_IIC_ADDR_3           (0x3<<5u)

#define MAG3D_MOD1_INT_DISABLE          (0x1<<2u)   /* INT disable register */
#define MAG3D_MOD1_FAST_MODE            (0x1<<1u)   /*Fast Mode at 400Khz*/
#define MAG3D_MOD1_LP_MODE              (0x1)       /*Low power Mode*/

/* Low power mode interval period config values*/
#define MAG3D_MOD2_LPP_100MS            (0x0u)
#define MAG3D_MOD2_LPP_12MS             (0x1u)

#define MAG3D_MOD2_PTT_ENABLE           (0x1<<5u)   /* Parity test enable */
#define MAG3D_MOD2_TEMP_DISABLE         (0x1<<7u)   /* Temperature read disable */

/******************************************************************************
 *                                Typedefs
 ******************************************************************************/
/* Structure to hold magnetic sensor data read from I2C */
typedef struct
{
    int32_t bx1:8;
    int32_t by1:8;
    int32_t bz1:8;
    int32_t temp1:4;
    int32_t diag0:4;
    int32_t bx0:4;
    int32_t by0:4;
    int32_t diag1:4;
    int32_t bz0:4;
    int32_t temp0:8;
    int32_t res0:8;
    int32_t res1:8;
    int32_t res2:8;
} __attribute__((packed)) tlv493d_t;


typedef struct
{
    int16_t bx;
    int16_t by;
    int16_t bz;

} __attribute__((packed)) sensorpack_t;

extern volatile uint8_t mag3d_shield_connected;

/******************************************************************************
*                             Function prototypes
******************************************************************************/


void mag3d_sensor_init(void);

void mag3d_sensor_read_val (sensorpack_t *sense);

void mag3d_sensor_update_gatt();

float mag3dsense_calc_flux(uint16_t val);

#endif /* MAG3D_SENSOR_HW_H_ */
