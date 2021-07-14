/******************************************************************************
* File Name:   lsm9ds1.h
*
* Description: This file provides definitions and function prototypes for
*              motion sensor.
*
* Related Document: See README.md
*
*******************************************************************************
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
******************************************************************************/

#ifndef LSM9DS1_H
#define LSM9DS1_H

#include "lsm9ds1_reg.h"

/******************************************************************************
 *                                Macros
 ******************************************************************************/
#define DELAY_FOR_SENSOR_WAKEUP (10) /* 10 ms */

#define XL_THRESHOLD (50)

#define DATA_READY (1)
/******************************************************************************
 *                                Typedefs
 ******************************************************************************/

/******************************************************************************
*                             Function prototypes
******************************************************************************/
void get_accel_val (axis3bit16_t* accel_struct);

void get_gyro_val (axis3bit16_t* gyro_struct);

void get_mag_val (axis3bit16_t* gyro_struct);

void motion_sensor_init (void);

void power_down_motion_sensor(void);

void power_up_motion_sensor(void);

#endif /* LSM9DS1_H_ */
