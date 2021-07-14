/*******************************************************************************
 * File Name : sensors.h
 *
 * This file provides definitions and function prototypes for Sensors.
 */

/*******************************************************************************
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

#ifndef SENSORS_H
#define SENSORS_H

#include "lsm9ds1.h"
#include "ncu15wf.h"
#include "tle493d.h"

/******************************************************************************
 *                                Macros
 ******************************************************************************/

#define MOTION_SENSOR_TIME_MS       (100u)  /* 100 ms interval for notification */

#define MAG3D_TIME_MS               (100u)  /* 100 ms interval for notification */

#define TEMPERATURE_TIME_MS         (1000u) /* 1000 ms interval for notification */

/* CCCD Index values of Sensors */
enum
{
    MOTION_SENSOR_CCCD_INDEX,
    MAG3D_SENSOR_CCCD_INDEX,
    TEMP_SENSOR_CCCD_INDEX
};
/*******************************************************************************
 *                                Typedefs
 ******************************************************************************/

/*******************************************************************************
*                             Function prototypes
*******************************************************************************/

void sensors_init(void);
void mag3d_timer_start(void);
void mag3d_timer_stop(void);
void temp_timer_start(void);
void temp_timer_stop(void);
void motion_timer_start(void);
void motion_timer_stop(void);
void motion_sensor_update_gatt(void);
void mag3d_sensor_update_gatt(void);
void temp_sensor_update_gatt(void);


#endif /* SENSORS_H_ */
