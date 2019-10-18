/*
 * $ Copyright Cypress Semiconductor $
 */

 /** @file
 *
 * This header file contains macros and functions prototype for motion sensor
 *
 */

#ifndef HW_H
#define HW_H

#include "lsm9ds1_drivers/lsm9ds1_reg.h"
/******************************************************************************
 *                                Constants
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

void initialize_all_sensors (void);

void read_all_sensors_update_gatt();

void power_down_motion_sensor(void);

void power_up_motion_sensor(void);

void motion_sensor_acc_interrupt_enable(void);

void motion_sensor_acc_interrupt_disable(void);

#endif
