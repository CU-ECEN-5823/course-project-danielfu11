/***************************************************************************//**
 * @file  app.h
 * @brief Application header file
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <gecko_configuration.h>
#include "utils.h"

/***************************************************************************//**
 * @defgroup app Application Code
 * @brief Sample Application Implementation
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Application
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup app
 * @{
 ******************************************************************************/

// Enumerations for externals signals
typedef enum
{
	PB0_PRESS              = BIT(0),
	PB1_PRESS              = BIT(1),
	ACC_MEASURE            = BIT(2),
	ACC_TIMER_WAIT         = BIT(3),
	ACC_MEASURE_DONE       = BIT(4),
	FLEX_MEASURE           = BIT(5),
	FLEX_TIMER_WAIT        = BIT(6),
	FLEX_TRANSFER_COMPLETE = BIT(7),
	FINGER1_FLEXED         = BIT(8),
	FINGER2_FLEXED         = BIT(9),
	FINGER3_FLEXED         = BIT(10),
} EXT_SIGNAL_e;

// Device OFF means flex sensors are off
typedef enum
{
	DEVICE_OFF,
	DEVICE_ON
} DEVICE_STATE_e;


/***************************************************************************//**
 * Main application code.
 * @param[in] pConfig  Pointer to stack configuration.
 ******************************************************************************/
void appMain(gecko_configuration_t *pConfig);

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

#endif /* APP_H */
