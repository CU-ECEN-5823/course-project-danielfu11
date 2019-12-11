/**
 *
 * @file   flex_sensors.h
 * @author Guanxiong Fu
 * @date   Nov 30 2019
 * @brief  Header file for flex sensor related defines, typedefs, functions
 *
 */

#ifndef __SRC_FLEX_SENSORS_H__
#define __SRC_FLEX_SENSORS_H__


/*****************************************************************************
 *                           I N C L U D E S                                 *
 *****************************************************************************/

/* Standard headers */
#include <stdint.h>
#include <stdio.h>

/* emlib headers */
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_i2c.h"
#include "i2cspm.h"

/* src headers */
#include "letimer.h"
#include "leds.h"
#include "app.h"
#include "utils.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"


/*****************************************************************************
 *                            D E F I N E S                                  *
 *****************************************************************************/


/*****************************************************************************
 *                           T Y P E D E F S                                 *
 *****************************************************************************/

// Flex sensor measurement states
// Currently only for 1 finger
typedef enum
{
	FLEX_OFF = 0,
	FLEX_POWER_ON,
	FLEX_CONFIG_COMPLETE,
	FLEX_CONVERSION_COMPLETE,
	FLEX_READ_READY,
	FLEX_READ_COMPLETE,
	FLEX_CONFIG_COMPLETE_2,
	FLEX_CONVERSION_COMPLETE_2,
	FLEX_READ_READY_2,
	FLEX_READ_COMPLETE_2,
	FLEX_CONFIG_COMPLETE_3,
	FLEX_CONVERSION_COMPLETE_3,
	FLEX_READ_READY_3,
	FLEX_READ_COMPLETE_3
} FLEX_STATE_e;


/*****************************************************************************
 *            P U B L I C   F U N C T I O N   D E C L A R A T I O N S        *
 *****************************************************************************/

void flex_power_on(void);

void flex_power_off(void);

void flex_sensor_state_machine(uint32_t ext_signal);

void flex_sensor_init(void);

void reset_flex(void);

#endif /* __SRC_FLEX_SENSORS_H__ */
