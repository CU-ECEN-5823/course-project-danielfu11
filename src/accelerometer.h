/**
 *
 * @file   accelerometer.h
 * @author Guanxiong Fu
 * @date   Nov 22 2019
 * @brief  Header file for accelerometer related defines, typedefs, functions
 *
 */

#ifndef __SRC_ACCELEROMETER_H__
#define __SRC_ACCELEROMETER_H__


/*****************************************************************************
 *                           I N C L U D E S                                 *
 *****************************************************************************/

/* Standard headers */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* emlib headers */
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_adc.h"

/* src headers */
#include "letimer.h"
#include "leds.h"
#include "app.h"

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

// Accelerometer measurement states
typedef enum
{
	ACC_OFF = 0,
	ACC_POWER_ON = 1,
	ACC_MEASUREMENT_DONE = 2,
} ACC_STATE_e;


/*****************************************************************************
 *            P U B L I C   F U N C T I O N   D E C L A R A T I O N S        *
 *****************************************************************************/

void accelerometer_state_machine(uint32_t ext_signal);

void accelerometer_init(void);


#endif /* __SRC_ACCELEROMETER_H__ */
