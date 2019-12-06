/**
 *
 * @file   letimer.h
 * @author Guanxiong Fu
 * @date   Nov 30 2019
 * @brief  Header file to configure the LETIMER
 *
 */

#ifndef __SRC_LETIMER_H__
#define __SRC_LETIMER_H__


/*****************************************************************************
 *                           I N C L U D E S                                 *
 *****************************************************************************/

/* Standard headers */
#include <stdint.h>

/* emlib headers */
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_core.h"

/* src headers */
#include "app.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"


/*****************************************************************************
 *                            D E F I N E S                                  *
 *****************************************************************************/

/* Sensor type defines to distinguish delays set for
 * either of the sensors state machines */
#define ACC_SENSOR          0
#define FLEX_SENSOR         1

/*****************************************************************************
 *                           T Y P E D E F S                                 *
 *****************************************************************************/


/*****************************************************************************
 *            P U B L I C   F U N C T I O N   D E C L A R A T I O N S        *
 *****************************************************************************/

void letimer_init(void);

void timerSetEventInMs(uint32_t ms_until_wakeup, uint8_t sensor_type);

uint32_t timerGetRunTimeMilliseconds(void);


#endif /* __SRC_LETIMER_H__ */
