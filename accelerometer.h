/**
 *
 * @file   accelerometer.h
 * @author Guanxiong Fu
 * @date   Nov 22 2019
 * @brief  Header file for accelerometer related functions
 *
 */

#ifndef __ACCELEROMETER_H__
#define __ACCELEROMETER_H__

#include <stdint.h>
#include <stdbool.h>
#include "sleep.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_adc.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#define EXT_SIGNAL_ADC_READING	0x03

void accelerometer_init(void);

#endif /* __ACCELEROMETER_H__ */
