/**
 *
 * @file   letimer.c
 * @author Guanxiong Fu
 * @date   Nov 30 2019
 * @brief  Implementation file to configure the LETIMER
 *
 */


/*****************************************************************************
 *                           I N C L U D E S                                 *
 *****************************************************************************/

#include "letimer.h"


/*****************************************************************************
 *                            D E F I N E S                                  *
 *****************************************************************************/

// LETIMER timer frequency
#define LETIMER_CLK_FREQ	((uint32_t)1000)
// Convert from ms to ticks
#define TICKS_IN_MS(X)		LETIMER_CLK_FREQ*(X) / 1000
// Convert from us to ticks
#define TICKS_IN_US(X)		LETIMER_CLK_FREQ*(X) / 1000000
// Top value of LETIMER0
#define PERIOD				1000


/*****************************************************************************
 *                            G L O B A L S                                  *
 *****************************************************************************/

// Timer count rollover indicator for timestamp calculation
volatile static uint32_t	rollover_cnt = 0;

extern DEVICE_STATE_e device_state;

/*****************************************************************************
 *                    P R I V A T E   F U N C T I O N S                      *
 *****************************************************************************/


/*****************************************************************************
 *                      P U B L I C   F U N C T I O N S                      *
 *****************************************************************************/

/**
 *
 * @brief Configure the LETIMER
 *
 */
void letimer_init(void)
{
	/* Select clock sources and enable clock sources */
	CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO); // 1000 Hz
	CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_1);

	CMU_ClockEnable(cmuClock_LFA, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	const LETIMER_Init_TypeDef LETIMER_config =
	{
			.enable = false,
			.debugRun = false,
			.comp0Top = false,
			.bufTop = false,
			.out0Pol = 0,
			.out1Pol = 0,
			.ufoa0 = letimerUFOANone,
			.ufoa1 = letimerUFOANone,
			.repMode = letimerRepeatFree,
			.topValue = TICKS_IN_MS(PERIOD),
	};
	LETIMER_Init(LETIMER0, &LETIMER_config);

	// Clear all interrupts and
	// enable LETIMER interrupts for underflow and comp0
	LETIMER_IntClear(LETIMER0, _LETIMER_IF_MASK);
	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);
	NVIC_EnableIRQ(LETIMER0_IRQn);

	// Start timer
	LETIMER_Enable(LETIMER0, true);
}


/**
 *
 * @brief Sets up LETIMER0 to interrupt in a certain number of milliseconds,
 *		  device sleeps in EM1 meanwhile
 * @param ms_until_wakeup	Sleep time in microseconds
 * @param sensor_type       0 = accelerometer, 1 = flex sensor
 *
 */
void timerSetEventInMs(uint32_t ms_until_wakeup, uint8_t sensor_type)
{
	// Save the current number of ticks, 1 tick is 1ms
	uint32_t cnt_past = LETIMER0->CNT;
	uint32_t comp_val;
	if (cnt_past < TICKS_IN_MS(ms_until_wakeup))
	{
		comp_val = TICKS_IN_MS(PERIOD) - (TICKS_IN_MS(ms_until_wakeup) - cnt_past);
	}
	else
	{
		comp_val = cnt_past - TICKS_IN_MS(ms_until_wakeup);
	}
	if (sensor_type == ACC_SENSOR)
	{
		// Set COMP0 interrupt period
		LETIMER_CompareSet(LETIMER0, 0, comp_val);
		// Enable COMP0 interrupt
		LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0);
	}
	else if (sensor_type == FLEX_SENSOR)
	{
		// Set COMP1 interrupt period
		LETIMER_CompareSet(LETIMER0, 1, comp_val);
		// Enable COMP1 interrupt
		LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
	}
}


/**
 *
 * @brief Calculates the current run time in ms, used in loggerGetTimestamp()
 * @ret	  Current run time in ms
 *
 */
uint32_t timerGetRunTimeMilliseconds(void)
{
	uint32_t current_ticks = LETIMER0->CNT;
	return (rollover_cnt*3000 + current_ticks);
}


/**
 *
 * @brief ISR for LETIMER0
 *
 */
void LETIMER0_IRQHandler(void)
{
	uint32_t flags = LETIMER_IntGet(LETIMER0);
	// Clear all interrupts
	LETIMER_IntClear(LETIMER0, _LETIMER_IF_MASK);
	// Set LETIMER event flag
	if (flags & LETIMER_IF_UF)
	{
		rollover_cnt++;
		// Take accelerometer reading
		gecko_external_signal(ACC_MEASURE);
		// Take flex sensor reading only if device is on
		if (device_state == DEVICE_ON)
		{
			gecko_external_signal(FLEX_MEASURE);
		}
	}
	if (flags & LETIMER_IF_COMP0)
	{
		gecko_external_signal(ACC_TIMER_WAIT);
		// Only want this interrupt to trigger once every timerSetEventInMs() call
		LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP0);
	}
	if (flags & LETIMER_IF_COMP1)
	{
		gecko_external_signal(FLEX_TIMER_WAIT);
		// Only want this interrupt to trigger once every timerSetEventInMs() call
		LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
	}
}
