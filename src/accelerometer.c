/**
 *
 * @file   accelerometer.c
 * @author Guanxiong Fu
 * @date   Nov 22 2019
 * @brief  Implementation file for accelerometer related functions
 *
 */


/*****************************************************************************
 *                           I N C L U D E S                                 *
 *****************************************************************************/

#include "accelerometer.h"


/*****************************************************************************
 *                            D E F I N E S                                  *
 *****************************************************************************/

// ADC sampling frequency
#define ADC_FREQ          16000000
// Number of ADC inputs
#define ADC_INPUTS        3 //1 for each of X, Y, Z axis
// GPIO pin PA3 that provides power to accelerometer
#define ACC_PWR_PORT      gpioPortA
#define ACC_PWR_PIN       3
// Accelerometer requires at least 1ms to power on
#define ACC_PWR_WAIT      2
// Threshold difference value to determine a shake
#define SHAKE_THRESHOLD   600
// Timeout to turn device off when there is no shaking movement, 1 unit is 500ms
#define TIMEOUT           20


/*****************************************************************************
 *                            G L O B A L S                                  *
 *****************************************************************************/

// Accelerometer values: X, Y, Z
volatile static uint32_t current_acc[3] = {0, 0, 0};

// Current and next states for accelerometer measurement
static ACC_STATE_e current_acc_state = ACC_OFF;
static ACC_STATE_e next_acc_state    = ACC_OFF;

extern DEVICE_STATE_e device_state;

/*****************************************************************************
 *                    P R I V A T E   F U N C T I O N S                      *
 *****************************************************************************/

/**
 *
 * @brief Configure ADC in scan mode
 * @ref   Using the SiLabs peripheral examples as reference
 *
 */
static void adc_init(void)
{
	// Enable clocks required
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitScan_TypeDef initScan = ADC_INITSCAN_DEFAULT;

	// Modify init structs
	init.prescale   = ADC_PrescaleCalc(ADC_FREQ, 0);
	init.timebase = ADC_TimebaseCalc(0);

	initScan.diff          = false;       // single ended
	initScan.reference     = adcRef2V5;   // internal 2.5V reference
	initScan.resolution    = adcRes12Bit; // 12-bit resolution
	initScan.acqTime       = adcAcqTime4; // set acquisition time to meet minimum requirements
	initScan.fifoOverwrite = true;        // FIFO overflow overwrites old data

	// Select ADC inputs for each accelerometer axis, PD10 PD11 PD12
	ADC_ScanSingleEndedInputAdd(&initScan, adcScanInputGroup0, adcPosSelAPORT3XCH2);
	ADC_ScanSingleEndedInputAdd(&initScan, adcScanInputGroup1, adcPosSelAPORT3YCH3);
	ADC_ScanSingleEndedInputAdd(&initScan, adcScanInputGroup2, adcPosSelAPORT3XCH4);

	// Set scan data valid level (DVL) to 2
	ADC0->SCANCTRLX = (ADC0->SCANCTRLX & ~_ADC_SCANCTRLX_DVL_MASK) |
			          (((ADC_INPUTS - 1) << _ADC_SCANCTRLX_DVL_SHIFT) &
			           _ADC_SCANCTRLX_DVL_MASK);

	// Clear ADC Scan FIFO
	ADC0->SCANFIFOCLEAR = ADC_SCANFIFOCLEAR_SCANFIFOCLEAR;

	// Initialize ADC and Scans
	ADC_Init(ADC0, &init);
	ADC_InitScan(ADC0, &initScan);

	// Enable Scan interrupts
	ADC_IntEnable(ADC0, ADC_IEN_SCAN);

	// Enable ADC Interrupts
	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);
}


/**
 *
 * @brief Power on the accelerometer
 *
 */
static inline void acc_power_on(void)
{
	GPIO_PinOutSet(ACC_PWR_PORT, ACC_PWR_PIN);
}


/**
 *
 * @brief Power off the accelerometer
 *
 */
static inline void acc_power_off(void)
{
	GPIO_PinOutClear(ACC_PWR_PORT, ACC_PWR_PIN);
}


/**
 *
 * @brief Set the current_acc_state flag
 * @param new_state New state to set to
 *
 */
static void set_acc_current_state(ACC_STATE_e new_state)
{
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	current_acc_state = new_state;
	CORE_EXIT_CRITICAL();
}


/**
 *
 * @brief Set the next_acc_state flag
 * @param new_state New state to set to
 *
 */
static void set_acc_next_state(ACC_STATE_e new_state)
{
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	next_acc_state = new_state;
	CORE_EXIT_CRITICAL();
}


/**
 *
 * @brief Compare current acc values to previous ones to
 *        detect shaking movement of device
 *
 */
static void detect_shake(void)
{
	int i;
	static bool initialized = false;
	static uint32_t prev_acc[3] = {0, 0, 0};
	static uint8_t active_time = 0;
	if (!initialized)
	{
		for (i=0; i < ADC_INPUTS; ++i)
		{
			prev_acc[i] = current_acc[i];
		}
		initialized = true;
	}
//	printf("X: %d, Y: %d, Z: %d\r\n", current_acc[0], current_acc[1], current_acc[2]);
	// Determine if there's a shake motion
	if (abs(prev_acc[0] - current_acc[0]) > SHAKE_THRESHOLD ||
		abs(prev_acc[1] - current_acc[1]) > SHAKE_THRESHOLD ||
		abs(prev_acc[2] - current_acc[2]) > SHAKE_THRESHOLD)
	{
		printf("Device turn on\r\n");
		led_set_state(LED_STATE_ON);
		reset_i2c_pins();
		device_state = DEVICE_ON;
		active_time = 0;
	}
	// If there's no shake motion, calculate how long it's been idle
	else
	{
		if (device_state == DEVICE_ON)
		{
			active_time++;
			// Turn off when idle time has reached 10 seconds
			if (active_time == TIMEOUT)
			{
				printf("Device turn off\r\n");
				device_state = DEVICE_OFF;
				flex_power_off();
				led_set_state(LED_STATE_OFF);
			}
		}
	}
	// Compare values
	if (initialized)
	{
		for (i=0; i < ADC_INPUTS; ++i)
		{
			prev_acc[i] = current_acc[i];
		}
	}
}


/*****************************************************************************
 *                      P U B L I C   F U N C T I O N S                      *
 *****************************************************************************/

/**
 *
 * @brief State machine for accelerometer readings
 *
 */
void accelerometer_state_machine(uint32_t ext_signal)
{
	switch (current_acc_state)
	{
		case ACC_OFF:
			if (ext_signal & ACC_MEASURE)
			{
				//printf("Power on acc\r\n");
				// Power on accelerometer
				acc_power_on();
				// Wait 1ms of turn on time
				timerSetEventInMs(ACC_PWR_WAIT, ACC_SENSOR);

				set_acc_next_state(ACC_POWER_ON);
			}
			break;

		case ACC_POWER_ON:
			if (ext_signal & ACC_TIMER_WAIT)
			{
				//printf("Start acc conversion\r\n");
				// Start next
				ADC_Start(ADC0, adcStartScan);

				set_acc_next_state(ACC_MEASUREMENT_DONE);
			}
			break;

		case ACC_MEASUREMENT_DONE:
			if (ext_signal & ACC_MEASURE_DONE)
			{
				//printf("Detect shake\r\n");
				// Detect shaking movement and turn device on/off based on results
				detect_shake();
				// Power off accelerometer
				acc_power_off();

				set_acc_next_state(ACC_OFF);
			}
			break;

		default:
			break;
	}

	// Transition to the next state
	if (current_acc_state != next_acc_state)
	{
		set_acc_current_state(next_acc_state);
	}
}


/**
 *
 * @brief Peripheral initialization for the 3 axis accelerometer
 *
 */
void accelerometer_init(void)
{
	// Accelerator power pin
	GPIO_PinModeSet(ACC_PWR_PORT, ACC_PWR_PIN, gpioModePushPull, false);
	adc_init();
	letimer_init();
}


/**
 *
 * @brief ISR for ADC0
 *
 */
void ADC0_IRQHandler(void)
{
	uint32_t i, id;
	CORE_DECLARE_IRQ_STATE;
	// Get each ADC conversion result
	for (i=0; i < ADC_INPUTS; ++i)
	{
		// Store accX, accY, accZ in global array
		CORE_ENTER_CRITICAL();
		current_acc[i] = ADC_DataIdScanGet(ADC0, &id);
		CORE_EXIT_CRITICAL();
	}
	// Signal that accelerometer reading is complete
	gecko_external_signal(ACC_MEASURE_DONE);
}
