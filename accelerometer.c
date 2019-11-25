/**
 *
 * @file   accelerometer.c
 * @author Guanxiong Fu
 * @date   Nov 22 2019
 * @brief  implementation file for accelerometer related functions
 *
 */

#include "accelerometer.h"

// LETIMER timer frequency
#define LETIMER_CLK_FREQ	((uint32_t)1000)
// Convert from ms to ticks
#define TICKS_IN_MS(X)		LETIMER_CLK_FREQ*(X) / 1000
// Convert from us to ticks
#define TICKS_IN_US(X)		LETIMER_CLK_FREQ*(X) / 1000000
// Top value of LETIMER0
#define PERIOD				1000
// ADC sampling frequency
#define ADC_FREQ			16000000
// Number of ADC inputs
#define ADC_INPUTS			3 //1 for each of X, Y, Z axis

// Accelerometer values: X, Y, Z
volatile uint32_t current_acc[3] = {0, 0, 0};


/**
 *
 * @brief ADC and LETIMER initialization for the 3 axis accelerometer
 *
 */
void accelerometer_init(void)
{
	adc_init();
	letimer_init();
}


/**
 *
 * @brief Configure ADC in scan mode
 * @ref   Using the SiLabs peripheral examples as reference
 *
 */
static void adc_init()
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
 * @brief Configure the LETIMER to trigger ADC conversions
 *
 */
static void letimer_init(void)
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
			.comp0Top = true,
			.bufTop = false,
			.out0Pol = 0,
			.out1Pol = 0,
			.ufoa0 = letimerUFOANone,
			.ufoa1 = letimerUFOANone,
			.repMode = letimerRepeatFree,
	};
	LETIMER_Init(LETIMER0, &LETIMER_config);

	// Set COMP0 interrupt period
	LETIMER_CompareSet(LETIMER0, 0, TICKS_IN_MS(PERIOD));

	// Clear all interrupts and
	// enable LETIMER interrupts for COMP0
	LETIMER_IntClear(LETIMER0, _LETIMER_IF_MASK);
	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0);
	NVIC_EnableIRQ(LETIMER0_IRQn);

	// Start timer
	LETIMER_Enable(LETIMER0, true);
}

/**
 *
 * @brief Handles the LETIMER0 interrupt
 *
 */
static inline void handle_letimer_int(uint32_t flags)
{
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	if (flags & LETIMER_IF_COMP0)
	{
		ADC_Start(ADC0, adcStartScan);
	}
	CORE_EXIT_CRITICAL();
}

/**
 *
 * @brief ISR for LETIMER0
 *
 */
void LETIMER0_IRQHandler(void)
{
	// Get interrupts triggered
	uint32_t flags = LETIMER_IntGet(LETIMER0);
	// Clear all interrupts
	LETIMER_IntClear(LETIMER0, _LETIMER_IF_MASK);
	// Handle interrupts
	handle_letimer_int(flags);
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

		// Send external signal to BT stack indicating new accelerometer reading
		gecko_external_signal(EXT_SIGNAL_ADC_READING);
	}
}
