/**
 *
 * @file   flex_sensors.c
 * @author Guanxiong Fu
 * @date   Nov 30 2019
 * @brief  Implementation file for flex sensor related functions
 *
 */

/*****************************************************************************
 *                           I N C L U D E S                                 *
 *****************************************************************************/

#include "flex_sensors.h"


/*****************************************************************************
 *                            D E F I N E S                                  *
 *****************************************************************************/

// Power up time
#define FLEX_PWR_WAIT      1
// Conversion time
#define CONVERSION_WAIT    1
// I2C slave addresses for each flex sensor
#define FLEX_ONE_ADDR      0x48 // Default GND
#define FLEX_TWO_ADDR      0x49 // VDD
// Flex sensors power pin PC6
#define FLEX_PWR_PIN       gpioPortC
#define FLEX_PWR_PORT      6

// I2C config register bit masks
#define CONTINUOUS_TRANSFER    0x0000
#define SAMPLE_RATE_1600HZ     0x0080

/*****************************************************************************
 *                            G L O B A L S                                  *
 *****************************************************************************/

// I2C status
volatile static I2C_TransferReturn_TypeDef	status;
static I2C_TransferSeq_TypeDef write_cmd;
static I2C_TransferSeq_TypeDef read_cmd;
static uint8_t read_buf[2];
static uint8_t * write_buf;

// Current and next states for flex sensor measurement
static FLEX_STATE_e current_flex_state = FLEX_OFF;
static FLEX_STATE_e next_flex_state    = FLEX_OFF;

/*****************************************************************************
 *                    P R I V A T E   F U N C T I O N S                      *
 *****************************************************************************/

/**
 *
 * @brief Power on the flex sensors
 *
 */
static inline void flex_power_on(void)
{
	GPIO_PinOutSet(FLEX_PWR_PORT, FLEX_PWR_PIN);
}


/**
 *
 * @brief Power off the flex sensors
 *
 */
static inline void flex_power_off(void)
{
	GPIO_PinOutClear(FLEX_PWR_PORT, FLEX_PWR_PIN);
}


/**
 *
 * @brief Set the current_flex_state flag
 * @param new_state New state to set to
 *
 */
static void set_flex_current_state(FLEX_STATE_e new_state)
{
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	current_flex_state = new_state;
	CORE_EXIT_CRITICAL();
}


/**
 *
 * @brief Set the next_flex_state flag
 * @param new_state New state to set to
 *
 */
static void set_flex_next_state(FLEX_STATE_e new_state)
{
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	next_flex_state = new_state;
	CORE_EXIT_CRITICAL();
}



/**
 *
 * @brief Send a write command to Si7021
 *
 */
static void I2C_Write(uint16_t addr)
{
	write_cmd.addr = addr << 1;
	write_cmd.flags = I2C_FLAG_WRITE;
	write_cmd.buf[0].data = write_buf;
	write_cmd.buf[0].len = 1;
	I2C_TransferInit(I2C0, &write_cmd);
	I2C_IntEnable(I2C0, I2C_FLAG_WRITE);
}


/**
 *
 * @brief Send a read command to Si7021
 *
 */
static void I2C_Read(uint16_t addr)
{
	read_cmd.addr = addr << 1;
	read_cmd.flags = I2C_FLAG_READ;
	read_cmd.buf[0].data = read_buf;
	read_cmd.buf[0].len = 2;
	I2C_TransferInit(I2C0, &read_cmd);
	I2C_IntEnable(I2C0, I2C_FLAG_READ);
}


/*****************************************************************************
 *                      P U B L I C   F U N C T I O N S                      *
 *****************************************************************************/

/**
 *
 * @brief State machine for accelerometer readings
 *
 */
void flex_sensor_state_machine(uint32_t ext_signal)
{
	switch (current_flex_state)
	{
		case FLEX_OFF:
			if (ext_signal & FLEX_MEASURE)
			{
				flex_power_on();
				timerSetEventInMs(FLEX_PWR_WAIT, FLEX_SENSOR);
				set_flex_next_state(FLEX_POWER_ON);
			}
			break;

		case FLEX_POWER_ON:
			if (ext_signal & FLEX_TIMER_WAIT)
			{

			}
			break;

		case FLEX_WRITE_COMPLETE:
			break;

		case FLEX_CONVERSION_COMPLETE:
			break;

		case FLEX_READ_COMPLETE:
			break;

		default:
			break;
	}

	// Transition to the next state
	if (current_flex_state != next_flex_state)
	{
		set_flex_current_state(next_flex_state);
	}
}


/**
 *
 * @brief Initialization for flex sensors
 *
 */
void flex_sensor_init(void)
{
	// Flex sensor power pin
	GPIO_PinModeSet(FLEX_PWR_PORT, FLEX_PWR_PIN, gpioModePushPull, false);
	// Initialize I2C
	I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
	I2CSPM_Init(&i2cInit);
	NVIC_EnableIRQ(I2C0_IRQn);
}


/**
 *
 * @brief ISR for I2C0
 *
 */
void I2C0_IRQHandler(void)
{
	// Call I2C transfer function, read return value
	status = I2C_Transfer(I2C0);
	if (status != i2cTransferInProgress)
	{
		// Set transfer complete event bit
		gecko_external_signal(FLEX_TRANSFER_COMPLETE);
	}
}
