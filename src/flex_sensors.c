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
#define FLEX_PWR_WAIT      100
// Conversion time 1/sampling rate
#define CONVERSION_WAIT    9
// I2C slave addresses for each flex sensor
#define FLEX_ONE_ADDR      0x48 // Default GND
#define FLEX_TWO_ADDR      0x49 // VDD
// Flex sensors power pin PC6
#define FLEX_PWR_PORT      gpioPortC
#define FLEX_PWR_PIN       7

// I2C register addresses
#define CONFIG_REGISTER        0x01
#define CONVERSION_REGISTER    0x00
// I2C config register bit masks (16 bit register)
#define SINGLE_SHOT            BIT(8)
#define START_CONVERSION       BIT(15) // This bit can only be set during config
#define ADC_CHANNEL_0          BIT(14)
#define ADC_CHANNEL_1          BIT(14) | BIT(12)
#define SAMPLE_RATE_128HZ      0x0000
#define ADS1015_CONFIG_PGA_23  0x0080
#define DISABLE_COMP           BIT(1) | BIT(0)

#define SENSOR_1_CONFIG        (SINGLE_SHOT | START_CONVERSION | \
		                        ADC_CHANNEL_0 | SAMPLE_RATE_128HZ | \
								ADS1015_CONFIG_PGA_23 | DISABLE_COMP)
#define SENSOR_2_CONFIG        (SINGLE_SHOT | START_CONVERSION | \
		                        ADC_CHANNEL_1 | SAMPLE_RATE_128HZ | \
								ADS1015_CONFIG_PGA_23 | DISABLE_COMP)

/*****************************************************************************
 *                            G L O B A L S                                  *
 *****************************************************************************/

// I2C globals
volatile static I2C_TransferReturn_TypeDef	status;
static I2C_TransferSeq_TypeDef write_cmd;
static I2C_TransferSeq_TypeDef read_cmd;
static uint8_t read_buf[2]; // {MSB, LSB}
static uint8_t write_config_buf[3]= {CONFIG_REGISTER,
		                             (SENSOR_2_CONFIG & 0xFF00) >> 8,
									 SENSOR_2_CONFIG & 0x00FF};
//static uint8_t write_config_buf[3]= {0x01, 0xC1, 0x83};
static uint8_t write_convert_buf[1] = {CONVERSION_REGISTER};


// Current and next states for flex sensor measurement
static FLEX_STATE_e current_flex_state = FLEX_OFF;
static FLEX_STATE_e next_flex_state    = FLEX_OFF;

extern DEVICE_STATE_e device_state;

/*****************************************************************************
 *                    P R I V A T E   F U N C T I O N S                      *
 *****************************************************************************/

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
 * @brief Send a write command to config register
 * @param addr 7 bit I2C address to write to
 *
 */
static void I2C_config_write(uint16_t addr)
{
	write_cmd.addr = addr << 1;
	write_cmd.flags = I2C_FLAG_WRITE;
	write_cmd.buf[0].data = write_config_buf;
	write_cmd.buf[0].len = 3;
//	printf("0x%X, 0x%X, 0x%X", write_config_buf[0], write_config_buf[1], write_config_buf[2]);
	I2C_TransferInit(I2C0, &write_cmd);
	I2C_IntEnable(I2C0, I2C_FLAG_WRITE);
}


/**
 *
 * @brief Send a write command to conversion register
 * @param addr 7 bit I2C address to write to
 *
 */
static void I2C_conversion_write(uint16_t addr)
{
	write_cmd.addr = addr << 1;
	write_cmd.flags = I2C_FLAG_WRITE;
	write_cmd.buf[0].data = write_convert_buf;
	write_cmd.buf[0].len = 1;
	I2C_TransferInit(I2C0, &write_cmd);
	I2C_IntEnable(I2C0, I2C_FLAG_WRITE);
}


/**
 *
 * @brief Send a read command to module
 * @param addr 7 bit I2C address to write to
 *
 */
static void I2C_read(uint16_t addr)
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
 * @brief Power on the flex sensors
 *
 */
void flex_power_on(void)
{
	GPIO_PinOutSet(FLEX_PWR_PORT, FLEX_PWR_PIN);
}


/**
 *
 * @brief Power off the flex sensors
 *
 */
void flex_power_off(void)
{
	GPIO_PinOutClear(FLEX_PWR_PORT, FLEX_PWR_PIN);
	if (device_state == DEVICE_OFF)
	{
		set_flex_current_state(FLEX_OFF);
	}
}



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
				I2C_config_write(FLEX_TWO_ADDR);
				set_flex_next_state(FLEX_CONFIG_COMPLETE);
			}
			break;

		case FLEX_CONFIG_COMPLETE:
			if (ext_signal & FLEX_TRANSFER_COMPLETE)
			{
				I2C_IntDisable(I2C0, I2C_FLAG_WRITE);
				if (status != i2cTransferDone)
				{
					printf("Error occurred during I2C config write! Error code is %d\r\n",
							status);
					flex_power_off();
					set_flex_next_state(FLEX_OFF);
				}
				else
				{
					timerSetEventInMs(CONVERSION_WAIT, FLEX_SENSOR);
					set_flex_next_state(FLEX_CONVERSION_COMPLETE);
				}
			}
			break;

		case FLEX_CONVERSION_COMPLETE:
			if (ext_signal & FLEX_TIMER_WAIT)
			{
				I2C_conversion_write(FLEX_TWO_ADDR);
				set_flex_next_state(FLEX_READ_READY);
			}
			break;

		case FLEX_READ_READY:
			if (ext_signal & FLEX_TRANSFER_COMPLETE)
			{
				I2C_IntDisable(I2C0, I2C_FLAG_WRITE);
				if (status != i2cTransferDone)
				{
					printf("Error occurred during I2C conversion write! Error code is %d\r\n",
							status);
					flex_power_off();
					set_flex_next_state(FLEX_OFF);
				}
				else
				{
					I2C_read(FLEX_TWO_ADDR);
					set_flex_next_state(FLEX_READ_COMPLETE);
				}
			}
			break;

		case FLEX_READ_COMPLETE:
			if (ext_signal & FLEX_TRANSFER_COMPLETE)
			{
				I2C_IntDisable(I2C0, I2C_FLAG_READ);
				if (status != i2cTransferDone)
				{
					printf("Error occurred during I2C read! Error code is %d\r\n",
							status);
//					flex_power_off();
				}
				else
				{
					// Handle value
					printf("Flex ADC Value: %d\r\n", ((read_buf[0] << 8) | read_buf[1]) >> 4);
				}
				flex_power_off();
				set_flex_next_state(FLEX_OFF);
			}
			break;

		default:
			break;
	}

	// Transition to the next state
	if (current_flex_state != next_flex_state)
	{
//		printf("State %d transitioned to State %d\r\n",
//				current_flex_state, next_flex_state);
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
	flex_power_on();
	// Initialize I2C
	I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
	I2CSPM_Init(&i2cInit);
	NVIC_EnableIRQ(I2C0_IRQn);
}


void reset_i2c_pins(void)
{
	current_flex_state = FLEX_OFF;
	next_flex_state    = FLEX_OFF;
	/* Output value must be set to 1 to not drive lines low. Set
	 SCL first, to ensure it is high before changing SDA. */
	GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAndPullUp, 1);
	GPIO_PinModeSet(gpioPortC, 11, gpioModeWiredAndPullUp, 1);

	/* In some situations, after a reset during an I2C transfer, the slave
	 device may be left in an unknown state. Send 9 clock pulses to
	 set slave in a defined state. */
	for (int i = 0; i < 9; i++) {
		GPIO_PinOutSet(gpioPortC, 10);
		GPIO_PinOutClear(gpioPortC, 10);
	}
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
