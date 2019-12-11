/***************************************************************************//**
 * @file  app.c
 * @brief Application code
 * @note  Modified by Guanxiong Fu
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

/* Bluetooth stack headers */
#include "native_gecko.h"
#include "gatt_db.h"
#include "mesh_generic_model_capi_types.h"

/* Sensor client header */
#include "sensor_client.h"

/* Buttons and LEDs headers */
#include "buttons.h"
#include "leds.h"

/* Sensors headers */
#include "accelerometer.h"
#include "flex_sensors.h"

/* Display Interface header */
#include "display_interface.h"

/* Retarget serial headers */
#include "retargetserial.h"
#include <stdio.h>

#include "app.h"

/***************************************************************************//**
 * @addtogroup Application
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup app
 * @{
 ******************************************************************************/

/*******************************************************************************
 * Provisioning bearers defines.
 ******************************************************************************/
#define PB_ADV   0x1 ///< Advertising Provisioning Bearer
#define PB_GATT  0x2 ///< GATT Provisioning Bearer

/*******************************************************************************
 * Timer handles enums.
 ******************************************************************************/
typedef enum
{
	TIMER_ID_RESTART           = 78,
	TIMER_ID_FACTORY_RESET     = 77,
	TIMER_ID_PROVISIONING      = 66,
	TIMER_ID_SENSOR_DATA       = 65,
	TIMER_ID_SENSOR_DESCRIPTOR = 64,
	TIMER_ID_SAVE_STATE        = 60,
} TIMER_ID_e;

#define TIMER_CLK_FREQ ((uint32_t)32768) ///< Timer Frequency used
/// Convert miliseconds to timer ticks
#define TIMER_MS_2_TICKS(ms) ((TIMER_CLK_FREQ * (ms)) / 1000)
/// Time equal 0 removes the scheduled timer with the same handle
#define TIMER_REMOVE  0

/// Flag for indicating DFU Reset must be performed
static uint8_t boot_to_dfu = 0;
/// Number of active Bluetooth connections
static uint8_t num_connections = 0;
/// Handle of the last opened LE connection
static uint8_t conn_handle = 0xFF;
/// Flag for indicating that initialization was performed
static uint8_t init_done = 0;

DEVICE_STATE_e device_state = DEVICE_OFF;

// Indicates how many LPNs are connected through friendship
static uint8_t lpn_num = 0;

// Index for traversing property id array
typedef enum
{
	VOLT_INDEX,
	PEOPLE_INDEX,
	TEMP_INDEX,
} INDEX_e;

// Data structure to store into persistent storage
static PACKSTRUCT(struct assist_state {
	uint8_t lpn1_assist_current;
	uint8_t lpn1_assist_target;
	uint8_t lpn2_assist_current;
	uint8_t lpn2_assist_target;
}) assist_state;

/*******************************************************************************
 * Function prototypes.
 ******************************************************************************/
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *pEvt);

/***************************************************************************//**
 * Initialise used bgapi classes.
 ******************************************************************************/
void gecko_bgapi_classes_init(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	gecko_bgapi_class_mesh_node_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	gecko_bgapi_class_mesh_generic_server_init();
	gecko_bgapi_class_mesh_sensor_client_init();
	gecko_bgapi_class_mesh_friend_init();
}

/*******************************************************************************
 * Main application code.
 * @param[in] pConfig  Pointer to stack configuration.
 ******************************************************************************/
void appMain(gecko_configuration_t *pConfig)
{
	// Initialize stack
	gecko_stack_init(pConfig);
	gecko_bgapi_classes_init();

	// Initialize coexistence interface. Parameters are taken from HAL config.
	gecko_initCoexHAL();

	// Initialize debug prints and display interface
	RETARGET_SerialInit();
	RETARGET_SerialCrLf(true);
	DI_Init();

	// Initialize buttons
	button_init();

	// Initialize accelerometer and flex sensors
	accelerometer_init();
	flex_sensor_init();

	while (1) {
		// Event pointer for handling events
		struct gecko_cmd_packet* evt;

		// If there are no events pending then the next call to gecko_wait_event()
		// may cause device go to deep sleep.
		// Make sure that debug prints are flushed before going to sleep
		if (!gecko_event_pending()) {
			RETARGET_SerialFlush();
		}

		// Check for stack event
		evt = gecko_wait_event();

		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	}
}

/***************************************************************************//**
 * This function is called to initiate factory reset. Factory reset may be
 * initiated by keeping one of the pushbuttons pressed during reboot.
 * Factory reset is also performed if it is requested by the provisioner
 * (event gecko_evt_mesh_node_reset_id).
 ******************************************************************************/
static void initiate_factory_reset(void)
{
	printf("factory reset\r\n");
	DI_Print("\n***\nFACTORY RESET\n***", DI_ROW_STATUS);

	// If connection is open then close it before rebooting
	if (conn_handle != 0xFF) {
		gecko_cmd_le_connection_close(conn_handle);
	}

	// Perform a factory reset by erasing PS storage. This removes all the keys
	// and other settings that have been configured for this node
	gecko_cmd_flash_ps_erase_all();
	// Reboot after a small delay
	gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(2000),
			TIMER_ID_FACTORY_RESET,
			1);
}

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] pAddr  Pointer to Bluetooth address.
 ******************************************************************************/
static void set_device_name(bd_addr *pAddr)
{
	char name[20];
	uint16_t result;

	// Create unique device name using the last two bytes of the Bluetooth address
	snprintf(name, 20, "sensor client %02x:%02x",
			pAddr->addr[1], pAddr->addr[0]);

	printf("Device name: '%s'\r\n", name);

	result = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name,
			0,
			strlen(name),
			(uint8_t *)name)->result;
	if (result) {
		printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n",
				result);
	}

	// Show device name on the LCD
	DI_Print(name, DI_ROW_NAME);
}


/***************************************************************************//**
 * This function loads the saved assistance state from Persistent Storage and
 * copies the data in the global variable assist_state.
 * If PS key with ID 0x4004 does not exist or loading failed,
 * assist_state is set to zero and some default values are written to it.
 *
 * @return 0 if loading succeeds. -1 if loading fails.
 ******************************************************************************/
static int assist_state_load(void)
{
	struct gecko_msg_flash_ps_load_rsp_t* pLoad;

	pLoad = gecko_cmd_flash_ps_load(0x4004);

	// Set default values if ps_load fail or size of assist_state has changed
	if (pLoad->result || (pLoad->value.len != sizeof(struct assist_state))) {
		memset(&assist_state, 0, sizeof(struct assist_state));
		assist_state.lpn1_assist_current = 0;
		assist_state.lpn2_assist_current = 0;
		assist_state.lpn1_assist_target = 0;
		assist_state.lpn2_assist_target = 0;
		return -1;
	}

	memcpy(&assist_state, pLoad->value.data, pLoad->value.len);

	printf("Assistance state restored! lpn1: %d, lpn2: %d\r\n",
			assist_state.lpn1_assist_current, assist_state.lpn2_assist_current);

	return 0;
}


/***************************************************************************//**
 * This function saves the current assistance state in Persistent Storage
 * so that the data is preserved over reboots and power cycles.
 * The light state is hold in a global variable assist_state.
 * A PS key with ID 0x4004 is used to store the whole struct.
 *
 * @return 0 if saving succeed, -1 if saving fails.
 ******************************************************************************/
static int assist_state_store(void)
{
	struct gecko_msg_flash_ps_save_rsp_t* pSave;

	pSave = gecko_cmd_flash_ps_save(0x4004, sizeof(struct assist_state), (const uint8*)&assist_state);

	if (pSave->result) {
		printf("assist_state_store(): PS save failed, code %x\r\n", pSave->result);
		return(-1);
	}

	printf("Assistance state stored lpn1: %d, lpn2: %d\r\n",
				assist_state.lpn1_assist_current, assist_state.lpn2_assist_current);

	return 0;
}


/**
 *
 * Assistance state initialization
 * Called when node is initialized
 * Or when provisioning is completed
 *
 */
static void assist_state_init(void)
{
	memset(&assist_state, 0, sizeof(struct assist_state));

	if (assist_state_load() != 0)
	{
		printf("assist_state_load() failed, using defaults\r\n");
	}
}

/***************************************************************************//**
 * This function is called each time the assist state in RAM is changed.
 * It sets up a soft timer that will save the state in flash after small delay.
 * The purpose is to reduce amount of unnecessary flash writes.
 ******************************************************************************/
static void assist_state_changed(void)
{
	assist_state.lpn1_assist_current = assist_state.lpn1_assist_target;
	assist_state.lpn2_assist_current = assist_state.lpn2_assist_target;
	gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(5000), TIMER_ID_SAVE_STATE, 1);
}

/**
 *
 * @brief Client request handler for generic server model
 * @param model_id Model that received the message
 * @param element_index Element where the model resides
 * @param client_addr Mesh address of the client node that sent the request
 * @param server_addr Mesh destination address of the request; may be a
 * group address in case of a multicast request
 * @param appkey_index Application key index of the key used to encrypt
 * the request; the response, if any, has to be encrypted with the same key.
 * @param req Request parameters
 * @param transition_ms Requested transition time in milliseconds, or zero
 * for immediate state transition
 * @param delay_ms Requested delay time in milliseconds before a state
 * transition or an immediate state change should occur
 * @param request_flags Request flags
 */
static void client_request(uint16_t model_id,
		uint16_t element_index,
		uint16_t client_addr,
		uint16_t server_addr,
		uint16_t appkey_index,
		const struct mesh_generic_request *req,
		uint32_t transition_ms,
		uint16_t delay_ms,
		uint8_t request_flags)
{
	// Display if button is pressed or released based on ON/OFF state
	if (req->on_off == MESH_GENERIC_ON_OFF_STATE_ON)
	{
		// Based on Generic ON/OFF server address display node requests assistance
		if (server_addr == 0xC000)
		{
			assist_state.lpn1_assist_target = 1;
			DI_Print("LPN1 assistance", 8);
		}
		else if (server_addr == 0xC001)
		{
			assist_state.lpn2_assist_target = 1;
			DI_Print("LPN2 assistance", 9);
		}
	}
	else if (req->on_off == MESH_GENERIC_ON_OFF_STATE_OFF)
	{
		// Patient no longer requires assistance
		if (server_addr == 0xC000)
		{
			assist_state.lpn1_assist_target = 0;
			DI_Print("", 8);
		}
		else if (server_addr == 0xC001)
		{
			assist_state.lpn2_assist_target = 0;
			DI_Print("", 9);
		}
	}
	// If the assistance state changed, update persistent storage
	if (assist_state.lpn1_assist_target != assist_state.lpn1_assist_current ||
		assist_state.lpn2_assist_target != assist_state.lpn2_assist_current)
	{
		printf("State changed\r\n");
		assist_state_changed();
	}
}

/**
 * @brief State change handler
 * @param model_id       Server model ID.
 * @param element_index  Server model element index.
 * @param current        Pointer to current state structure.
 * @param target         Pointer to target state structure.
 * @param remaining_ms   Time (in milliseconds) remaining before transition
 *                       from current state to target state is complete.
 */
static void state_change(
		uint16_t model_id,
		uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target,
		uint32_t remaining_ms)
{
}

/***************************************************************************//**
 * Handling of boot event.
 * If needed it performs factory reset. In other case it sets device name
 * and initialize mesh node.
 ********************************i2c_pin**********************************************/
static void handle_boot_event(void)
{
	uint16_t result;
	char buf[30];
	// Check pushbutton state at startup.
	// If either PB0 or PB1 is held down then do factory reset
	if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0
			|| GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
		initiate_factory_reset();
	} else {
		struct gecko_msg_system_get_bt_address_rsp_t* pAddr =
				gecko_cmd_system_get_bt_address();
		set_device_name(&pAddr->address);
		// Initialize Mesh stack in Node operation mode, wait for initialized event
		result = gecko_cmd_mesh_node_init()->result;
		if (result) {
			snprintf(buf, 30, "init failed (0x%x)", result);
			DI_Print(buf, DI_ROW_STATUS);
		}
	}
}

/***************************************************************************//**
 * Handling of mesh node initialized event.
 * If device is provisioned it initializes the sensor server node.
 * If device is unprovisioned it starts sending Unprovisioned Device Beacons.
 *
 * @param[in] pEvt  Pointer to mesh node initialized event.
 ******************************************************************************/
static void handle_node_initialized_event(
		struct gecko_msg_mesh_node_initialized_evt_t *pEvt)
{
	printf("node initialized\r\n");
	gecko_cmd_mesh_generic_server_init();
	if (pEvt->provisioned) {
		printf("node is provisioned. address:%x, ivi:%ld\r\n",
				pEvt->address,
				pEvt->ivi);

		/* Initialize mesh lib */
		mesh_lib_init(malloc, free, 9);

		uint16_t res;
		//Initialize Friend functionality
		printf("Friend mode initialization\r\n");
		res = gecko_cmd_mesh_friend_init()->result;
		if (res) {
			printf("Friend init failed 0x%x\r\n", res);
		}

		// Set generic on/off server model
		mesh_lib_generic_server_register_handler(
				MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
				0,
				client_request,
				state_change);

		// Server publish
		mesh_lib_generic_server_publish(
				MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
				0,
				mesh_generic_state_on_off);

		gecko_cmd_mesh_sensor_client_init();
		enable_button_interrupts();
		assist_state_init();
		gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(100),
				TIMER_ID_SENSOR_DESCRIPTOR,
				1);
		DI_Print("provisioned", DI_ROW_STATUS);
	} else {
		printf("node is unprovisioned\r\n");
		DI_Print("unprovisioned", DI_ROW_STATUS);
		printf("starting unprovisioned beaconing...\r\n");
		// Enable ADV and GATT provisioning bearer
		gecko_cmd_mesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
	}
}

/***************************************************************************//**
 *  Handling of mesh node provisioning events.
 *  It handles:
 *   - mesh_node_provisioning_started
 *   - mesh_node_provisioned
 *   - mesh_node_provisioning_failed
 *
 *  @param[in] pEvt  Pointer to incoming provisioning event.
 ******************************************************************************/
void handle_node_provisioning_events(struct gecko_cmd_packet *pEvt)
{
	switch (BGLIB_MSG_ID(pEvt->header)) {
		case gecko_evt_mesh_node_provisioning_started_id:
			printf("Started provisioning\r\n");
			DI_Print("provisioning...", DI_ROW_STATUS);
	#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
			led_init(); /* shared GPIO pins used as LED output */
	#endif
			// start timer for blinking LEDs to indicate which node is being provisioned
			gecko_cmd_hardware_set_soft_timer(
					TIMER_MS_2_TICKS(250),
					TIMER_ID_PROVISIONING,
					0);
			break;

		case gecko_evt_mesh_node_provisioned_id:
			gecko_cmd_mesh_generic_server_init();
			/* Initialize mesh lib */
			mesh_lib_init(malloc, free, 9);
			// Set generic on/off server model
			mesh_lib_generic_server_register_handler(
					MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
					0,
					client_request,
					state_change);

			// Server publish
			mesh_lib_generic_server_publish(
					MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
					0,
					mesh_generic_state_on_off);

			uint16_t res;
			//Initialize Friend functionality
			printf("Friend mode initialization\r\n");
			res = gecko_cmd_mesh_friend_init()->result;
			if (res) {
				printf("Friend init failed 0x%x\r\n", res);
			}
			gecko_cmd_mesh_sensor_client_init();
			printf("node provisioned, got address=%x, ivi:%ld\r\n",
					pEvt->data.evt_mesh_node_provisioned.address,
					pEvt->data.evt_mesh_node_provisioned.iv_index);
			// stop LED blinking when provisioning complete
			gecko_cmd_hardware_set_soft_timer(TIMER_REMOVE, TIMER_ID_PROVISIONING, 0);
			led_set_state(LED_STATE_OFF);
			DI_Print("provisioned", DI_ROW_STATUS);

			enable_button_interrupts();
			assist_state_init();
			gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(20000),
					TIMER_ID_SENSOR_DESCRIPTOR,
					1);
			init_done = 1;
			break;

		case gecko_evt_mesh_node_provisioning_failed_id:
			printf("provisioning failed, code %x\r\n",
					pEvt->data.evt_mesh_node_provisioning_failed.result);
			DI_Print("prov failed", DI_ROW_STATUS);
			// start a one-shot timer that will trigger soft reset after small delay
			gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(2000),
					TIMER_ID_RESTART,
					1);
			break;

		default:
			break;
	}
}

/***************************************************************************//**
 *  Handling of le connection events.
 *  It handles:
 *   - le_connection_opened
 *   - le_connection_parameters
 *   - le_connection_closed
 *
 *  @param[in] pEvt  Pointer to incoming connection event.
 ******************************************************************************/
void handle_le_connection_events(struct gecko_cmd_packet *pEvt)
{
	switch (BGLIB_MSG_ID(pEvt->header)) {
		case gecko_evt_le_connection_opened_id:
			printf("evt:gecko_evt_le_connection_opened_id\r\n");
			num_connections++;
			conn_handle = pEvt->data.evt_le_connection_opened.connection;
			DI_Print("connected", DI_ROW_CONNECTION);
			break;

		case gecko_evt_le_connection_parameters_id:
			printf("evt:gecko_evt_le_connection_parameters_id: interval %u, latency %u, timeout %u\r\n",
					pEvt->data.evt_le_connection_parameters.interval,
					pEvt->data.evt_le_connection_parameters.latency,
					pEvt->data.evt_le_connection_parameters.timeout);
			break;

		case gecko_evt_le_connection_closed_id:
			// Check if need to boot to dfu mode
			if (boot_to_dfu) {
				// Enter to DFU OTA mode
				gecko_cmd_system_reset(2);
			}
			printf("evt:conn closed, reason 0x%x\r\n",
					pEvt->data.evt_le_connection_closed.reason);
			conn_handle = 0xFF;
			if (num_connections > 0) {
				if (--num_connections == 0) {
					DI_Print("", DI_ROW_CONNECTION);
				}
			}
			break;

		default:
			break;
	}
}

/***************************************************************************//**
 *  Entering to OTA DFU.
 *
 *  @param[in] connection  Connection that handle OTA DFU.
 ******************************************************************************/
void enter_to_dfu_ota(uint8_t connection)
{
	// Set flag to enter to OTA mode
	boot_to_dfu = 1;
	// Send response to Write Request
	gecko_cmd_gatt_server_send_user_write_response(connection,
			gattdb_ota_control,
			bg_err_success);
	// Close connection to enter to DFU OTA mode
	gecko_cmd_le_connection_close(connection);
}

/***************************************************************************//**
 *  Handling of timer events.
 *
 *  @param[in] handle  Timer handle that is serviced by this function.
 ******************************************************************************/
void handle_timer_event(uint8_t handle)
{
	switch (handle) {
		case TIMER_ID_FACTORY_RESET:
			gecko_cmd_system_reset(0);
			break;

		case TIMER_ID_RESTART:
			gecko_cmd_system_reset(0);
			break;

		case TIMER_ID_PROVISIONING:
			if (!init_done) {
				led_set_state(LED_STATE_PROV);
			}
			break;

		case TIMER_ID_SENSOR_DESCRIPTOR:
			sensor_client_publish_get_descriptor_request();
			gecko_cmd_hardware_set_soft_timer(
					TIMER_MS_2_TICKS(2000),
					TIMER_ID_SENSOR_DATA,
					0);
			break;

		case TIMER_ID_SENSOR_DATA:
			sensor_client_publish_get_request();
			break;

		case TIMER_ID_SAVE_STATE:
			assist_state_store();
			break;

		default:
			break;
	}
}

/***************************************************************************//**
 *  Handling of external signal events.
 *
 *  @param[in] signal  External signal handle that is serviced by this function.
 ******************************************************************************/
void handle_external_signal_event(uint32_t signal)
{
	if (signal & PB1_PRESS)
	{
		sensor_client_publish_get_descriptor_request();
		gecko_cmd_hardware_set_soft_timer(
				TIMER_MS_2_TICKS(2000),
				TIMER_ID_SENSOR_DATA,
				0);
	}
	if (signal & FINGER1_FLEXED)
	{
		sensor_client_change_property(VOLT_INDEX);
	}
	if (signal & FINGER2_FLEXED)
	{
		sensor_client_change_property(PEOPLE_INDEX);
	}
	if (signal & FINGER3_FLEXED)
	{
		sensor_client_change_property(TEMP_INDEX);
	}
	// Accelerometer state machine
	accelerometer_state_machine(signal);
	if (device_state == DEVICE_ON)
	{
		// I2C state machine for flex sensors
		flex_sensor_state_machine(signal);
	}
}

/***************************************************************************//**
 * Handling of stack events. Both BLuetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] pEvt    Pointer to incoming event.
 ******************************************************************************/
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *pEvt)
{
	if (NULL == pEvt) {
		return;
	}

	// Handle events
	switch (evt_id) {
	case gecko_evt_system_boot_id:
		handle_boot_event();
		break;

	case gecko_evt_mesh_node_initialized_id:
		handle_node_initialized_event(&(pEvt->data.evt_mesh_node_initialized));
		break;

	case gecko_evt_mesh_generic_server_client_request_id:
		mesh_lib_generic_server_event_handler(pEvt);
		break;

	case gecko_evt_mesh_generic_server_state_changed_id:
		mesh_lib_generic_server_event_handler(pEvt);
		break;

	case gecko_evt_mesh_node_provisioning_started_id:
	case gecko_evt_mesh_node_provisioned_id:
	case gecko_evt_mesh_node_provisioning_failed_id:
		handle_node_provisioning_events(pEvt);
		break;

	case gecko_evt_mesh_sensor_client_descriptor_status_id:
	case gecko_evt_mesh_sensor_client_status_id:
		handle_sensor_client_events(pEvt);
		break;

	case gecko_evt_mesh_node_key_added_id:
		printf("got new %s key with index %x\r\n",
				pEvt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
						pEvt->data.evt_mesh_node_key_added.index);
		break;

	case gecko_evt_mesh_node_model_config_changed_id:
		printf("model config changed\r\n");
		break;

	case gecko_evt_mesh_node_reset_id:
		printf("evt: gecko_evt_mesh_node_reset_id\r\n");
		initiate_factory_reset();
		break;

	case gecko_evt_mesh_friend_friendship_established_id:
		printf("evt gecko_evt_mesh_friend_friendship_established, lpn_address=%x\r\n",
				pEvt->data.evt_mesh_friend_friendship_established.lpn_address);
		lpn_num++;
		if (lpn_num == 2)
		{
			DI_Print("2 FRIENDS", DI_ROW_FRIEND);
		}
		else
		{
			DI_Print("1 FRIEND", DI_ROW_FRIEND);
		}
		break;

	case gecko_evt_mesh_friend_friendship_terminated_id:
		printf("evt gecko_evt_mesh_friend_friendship_terminated, reason=%x\r\n",
				pEvt->data.evt_mesh_friend_friendship_terminated.reason);
		lpn_num--;
		if (lpn_num == 1)
		{
			DI_Print("1 FRIEND", DI_ROW_FRIEND);
		}
		else
		{
			DI_Print("NO LPN", DI_ROW_FRIEND);
		}
		break;

	case gecko_evt_le_connection_opened_id:
	case gecko_evt_le_connection_parameters_id:
	case gecko_evt_le_connection_closed_id:
		handle_le_connection_events(pEvt);
		break;

	case gecko_evt_gatt_server_user_write_request_id:
		if (gattdb_ota_control
				== pEvt->data.evt_gatt_server_user_write_request.characteristic) {
			enter_to_dfu_ota(pEvt->data.evt_gatt_server_user_write_request.connection);
		}
		break;

	case gecko_evt_hardware_soft_timer_id:
		handle_timer_event(pEvt->data.evt_hardware_soft_timer.handle);
		break;

	case gecko_evt_system_external_signal_id:
		handle_external_signal_event(pEvt->data.evt_system_external_signal.extsignals);
		break;

	default:
		//printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
		break;
	}
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
