#include "fusion_can_open.h"
#include <string.h>


// ------------------------- MODULE MACROS ---------------------

// NMT command service codes
#define NMT_SERVICE_CODE_START 		 	0x01
#define NMT_SERVICE_CODE_STOP 		 	0x02
#define NMT_SERVICE_CODE_ENTER_PREOP 	0x80
#define NMT_SERVICE_CODE_RESET_NODE  	0x81
#define NMT_SERVICE_CODE_RESET_COMMS 	0x82

// SDO index definitions
#define OD_INDEX_DEVICE_TYPE                0x1000    // type of device according to CiA profiles
#define OD_INDEX_ERROR_REGISTER             0x1001    // error register (Summary of error bitfields)
#define OD_INDEX_MANUFACTURER_STATUS        0x1002    // manufacturer specific status register (Optional)
#define OD_INDEX_PREDEFINED_ERROR           0x1003    // 32-bit error code (log of the last errors that occurred)
#define OD_INDEX_COMM_CYCLE_PERIOD          0x1006    // SYNC communication cycle period in microseconds
#define OD_INDEX_HEARTBEAT_TIME       		0x1017    // Heartbeat time in milliseconds
#define OD_INDEX_MFG_DEVICE_NAME            0x1008    // Manufacturer device name (visible in CANopen network management)
#define OD_INDEX_MFG_HW_VERSION             0x1009    // Manufacturer hardware version (visible in CANopen network management)
#define OD_INDEX_MFG_SW_VERSION             0x100A    // Manufacturer software version (visible in CANopen network management)
#define OD_INDEX_GUARDING_TIME              0x100C    // Guarding time in milliseconds (Guard time is an older mechanism superceeded heartbeat)
#define OD_INDEX_GUARDING_TIMEOUT           0x100D    // Guarding life factor (multiplier for guarding time to determine timeout period)
#define OD_INDEX_STORE_PARAMETERS           0x1010    // Command to store parameters in non-volatile memory
#define OD_INDEX_RESTORE_DEFAULTS           0x1011    // Command to restore parameters to default values
#define OD_INDEX_TIME_STAMP_COB_ID          0x1012    // The COB-ID used for TIME messages
#define OD_INDEX_HIGH_RES_TIME_STAMP        0x1013    // High resolution time stamp (Optional)
#define OD_INDEX_EMERGENCY_COB_ID           0x1014    // The COB-ID used for EMERGENCY messages
#define OD_INDEX_INHIBIT_EMCY_TIME          0x1015    // inhibit time in milliseconds for EMERGENCY messages
#define OD_INDEX_CONSUMER_HEARTBEAT_TIME    0x1016    // Consumer heartbeat time (time to wait for heartbeat from another node)
#define OD_INDEX_PRODUCER_HEARTBEAT_TIME    0x1017    // Producer heartbeat time (time between sending heartbeats)
#define OD_INDEX_IDENTITY_OBJECT            0x1018    // Identity object (contains vendor ID, product code, revision number, serial number)
#define OD_INDEX_SYNC_COUNTER_OVERFLOW      0x1019    // SYNC counter overflow value (Optional) (Value at which SYNC counter rolls over to 0)
#define OD_INDEX_ERROR_BEHAVIOR             0x1029    // Error behavior (defines behavior on error conditions) (What state to enter on error)
#define OD_INDEX_SDO_SERVER_PARAM_FIRST     0x1200    // start index of server params. Normally the only one used, but more can exist if multiple clients will access the device frequently)
#define OD_INDEX_SDO_SERVER_PARAM_LAST      0x127F    // end index of server params
#define OD_INDEX_SDO_CLIENT_PARAM_FIRST     0x1280    // start index of client params
#define OD_INDEX_SDO_CLIENT_PARAM_LAST      0x12FF    // end index of client params
#define OD_INDEX_NMT_PARAMS_FIRST           0x1300    // start index of NMT params
#define OD_INDEX_NMT_PARAMS_LAST            0x137F    // end index of NMT params
#define OD_INDEX_PDO_COMMS_PARAMS_FIRST     0x1400    // start index of PDO communication params
#define OD_INDEX_PDO_COMMS_PARAMS_LAST      0x15FF    // end index of PDO communication params
#define OD_INDEX_PDO_MAPPING_TX_FIRST       0x1600    // start index of PDO mapping TX params
#define OD_INDEX_PDO_MAPPING_TX_LAST        0x16FF    // end index of PDO mapping TX params
#define OD_INDEX_PDO_MAPPING_RX_FIRST       0x1A00    // start index of PDO mapping RX params
#define OD_INDEX_PDO_MAPPING_RX_LAST        0x1BFF    // end index of PDO mapping RX params


// ------------------ MODULE STATIC VARIABLES ------------------

// pointer reference to the CAN handle structure
static FDCAN_HandleTypeDef *hcan_ptr = NULL;

// Holds the current operational state of the device
static CANOpen_Device_State_t device_state = DEVICE_STATE_INITIALIZING;

// holds the value of the last received NMT SYNC counter.
// 255 indicates no SYNC received yet.
// 0 indicates SYNC counter is disabled (initialize to 255 so first SYNC sets it to 0)
static uint8_t sync_counter = 255;

// last time a heartbeat message was received
static uint32_t last_heartbeat_time = 0;

// holds the most recently received time of day value from a Time Stamp message
static uint32_t time_of_day_ms = 0;						// milliseconds since midnight
static uint16_t time_of_day_days = 0;					// number of days since Jan 1, 1984
static uint32_t time_of_day_messaged_receied_time = 0;  // system time when the last time of day message was received


// ------------------- MODULE STATIC FUNCTIONS -----------------

// Callback function to handle NMT messages
static void fusion_can_open_nmt_message_callback(CAN_Message_t *msg)
{
	uint8_t command = msg->data[0]; 	// Extract the command byte
	uint8_t node_id = msg->data[1]; 	// Extract the node ID byte

	// Check if the command is for this node or for all nodes (node ID 0)
	if (node_id == device_profile_node_id || node_id == 0) {

		// update the device state based on the received command
		switch (command) {
			case NMT_SERVICE_CODE_START:
				device_state = DEVICE_STATE_OPERATIONAL;
				break;
			case NMT_SERVICE_CODE_STOP:
				device_state = DEVICE_STATE_STOPPED;
				break;
			case NMT_SERVICE_CODE_ENTER_PREOP:
				device_state = DEVICE_STATE_PRE_OPERATIONAL;
				break;
			case NMT_SERVICE_CODE_RESET_NODE:
				device_state = DEVICE_STATE_INITIALIZING;
				break;
			case NMT_SERVICE_CODE_RESET_COMMS:
				device_state = DEVICE_STATE_INITIALIZING;
				break;
			default:
				// return an error for unknown command
				break;
		}
		// return success if command was processed
	}
	// ignore and just return success if the command was not for this node
}

// Callback function to handle SYNC messages
static void fusion_can_open_sync_message_callback(CAN_Message_t *msg)
{

	// if the message has a counter update the sync counter
	if (msg->dlc == 1) {
		sync_counter = msg->data[0];
	}
	else if (msg->dlc == 0) {
		// if the message has no counter, set sync counter to 0 to indicate disabled
		sync_counter = 0;
	}
	else {
		// invalid message length, ignore the message
	}
}

// Callback to handle Heartbeat messages
static void fusion_can_open_heartbeat_message_callback(CAN_Message_t *msg)
{
	// create a response message structure
	CAN_Message_t response_msg = {0};
	response_msg.id = (CANOPEN_HEARTBEAT_ID | device_profile_node_id);
	response_msg.dlc = 1; 	// set the data length to 1 byte
	response_msg.data[0] = (uint8_t)device_state; // set the state byte

	// send the response message
	fusion_can_bus_send(hcan_ptr, &response_msg);
}

// Callback to handle Time Stamp messages
static void fusion_can_open_time_stamp_message_callback(CAN_Message_t *msg)
{
	// check if the message has 6 bytes - indicating valid time stamp data of CiA-301 V4.0 or lower
	// or
	// check if the message has 8 bytes - indicating valid time stamp data of CiA-301 V4.2 or higher
	if (msg->dlc == 6 || msg->dlc == 8) {
		// extract the time of day data from the message
		time_of_day_ms = (uint32_t)(msg->data[0]) | ((uint32_t)(msg->data[1]) << 8) | ((uint32_t)(msg->data[2]) << 16) | ((uint32_t)(msg->data[3]) << 24);
		time_of_day_days = (uint16_t)(msg->data[4]) | ((uint16_t)(msg->data[5]) << 8);
		// record the system time when the message was received
		time_of_day_messaged_receied_time = HAL_GetTick();
	}
	else {
		// invalid message length, ignore the message
	}
}

// Callback to handle SDO messages
static void fusion_can_open_sdo_message_callback(CAN_Message_t *msg)
{
	// get the command specifier from the message
	uint8_t command_specifier = (msg->data[0] & 0xE0);

	// get the index and sub-index from the message
	uint16_t index = (uint16_t)(msg->data[1]) | ((uint16_t)(msg->data[2]) << 8);
	uint8_t sub_index = msg->data[3];

	// check if this is a read request
	if (command_specifier == SDO_INITIATE_UPLOAD) {

		// create a response message structure
		CAN_Message_t response_msg = {0};
		response_msg.id = (CANOPEN_SDO_TX_ID | device_profile_node_id);
		response_msg.dlc = 8; // default to 8 bytes, may be changed later

		// set the index and sub-index
		response_msg.data[1] = (uint8_t)(index & 0xFF);
		response_msg.data[2] = (uint8_t)((index >> 8) & 0xFF);
		response_msg.data[3] = sub_index;

		// process the read request based upon the index and sub-index
		if (index == OD_INDEX_DEVICE_TYPE) {

			// set the expedited flag and last segment flag to true
			response_msg.data[0] = SDO_INITIATE_UPLOAD | 0x03;

			// set the number of unused bytes to 0 (4 bytes used for device type)
			response_msg.data[0] |= 0x00 << 2;

			// set the device type value
			response_msg.data[4] = (uint8_t)(device_profile_device_type & 0xFF);
			response_msg.data[5] = (uint8_t)((device_profile_device_type >> 8) & 0xFF);
			response_msg.data[6] = (uint8_t)((device_profile_device_type >> 16) & 0xFF);
			response_msg.data[7] = (uint8_t)((device_profile_device_type >> 24) & 0xFF);

			// send the response message
			fusion_can_bus_send(hcan_ptr, &response_msg);

		} else if (index == OD_INDEX_ERROR_REGISTER) {

		} else if (index == OD_INDEX_MANUFACTURER_STATUS) {

		} else if (index == OD_INDEX_PREDEFINED_ERROR) {

		} else if (index == OD_INDEX_COMM_CYCLE_PERIOD) {

		} else if (index == OD_INDEX_HEARTBEAT_TIME) {

		} else if (index == OD_INDEX_MFG_DEVICE_NAME) {

			// get the length of the device name string
			uint8_t name_length = strlen(device_profile_device_name);

			// if the name is less than or equal to 4 bytes, send it as an expedited transfer
			if (name_length <= 4) {
				// set the expedited flag and last segment flag to true
				response_msg.data[0] = SDO_INITIATE_UPLOAD | 0x03;

				// set the number of unused bytes
				response_msg.data[0] |= (4 - name_length) << 2;

				// copy the device name into the response data
				memset(&response_msg.data[4], 0, 4); // clear the data bytes
				memcpy(&response_msg.data[4], device_profile_device_name, name_length);

				// send the response message
				fusion_can_bus_send(hcan_ptr, &response_msg);
			}
			// if the name is longer than 4 bytes, send it as a segmented transfer
			else {
				// set the expedited flag and last segment flag to false
				response_msg.data[0] |= 0x00;

				// set the number of unused bytes to 0
				response_msg.data[0] |= 0x00 << 2;

				// send the response message
				fusion_can_bus_send(hcan_ptr, &response_msg);
			}

		} else if (index == OD_INDEX_MFG_HW_VERSION) {

			// get the length of the hardware version string
			uint8_t hw_version_length = strlen(device_profile_hardware_version);

			// if the version is less than or equal to 4 bytes, send it as an expedited transfer
			if (hw_version_length <= 4) {
				// set the expedited flag and last segment flag to true
				response_msg.data[0] = SDO_INITIATE_UPLOAD | 0x03;

				// set the number of unused bytes
				response_msg.data[0] |= (4 - hw_version_length) << 2;

				// copy the hardware version into the response data
				memset(&response_msg.data[4], 0, 4); // clear the data bytes
				memcpy(&response_msg.data[4], device_profile_hardware_version, hw_version_length);

				// send the response message
				fusion_can_bus_send(hcan_ptr, &response_msg);
			} else {
				// set the expedited flag and last segment flag to false
				response_msg.data[0] |= 0x00;

				// set the number of unused bytes to 0
				response_msg.data[0] |= 0x00 << 2;

				// send the response message
				fusion_can_bus_send(hcan_ptr, &response_msg);
			}

		} else if (index == OD_INDEX_MFG_SW_VERSION) {

			// get the length of the software version string
			uint8_t sw_version_length = strlen(device_profile_software_version);

			// if the version is less than or equal to 4 bytes, send it as an expedited transfer
			if (sw_version_length <= 4) {
				// set the expedited flag and last segment flag to true
				response_msg.data[0] = SDO_INITIATE_UPLOAD | 0x03;

				// set the number of unused bytes
				response_msg.data[0] |= (4 - sw_version_length) << 2;

				// copy the software version into the response data
				memset(&response_msg.data[4], 0, 4); // clear the data bytes
				memcpy(&response_msg.data[4], device_profile_software_version, sw_version_length);

				// send the response message
				fusion_can_bus_send(hcan_ptr, &response_msg);
			} else {
				// set the expedited flag and last segment flag to false
				response_msg.data[0] |= 0x00;

				// set the number of unused bytes to 0
				response_msg.data[0] |= 0x00 << 2;

				// send the response message
				fusion_can_bus_send(hcan_ptr, &response_msg);
			}

		} else if (index == OD_INDEX_GUARDING_TIME) {

		} else if (index == OD_INDEX_GUARDING_TIMEOUT) {

		} else if (index == OD_INDEX_STORE_PARAMETERS) {

		} else if (index == OD_INDEX_RESTORE_DEFAULTS) {

		} else if (index == OD_INDEX_TIME_STAMP_COB_ID) {

		} else if (index == OD_INDEX_HIGH_RES_TIME_STAMP) {

		} else if (index == OD_INDEX_EMERGENCY_COB_ID) {

		} else if (index == OD_INDEX_INHIBIT_EMCY_TIME) {

		} else if (index == OD_INDEX_CONSUMER_HEARTBEAT_TIME) {

		} else if (index == OD_INDEX_PRODUCER_HEARTBEAT_TIME) {

		} else if (index == OD_INDEX_IDENTITY_OBJECT) {

		} else if (index == OD_INDEX_SYNC_COUNTER_OVERFLOW) {

		} else if (index == OD_INDEX_ERROR_BEHAVIOR) {

		} else {
			// unsupported index, ignore the message
		}
	}
	// check if this is a read request for the next segment
	else if (command_specifier == SDO_UPLOAD_SEGMENT) {
		// SDO Read Request for next segment
		// process the read request for the next segment
	}
	// check if this is a write request
	else if (command_specifier == SDO_INITIATE_DOWNLOAD) {
		// SDO Write Request
		// process the write request based upon the index and sub-index
	}
	// check if this is a write request for the next segment
	else if (command_specifier == SDO_DOWNLOAD_SEGMENT) {
		// SDO Write Request for next segment
		// process the write request for the next segment
	}
	else {
		// unsupported command, ignore the message
	}

}

// Callback to handle PDO1 messages
static void fusion_can_open_pdo1_message_callback(CAN_Message_t *msg)
{
	// set the output states based upon the received data
	HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, msg->data[0] & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, msg->data[0] & 0x02 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


// 700 microns for each stack
// 10 filters a minute

// Callback to handle PDO2 messages
static void fusion_can_open_pdo2_message_callback(CAN_Message_t *msg)
{

}

// Callback to handle PDO3 messages
static void fusion_can_open_pdo3_message_callback(CAN_Message_t *msg)
{

}

// Callback to handle PDO4 messages
static void fusion_can_open_pdo4_message_callback(CAN_Message_t *msg)
{

}


// ---------------- MODULE FUNCTION DEFINITIONS ------------------


// Initializes can bus hardware and configures the node based upon settings from the can_open_device_profile file
// Call one time during device startup
HAL_StatusTypeDef fusion_can_open_init(FDCAN_HandleTypeDef *hcan)
{
	// store the CAN handle pointer for later use
	hcan_ptr = hcan;

	// register the NMT message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_nmt_message_callback, CANOPEN_NMT_ID);
	// register the SYNC message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_sync_message_callback, CANOPEN_SYNC_ID);
	// register the heartbeat message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_heartbeat_message_callback, (CANOPEN_HEARTBEAT_ID | device_profile_node_id));
	// register the Time Stamp message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_time_stamp_message_callback, CANOPEN_TIME_ID);
	// register the PDO1 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo1_message_callback, (CANOPEN_TPDO1_ID | device_profile_node_id));
	// register the PDO2 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo2_message_callback, (CANOPEN_TPDO2_ID | device_profile_node_id));
	// register the PDO3 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo3_message_callback, (CANOPEN_TPDO3_ID | device_profile_node_id));
	// register the PDO4 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo4_message_callback, (CANOPEN_TPDO4_ID | device_profile_node_id));
	// register the SDO message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_sdo_message_callback, (CANOPEN_SDO_RX_ID | device_profile_node_id));

	// Initialize the CAN bus interface
	if (fusion_can_bus_init(hcan) != HAL_OK) {
		return HAL_ERROR;
	}
	else {
		return HAL_OK;
	}
}


// Updates and performs tasks required for the fusion CANOpen module
void fusion_can_open_update(void)
{
	// check if its time to send a heartbeat message
	if ((HAL_GetTick() - last_heartbeat_time) > device_profile_heartbeat_interval_ms) {
		// send a heartbeat message
		CAN_Message_t heartbeat_msg;
		heartbeat_msg.id = (CANOPEN_HEARTBEAT_ID | device_profile_node_id);
		heartbeat_msg.dlc = 1;
		heartbeat_msg.data[0] = (uint8_t)device_state;
		fusion_can_bus_send(hcan_ptr, &heartbeat_msg);

		// update the last heartbeat time
		last_heartbeat_time = HAL_GetTick();
	}

	// update indicator led's based upon the current device state



}


// Returns the current operational state of the node (Initialized, Stopped, ect.)
CANOpen_Device_State_t fusion_can_open_get_state(void)
{
	return device_state;
}
