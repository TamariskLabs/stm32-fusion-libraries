#include "fusion_can_open.h"


// ------------------------- MODULE MACROS ---------------------

// NMT command service codes
#define NMT_SERVICE_CODE_START 		 	0x01
#define NMT_SERVICE_CODE_STOP 		 	0x02
#define NMT_SERVICE_CODE_ENTER_PREOP 	0x80
#define NMT_SERVICE_CODE_RESET_NODE  	0x81
#define NMT_SERVICE_CODE_RESET_COMMS 	0x82


// ------------------ MODULE STATIC VARIABLES ------------------

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
static HAL_StatusTypeDef fusion_can_open_nmt_message_callback(CAN_Message_t *msg)
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
				return HAL_ERROR;
				break;
		}
		// return success if command was processed
		return HAL_OK;
	}
	// ignore and just return success if the command was not for this node
	return HAL_OK;
}

// Callback function to handle SYNC messages
static HAL_StatusTypeDef fusion_can_open_sync_message_callback(CAN_Message_t *msg)
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
		return HAL_ERROR;
	}
}

// Callback to handle Time Stamp messages
static HAL_StatusTypeDef fusion_can_open_time_stamp_message_callback(CAN_Message_t *msg)
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
		return HAL_OK;
	}
	else {
		// invalid message length, ignore the message
		return HAL_ERROR;
	}
}

// Callback to handle PDO1 messages
static HAL_StatusTypeDef fusion_can_open_pdo1_message_callback(CAN_Message_t *msg)
{

}

// Callback to handle PDO2 messages
static HAL_StatusTypeDef fusion_can_open_pdo2_message_callback(CAN_Message_t *msg)
{

}

// Callback to handle PDO3 messages
static HAL_StatusTypeDef fusion_can_open_pdo3_message_callback(CAN_Message_t *msg)
{

}

// Callback to handle PDO4 messages
static HAL_StatusTypeDef fusion_can_open_pdo4_message_callback(CAN_Message_t *msg)
{

}

// Call back to handle SDO messages
static HAL_StatusTypeDef fusion_can_open_sdo_message_callback(CAN_Message_t *msg)
{

}


// ---------------- MODULE FUNCTION DEFINITIONS ------------------


// Initializes can bus hardware and configures the node based upon settings from the can_open_device_profile file
// Call one time during device startup
HAL_StatusTypeDef fusion_can_open_init(FDCAN_HandleTypeDef *hcan)
{

	// register the NMT message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_nmt_message_callback, CANOPEN_NMT_ID);
	// register the SYNC message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_sync_message_callback, CANOPEN_SYNC_ID);
	// register the Time Stamp message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_time_stamp_message_callback, CANOPEN_TIME_ID);
	// register the PDO1 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo1_message_callback, (CANOPEN_PDO1_ID | device_profile_node_id));
	// register the PDO2 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo2_message_callback, (CANOPEN_PDO2_ID | device_profile_node_id));
	// register the PDO3 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo3_message_callback, (CANOPEN_PDO3_ID | device_profile_node_id));
	// register the PDO4 message handler
	fusion_can_bus_register_rx_callback(fusion_can_open_pdo4_message_callback, (CANOPEN_PDO4_ID | device_profile_node_id));
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
		heartbeat_msg.id = (CANOPEN_NMT_ERROR_CONTROL_ID | device_profile_node_id);
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
