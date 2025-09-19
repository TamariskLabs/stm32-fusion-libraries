#ifndef FUSION_CAN_OPEN_H
#define FUSION_CAN_OPEN_H

#include "main.h"
#include "fusion_can_interface.h"
#include "can_open_device_profile.h"
#include <stdint.h>

// This module pulls device profile configuration settings
// from user defined header and source file called can_open_device_profile.h/.c
// a description of what should be included in these files is contained in the
// README.md file contained in this component directory.


// ------------------------Module Type Definitions-----------------------------------------


// defines the different operational states the device can be in
typedef enum {
	DEVICE_STATE_INITIALIZING = 0,
	DEVICE_STATE_PRE_OPERATIONAL = 1,
	DEVICE_STATE_OPERATIONAL = 2,
	DEVICE_STATE_STOPPED = 3
} CANOpen_Device_State_t;

// defines different message function code types for CANopen
// the function code is the encoded as the upper 4 bits of the 11-bit CAN ID
// the lower 7 bits are used for the node ID
typedef enum {
    CANOPEN_NMT_ID         = 0x000,          /**< Network Management, a command for start, stop, reset, and state change for all nodes on the network */
    CANOPEN_SYNC_ID        = 0x080,          /**< Synchronization */
	CANOPEN_TIME_ID        = 0x100,          /**< Time Stamp */
    CANOPEN_EMERGENCY_ID   = 0x080,          /**< Emergency */
    CANOPEN_TPDO1_ID       = 0x180,          /**< Transmit Process Data Object 1 */
    CANOPEN_RPDO1_ID       = 0x200,          /**< Receive Process Data Object 1 */
    CANOPEN_TPDO2_ID       = 0x280,          /**< Transmit Process Data Object 2 */
    CANOPEN_RPDO2_ID       = 0x300,          /**< Receive Process Data Object 2 */
    CANOPEN_TPDO3_ID       = 0x380,          /**< Transmit Process Data Object 3 */
    CANOPEN_RPDO3_ID       = 0x400,          /**< Receive Process Data Object 3 */
    CANOPEN_TPDO4_ID       = 0x480,          /**< Transmit Process Data Object 4 */
    CANOPEN_RPDO4_ID       = 0x500,          /**< Receive Process Data Object 4 */
    CANOPEN_SDO_TX_ID      = 0x580,          /**< Service Data Object Transmit */
    CANOPEN_SDO_RX_ID      = 0x600,          /**< Service Data Object Receive */
	CANOPEN_HEARTBEAT_ID   = 0x700,		     /**< Resets the internal software heartbeat watchdog timer */
} CANOpen_Function_Code_t;

// defines the possible error codes specified by CiA 301
typedef enum {
	CANOPEN_NO_ERROR 	   			= 0x0000,
	CANOPEN_UNSPECIFIED_ERROR 		= 0x1000,
	CANOPEN_OVERCURRENT_ERROR		= 0x2001,
	CANOPEN_UNDERVOLTAGE_ERROR		= 0x3001,
	CANOPEN_OVERVOLTAGE_ERROR		= 0x3002,
	CANOPEN_OVER_TEMP_ERROR			= 0x4001,
	CANOPEN_INTERNAL_HARDWARE_ERRPR	= 0x5000,
	CANOPEN_INTERNAL_SOFTWARE_ERROR	= 0x6000,
	CANOPEN_EXTERNAL_MONITOR_ERRPR	= 0x7000,
	CANOPEN_COMMUNICATION_ERROR		= 0x8000,
	CANOPEN_MESSAGE_OVERRUN_ERROR	= 0x8020,
	CANOPEN_LOST_HEARTBEAT_ERROR	= 0x8081,
	CANOPEN_EXTERNAL_TRIGGER_ERROR	= 0x9000,
	CANOPEN_MFG_SPECIFIC_ERROR		= 0xF000

} CANOpen_Error_Code_t;

// defines a Service Data Object (SDO) command specifier
typedef enum {
    SDO_INITIATE_DOWNLOAD        = 0x20, /**< Initiate SDO Download (Client to Server) */
    SDO_INITIATE_UPLOAD          = 0x40, /**< Initiate SDO Upload (Server to Client) */
    SDO_DOWNLOAD_SEGMENT         = 0x00, /**< SDO Download Segment (Client to Server) */
    SDO_UPLOAD_SEGMENT           = 0x60, /**< SDO Upload Segment (Server to Client) */
    SDO_ABORT_TRANSFER           = 0x80  /**< SDO Abort Transfer */
} CANOpen_SDO_Command_Specifier_t;


// defines an object dictionary entry
typedef struct {
    uint16_t index;          /**< Object dictionary index */
    char object_name[32];    /**< Name of the object */
    uint8_t object_code;     /**< Object code (e.g., variable, array, record) */
    uint8_t data_type;       /**< Data type (e.g., integer, float, string) */
    uint8_t access_type;     /**< Access type (e.g., read-only, read-write) */
    char attribute[32];      /**< Attribute (e.g., default, constant) */
    bool is_mandatory;       /**< Indicates if the object is mandatory */
    uint8_t category;        /**< Object category (e.g., communication, device) */
} CANOpen_Object_Dictionary_Entry_t;

// structure to hold a CANopen message
typedef struct {
    uint8_t function_code;                    /**< CANopen function code */
    uint8_t node_id;                          /**< CANopen node ID */
    uint16_t data_length;                     /**< Length of the data in bytes */
    uint8_t data[MAX_CAN_MESSAGE_LENGTH];     /**< Data payload (max length adjusted for function code and node ID) */
} CANOpen_Message_t;


// --------------------- MODULE FUNCTION PROTOTYPES --------------------------

// Initializes can bus hardware and configures the node based upon settings from the can_open_device_profile file
// Call one time during device startup
HAL_StatusTypeDef fusion_can_open_init(FDCAN_HandleTypeDef *hcan);

// Updates and performs tasks required for the fusion CANOpen module
void fusion_can_open_update(void);

// returns the current operational state of the node (Initialized, Stopped, Ect)
CANOpen_Device_State_t fusion_can_open_get_state(void);

// send an emergency packet to the master node containing the corresponding code
// and places the device in the DEVICE_STATE_STOPPED state
HAL_StatusTypeDef fusion_can_open_send_emergency(CANOpenERROR error);


#endif
