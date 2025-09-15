#ifndef FUSION_CAN_OPEN_H
#define FUSION_CAN_OPEN_H

#include "main.h"
#include "fusion_can_interface.h"
#include <stdint.h>


// ------------------------Module Type Definitions-----------------------------------------

// defines different message function code types for CANopen
// the function code is the encoded as the upper 4 bits of the 11-bit CAN ID
// the lower 7 bits are used for the node ID
typedef enum {
    CANOPEN_NMT         = 0x00,          /**< Network Management, a command for start, stop, reset, and state change for all nodes on the network */
    CANOPEN_SYNC        = 0x01,          /**< Synchronization */
    CANOPEN_EMERGENCY   = 0x02,          /**< Emergency */
    CANOPEN_TIME        = 0x03,          /**< Time Stamp */
    CANOPEN_TPDO1       = 0x04,          /**< Transmit Process Data Object 1 */
    CANOPEN_RPDO1       = 0x05,          /**< Receive Process Data Object 1 */
    CANOPEN_TPDO2       = 0x06,          /**< Transmit Process Data Object 2 */
    CANOPEN_RPDO2       = 0x07,          /**< Receive Process Data Object 2 */
    CANOPEN_TPDO3       = 0x08,          /**< Transmit Process Data Object 3 */
    CANOPEN_RPDO3       = 0x09,          /**< Receive Process Data Object 3 */
    CANOPEN_TPDO4       = 0x0A,          /**< Transmit Process Data Object 4 */
    CANOPEN_RPDO4       = 0x0B,          /**< Receive Process Data Object 4 */
    CANOPEN_SDO_TX      = 0x0C,          /**< Service Data Object Transmit */
    CANOPEN_SDO_RX      = 0x0D,          /**< Service Data Object Receive */
    CANOPEN_HEARTBEAT   = 0x0E           /**< Heartbeat */
} canopen_function_code_t;

// defines a Service Data Object (SDO) command specifier
typedef enum {
    SDO_INITIATE_DOWNLOAD        = 0x20, /**< Initiate SDO Download (Client to Server) */
    SDO_INITIATE_UPLOAD          = 0x40, /**< Initiate SDO Upload (Server to Client) */
    SDO_DOWNLOAD_SEGMENT         = 0x00, /**< SDO Download Segment (Client to Server) */
    SDO_UPLOAD_SEGMENT           = 0x60, /**< SDO Upload Segment (Server to Client) */
    SDO_ABORT_TRANSFER           = 0x80  /**< SDO Abort Transfer */
} sdo_command_specifier_t;

// defines a Process Data Object (PDO) type
typedef enum {
    PDO_TYPE_SYNC = 0,   /**< Synchronous PDO */
    PDO_TYPE_ASYNC = 1   /**< Asynchronous PDO */
} pdo_type_t;

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
} object_dictionary_entry_t;

// structure to hold a CANopen message
typedef struct {
    uint8_t function_code;                    /**< CANopen function code */
    uint8_t node_id;                          /**< CANopen node ID */
    uint16_t data_length;                     /**< Length of the data in bytes */
    uint8_t data[MAX_CAN_MESSAGE_LENGTH];     /**< Data payload (max length adjusted for function code and node ID) */
} canopen_message_t;


// --------------------- MODULE FUNCTION PROTOTYPES --------------------------


HAL_StatusTypeDef fusion_can_open_init(FDCAN_HandleTypeDef *hcan);


#endif