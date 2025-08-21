#include "fusion_can_interface.h"
#include <string.h>

// ------------------------- MODULE MACROS ---------------------
#define CAN_COMM_MAX_CALLBACKS 10

// ------------------ MODULE STATIC VARIABLES ------------------

// Pointer to the CAN handle structure
static FDCAN_HandleTypeDef *hcan_ptr;

// defines a callback type for received CAN messages
typedef void (*CAN_Comm_RxCallback_t)(const CAN_Message_t *msg);
// list of callbacks for routing received messages to
static CAN_Comm_RxCallback_t *rx_callbacks[CAN_COMM_MAX_CALLBACKS] = {NULL};
// keeps track of the number of registered callbacks
static uint8_t rx_callback_count = 0;

// ------------------- MODULE STATIC FUNCTIONS -----------------

/**
 * @brief called by the HAL rx interrupt handler to process received messages.
 * This function will route the message to any registered callback that matches the message ID.
 * @param msg Pointer to the received CAN message.
 */
static void fusion_can_interface_process_rx_message(const CAN_Message_t *msg)
{
    // iterate through each of the registered callbacks to route the message
    // to the appropriate handler
    for (uint8_t i = 0; i < rx_callback_count; i++) {
        if (rx_callbacks[i] != NULL) {
            rx_callbacks[i](msg);
        }
    }
}

// ------------------ FUNCTION DEFINITIONS ---------------------

/** 
 * @brief Initializes the CAN communication interface.
 * @param hcan Pointer to the CAN handle structure.
 * @return HAL status code.
 */
HAL_StatusTypeDef CAN_Comm_Init(FDCAN_HandleTypeDef *hcan)
{
    // Store the CAN handle pointer for later use
    hcan_ptr = hcan;

    // Start the CAN peripheral and validate it started successfully
    if (HAL_FDCAN_Start(hcan_ptr) != HAL_OK)
        return HAL_ERROR;

    // Optional filter to accept all messages
    FDCAN_FilterTypeDef f = {0};
	f.IdType      = FDCAN_STANDARD_ID;
	f.FilterIndex = 0;
	f.FilterType  = FDCAN_FILTER_MASK;          // classic mask
	f.FilterConfig= FDCAN_FILTER_TO_RXFIFO0;
	f.FilterID1   = 0x000;                      // ID
	f.FilterID2   = 0x000;                      // mask=0 → accept all

    // Configure the CAN filter with the defined settings
    // return the status of the filter configuration
	if (HAL_FDCAN_ConfigFilter(hcan_ptr, &f) != HAL_OK) {
		Error_Handler();
	}


    // Enable RX FIFO0 “new message” notification (interrupt mode)
	if (HAL_FDCAN_ActivateNotification(hcan_ptr, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

	return HAL_OK;

}


/**
 * @brief Sends a CAN message.
 * @param hcan Pointer to the CAN handle structure.
 * @param msg Pointer to the CAN message to send.
 * @return HAL status code.
 */
HAL_StatusTypeDef CAN_Comm_Send(FDCAN_HandleTypeDef *hcan, const CAN_Message_t *msg)
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.Identifier = msg->id;                   // Set the standard ID of the message
    txHeader.IdType = FDCAN_STANDARD_ID;                  // Set the identifier type to standard
    //txHeader.TxFrameType = FDCAN_RTR_DATA;                // Set the remote transmission request to data frame
    txHeader.DataLength = msg->dlc;                    // Set the data length code (DLC) (see message structure for more info)
    //txHeader.TransmitGlobalTime = DISABLE;      // Disable global time for transmission

    // Add the message to the transmit mailbox (will queue up to 3 messages for sending before returning an error)
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &txHeader, (uint8_t*)msg->data, &txMailbox);
}

/**
 * @brief Callback function for CAN RX FIFO0 message pending.
 * This function is called automatically on RX interrupt.
 * @param hcan Pointer to the CAN handle structure.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
    // validate that the can handler is the same as the one initialized
    // this prevents processing messages from other CAN interfaces
    if (hcan != hcan_ptr) return;

    // Create a hal structure to hold the received message
    FDCAN_RxHeaderTypeDef rxHeader;
    // Create a message structure to hold the received data
    CAN_Message_t msg;

    // Get the received message from the CAN RX FIFO
    // If the message cannot be retrieved, return without processing
    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rxHeader, msg.data) != HAL_OK) { return; }

    // Check if the received message is a standard ID message
    // If it is, populate the message structure and call the user-defined callback
    if (rxHeader.IdType == FDCAN_STANDARD_ID) {
        // Populate the message structure with the received data
        msg.id = rxHeader.Identifier;
        msg.dlc = rxHeader.DataLength;

        // route the packet to the appropriate user defined callback function

    }
}

/**
 * @brief Registers a callback for received CAN messages.
 * @param callback Pointer to the callback function to register.
 * @return HAL status code.
 */
/*
HAL_StatusTypeDef CAN_Comm_RegisterRxCallback(CAN_Comm_RxCallback_t callback)
{
    // Check if there is room for a new callback
    if (rx_callback_count < MAX_RX_CALLBACKS) {
        rx_callbacks[rx_callback_count++] = callback;
        return HAL_OK;
    }
    return HAL_ERROR;
}
*/
