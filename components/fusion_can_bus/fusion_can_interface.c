#include "fusion_can_interface.h"
#include <string.h>

// ------------------------- MODULE MACROS ---------------------
#define CAN_COMM_MAX_CALLBACKS 10

// ------------------ MODULE STATIC VARIABLES ------------------

// Pointer to the CAN handle structure
static CAN_HandleTypeDef *hcan_ptr;

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
HAL_StatusTypeDef CAN_Comm_Init(CAN_HandleTypeDef *hcan)
{
    // Store the CAN handle pointer for later use
    hcan_ptr = hcan;

    // Start the CAN peripheral and validate it started successfully
    if (HAL_CAN_Start(hcan_ptr) != HAL_OK)
        return HAL_ERROR;

    // Activate the notification for RX FIFO0 message pending
    // This allows the HAL to call the callback when a message is received
    if (HAL_CAN_ActivateNotification(hcan_ptr, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return HAL_ERROR;

    // Optional filter to accept all messages
    CAN_FilterTypeDef filter = {
        .FilterActivation = ENABLE,
        .FilterBank = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterIdHigh = 0,
        .FilterIdLow = 0,
        .FilterMaskIdHigh = 0,
        .FilterMaskIdLow = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT
    };

    // Configure the CAN filter with the defined settings
    // return the status of the filter configuration
    return HAL_CAN_ConfigFilter(hcan_ptr, &filter);
}


/**
 * @brief Sends a CAN message.
 * @param hcan Pointer to the CAN handle structure.
 * @param msg Pointer to the CAN message to send.
 * @return HAL status code.
 */
HAL_StatusTypeDef CAN_Comm_Send(CAN_HandleTypeDef *hcan, const CAN_Message_t *msg)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.StdId = msg->id;                   // Set the standard ID of the message
    txHeader.IDE = CAN_ID_STD;                  // Set the identifier type to standard
    txHeader.RTR = CAN_RTR_DATA;                // Set the remote transmission request to data frame
    txHeader.DLC = msg->dlc;                    // Set the data length code (DLC) (see message structure for more info)
    txHeader.TransmitGlobalTime = DISABLE;      // Disable global time for transmission

    // Add the message to the transmit mailbox (will queue up to 3 messages for sending before returning an error)
    return HAL_CAN_AddTxMessage(hcan, &txHeader, (uint8_t*)msg->data, &txMailbox);
}

/**
 * @brief Callback function for CAN RX FIFO0 message pending.
 * This function is called automatically on RX interrupt.
 * @param hcan Pointer to the CAN handle structure.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // validate that the can handler is the same as the one initialized
    // this prevents processing messages from other CAN interfaces
    if (hcan != hcan_ptr) return;

    // Create a hal structure to hold the received message
    CAN_RxHeaderTypeDef rxHeader;
    // Create a message structure to hold the received data
    CAN_Message_t msg;

    // Get the received message from the CAN RX FIFO
    // If the message cannot be retrieved, return without processing
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO, &rxHeader, msg.data) != HAL_OK)
        return;

    // Check if the received message is a standard ID message
    // If it is, populate the message structure and call the user-defined callback
    if (rxHeader.IDE == CAN_ID_STD) {
        // Populate the message structure with the received data
        msg.id = rxHeader.StdId;
        msg.dlc = rxHeader.DLC;
        // call the user-defined callback to process the received message
        CAN_Comm_RxCallback(&msg);
    }
}

/**
 * @brief Registers a callback for received CAN messages.
 * @param callback Pointer to the callback function to register.
 * @return HAL status code.
 */
HAL_StatusTypeDef CAN_Comm_RegisterRxCallback(CAN_Comm_RxCallback_t callback)
{
    // Check if there is room for a new callback
    if (rx_callback_count < MAX_RX_CALLBACKS) {
        rx_callbacks[rx_callback_count++] = callback;
        return HAL_OK;
    }
    return HAL_ERROR;
}