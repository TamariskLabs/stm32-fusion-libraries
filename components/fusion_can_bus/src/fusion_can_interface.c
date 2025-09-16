#include "fusion_can_interface.h"
#include <string.h>

// ------------------------- MODULE MACROS ---------------------
#define MAX_RX_CALLBACKS 15

// ------------------ MODULE STATIC VARIABLES ------------------

// Pointer to the CAN handle structure
static FDCAN_HandleTypeDef *hcan_ptr;

// list of callbacks for routing received messages to
static CAN_Comm_RxCallback_t rx_callbacks[MAX_RX_CALLBACKS] = {NULL};
// list of message id's that correspond with each callback
static uint32_t rx_callback_id_list[MAX_RX_CALLBACKS] = {0};
// list of bit masks to apply to the provided id's when matching with
// received message id's. This is useful when the a portion of the id
// is used for function codes like in CANopen.
static uint32_t rx_callback_id_mask_list[i] = {0};
// keeps track of the number of registered callback functions
static uint8_t rx_callback_count = 0;

// ------------------- MODULE STATIC FUNCTIONS -----------------



// ------------------ FUNCTION DEFINITIONS ---------------------

/**
 * @brief Initializes the CAN communication interface.
 * @param hcan Pointer to the CAN handle structure.
 * @return HAL status code.
 */
HAL_StatusTypeDef fusion_can_bus_init(FDCAN_HandleTypeDef *hcan)
{
    // Store the CAN handle pointer for later use
    hcan_ptr = hcan;

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

	// Start the CAN peripheral and validate it started successfully
	if (HAL_FDCAN_Start(hcan_ptr) != HAL_OK) {
		return HAL_ERROR;
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
HAL_StatusTypeDef fusion_can_bus_send(FDCAN_HandleTypeDef *hcan, const CAN_Message_t *msg)
{
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = msg->id;						// Set the standard ID of the message
	txHeader.IdType = FDCAN_STANDARD_ID;				// Set the identifier type to standard
	txHeader.TxFrameType = FDCAN_DATA_FRAME;			// Set the data frame format to fd can
	txHeader.DataLength = FDCAN_DLC_BYTES_8;			// Set the length of the format to 8 bytes
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;    // Set the Error Status Interrupt to active
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;				// Set the Bit Rate Switching to off
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;				// Use a classic CAN Frame format
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// Turn off the TX interrupt events
	txHeader.MessageMarker = 0;

    // check that the out-bound mailbox's are not full
    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) > 0) {
    	// Add the message to the transmit mailbox (will queue up to 3 messages for sending before returning an error)
    	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &txHeader, (uint8_t*)msg->data) != HAL_OK) {
    		return HAL_ERROR;
    	}
    	else {
    		return HAL_OK;
    	}
    }
}


/**
 * @brief Callback function for CAN RX FIFO0 message pending.
 * This function is called automatically on RX interrupt.
 * @param hcan Pointer to the CAN handle structure.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{

    // Read the interrupt status register to determine the cause of the interrupt
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;

    // Create a hal structure to hold the received message
    FDCAN_RxHeaderTypeDef rxHeader;
    // Create a message structure to hold the received data
    CAN_Message_t msg;

    // Get the received message from the CAN RX FIFO
    // If the message cannot be retrieved, return without processing
    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rxHeader, msg.data) != HAL_OK) {
    	Error_Handler();
    }

    // Check if the received message is a standard ID message
    // If it is, populate the message structure and call the user-defined callback
    if (rxHeader.IdType == FDCAN_STANDARD_ID) {
        // Populate the message structure with the received data
        msg.id = rxHeader.Identifier;
        msg.dlc = rxHeader.DataLength;

        // iterate through each of the registered callbacks to route the message to the appropriate handler
		for (uint8_t i = 0; i < rx_callback_count; i++) {
			// check if the received message id matches any of the registered callback ids
			if ( (rx_callback_id_list[i]) == msg.id) {
				if (rx_callbacks[i] != NULL) {
					// Call the callback function with the message
					rx_callbacks[i](&msg);
				}
			}
		}
    }
}

/**
 * @brief Registers a callback for received CAN messages.
 * @param callback Pointer to the callback function to register.
 * @param msg_id Message ID to filter for the callback.
 * @param mask Bit mask to apply to the message ID for filtering.
 * @return HAL status code.
 */

HAL_StatusTypeDef fusion_can_bus_register_rx_callback(CAN_Comm_RxCallback_t callback, uint32_t msg_id)
{
    // Check if there is room for a new callback
    if (rx_callback_count < MAX_RX_CALLBACKS) {

    	// Add the callback to the list
        rx_callbacks[rx_callback_count] = callback;
        // add the corresponding message id to the list
        rx_callback_id_list[rx_callback_count] = msg_id;

        // increment the callback counter
        rx_callback_count++;

        return HAL_OK;
    }
    return HAL_ERROR;
}
