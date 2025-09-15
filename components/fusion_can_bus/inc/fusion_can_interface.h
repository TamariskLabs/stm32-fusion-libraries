#ifndef FUSION_CAN_INTERFACE_H
#define FUSION_CAN_INTERFACE_H

#include "main.h"
#include <stdint.h>

#define CAN_RX_FIFO CAN_RX_FIFO0   // FIFO0 or FIFO1

typedef struct {
    uint32_t id;        // 11-bit standard identifier
    uint8_t dlc;        // Data length (0–8)
    uint8_t data[8];    // Payload
} CAN_Message_t;

// defines a callback type for received CAN messages
typedef void (*CAN_Comm_RxCallback_t)(const CAN_Message_t *msg);

// Initializes CAN and starts reception
HAL_StatusTypeDef fusion_can_bus_init(FDCAN_HandleTypeDef *hcan);

// Sends a CAN message
HAL_StatusTypeDef fusion_can_bus_send(FDCAN_HandleTypeDef *hcan, const CAN_Message_t *msg);

// Registers a callback for received messages
HAL_StatusTypeDef fusion_can_bus_register_rx_callback(CAN_Comm_RxCallback_t callback, uint32_t msg_id, uint32_t mask);

#endif
