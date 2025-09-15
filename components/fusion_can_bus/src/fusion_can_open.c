#include "fusion_can_open.h"

HAL_StatusTypeDef fusion_can_open_init(FDCAN_HandleTypeDef *hcan)
{
	if (fusion_can_bus_init(hcan) != HAL_OK) {
		return HAL_ERROR;
	}
	else {
		return HAL_OK;
	}
}
