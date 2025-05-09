#ifndef __MOSFET_HAPTIC_H__
#define __MOSFET_HAPTIC_H__

#include "main.h"

typedef struct {
	GPIO_TypeDef *GPIOPort;
	uint16_t GPIOPin;
} Haptic_TypeDef;

void haptic_update(Haptic_TypeDef *haptic_handler);
void haptic_set_level(Haptic_TypeDef *haptic_handler, uint8_t);
void haptic_set_pattern(Haptic_TypeDef *haptic_handler, uint8_t, uint32_t, uint32_t);

#endif
