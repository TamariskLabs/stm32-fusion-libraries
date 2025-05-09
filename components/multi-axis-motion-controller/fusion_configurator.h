#ifndef FUSION_CONFIGURATOR_H_
#define FUSION_CONFIGURATOR_H_

#include <fusion_gpio.h>
#include "TMC4361A.h"
#include <stdlib.h>

// Sets up the parameters specific to this fusion hardware device
#define DEVICE_MODEL "DIGITOOL MOTION CONTROLLER"

// gpio pin functions
typedef enum {
	RELAY_OUTPUT = 0x02,
	X_HOME = 0x03,
	Y_HOME = 0x04,
	Z_HOME = 0x05,
	A_HOME = 0x06,
	B_HOME = 0x07,
	C_HOME = 0x08,
	X_END_LIMIT
} gpio_function_Type;

void fusionInit(void);
void fusionService(void);

#endif
