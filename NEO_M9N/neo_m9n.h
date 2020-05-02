#ifndef __TAMARISK_NEO_M9N_H
#define __TAMARISK_NEO_M9N_H

//update this include to reflect the MCU series
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

bool process_gps_messages(void);
bool configure_gps(void);

#endif
