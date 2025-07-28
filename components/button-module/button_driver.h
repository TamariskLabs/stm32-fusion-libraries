#ifndef __BUTTON_DRIVER_H
#define __BUTTON_DRIVER_H

#include "stm32l4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

struct Button {
  GPIO_TypeDef* button_port;
  uint16_t button_pin;
  uint32_t last_falling_edge_time;
  uint32_t debounce_time;
  uint8_t last_button_state;
  float filter_weight;
  float filtered_value;
};

bool update_button_falling(struct Button*);
bool update_button_rising(struct Button*);
bool update_button_rise_fall(struct Button*);

#endif
