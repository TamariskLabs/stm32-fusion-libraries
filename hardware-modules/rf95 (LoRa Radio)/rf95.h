#ifndef __RF_95_H
#define __RF_95_H

#include "stm32l4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

void init_rf_95(void);
void set_rf_95_mode_rx(void);
void set_rf_95_mode_tx(void);
void set_rf_95_mode_standby(void);
void set_rf_95_sleep(void);
void get_rf_95_rx_data(void);
void send_rf_95_data(uint8_t*, uint8_t);
void get_tx_buffer(void);
void rf_parser(void);
void set_rf_95_tx_power(uint8_t);

#endif
