#ifndef FUSION_GPIO_H_
#define FUSION_GPIO_H_

#include "main.h"
#include <stdbool.h>

// pre-compiled module configurations
#define INPUT_PIN_COUNT_LIMIT	 32		// size of the input pin buffer.
#define OUTPUT_PIN_COUNT_LIMIT	 32		// size of the output pin buffer.
#define MAX_SERVICE_RATE		 1000	// max allowable service frequency (checking logic operates in mS so this should not exceed 1kHz).

// different states that pins (including special pins) can be in.
enum pin_state{
	LOW = 0x00,
	HIGH = 0x01,
	RISING = 0x02,
	FALLING = 0x03,
};

// fusion gpio pin structure
typedef struct  {
	bool riseCallbackEnabled;				// If this is true the controller will return an async message with a rising edge.
	bool fallCallbakcEnabled; 				// If this is true the controller will return an async message with a falling edge.
	uint16_t pin;							// The hardware pin number.
	GPIO_TypeDef * port;					// The hardware port of the pin.
	enum pin_state activeState;				// sets if a high or low signal should be considered to be the pins active state
	enum pin_state state;					// Holds the most recent state of pin.
	enum pin_state previousState;			// holds the previous state of the pin. Used for software based checks of state changes.
} fusionGPIO_t;


void fusionGPIOService(void);
void fusionAddInputPin(fusionGPIO_t *p);
void fusionAddOutputPin(fusionGPIO_t *p);
void fusionGPIORegisterCallback(void);
enum pin_state fusionGPIOGetInput(fusionGPIO_t *p);
void fusionGPIOSetOutput(fusionGPIO_t *p, enum pin_state);
void fusionGPIOSetServiceRate(uint32_t);
uint32_t fusioGPIOGetServiceRate(void);

#endif
