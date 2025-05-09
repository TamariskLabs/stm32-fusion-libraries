#include <rtt_logging.h>
#include "fusion_gpio.h"
#include "main.h"
#include <stdio.h>

// working variables of the fusion gpio module
static fusionGPIO_t *input[INPUT_PIN_COUNT_LIMIT];			// buffer to hold all registered input pins
static fusionGPIO_t *output[OUTPUT_PIN_COUNT_LIMIT];		// buffer to hold all registered output pins
static uint32_t registeredInputCount = 0;			// holds the total number of input pins that have been registered
static uint32_t registeredOutputCount = 0;			// holds the total number of output pins that have been registered
static uint32_t serviceRate = 100;					// frequency to update gpio tasks in Hz
static uint32_t servicePeriod = 10;					// period to update gpio tasks in Ms (calculated in the init from serviceRate)
static uint32_t lastServiceTime = 0;				// holds the last time the gpio tasks were updated

void fusionGPIOInit(void)
{
	// convert service rate to a period in milliseconds
	// this avoids needing to perform the slow divide operation
	// with each service check
	servicePeriod = 1000/serviceRate;
}

void fusionGPIOService(void)
{
	if (HAL_GetTick() - lastServiceTime >= (servicePeriod))
	{
		// update the state of all registered input pins
		for (uint32_t i=0; i < registeredInputCount; i++)
		{
			// update what the previous state was
			input[i]->previousState = input[i]->state;
			// read in the new state
			input[i]->state = (enum pin_state)HAL_GPIO_ReadPin(input[i]->port, input[i]->pin);

			// log an event of the state changed
			if (input[i]->previousState != input[i]->state)
			{
				// check if it was a rising edge
				if (input[i]->state == input[i]->activeState)
				{
					int length = snprintf( NULL, 0, "Input Activated on Channel: %ld \r\n", i);
					char str[length+2];
					snprintf( str, length, "Input Activated on Channel: %ld \r\n", i);
					SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, str);
				}
				else
				{
					int length = snprintf( NULL, 0, "Input Deactivated on Channel: %ld \r\n", i);
					char str[length+2];
					snprintf( str, length, "Input Deactivated on Channel: %ld \r\n", i);
					SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, str);
				}
			}
		}
	}
}

void fusionAddInputPin(fusionGPIO_t *p)
{
	if (registeredInputCount < INPUT_PIN_COUNT_LIMIT)
	{
		// initialize gpio working values
		p->previousState = !p->activeState;
		p->state = !p->activeState;

		// copy in the input pin to the buffer
		input[registeredInputCount] = p;

		// increment the registered pin count
		registeredInputCount++;
	}
	// log an error if we have exceeded out pincount
	else
	{
		SEGGER_RTT_WriteString(RTT_ERROR_CHANNEL, "INPUT PIN COUNT EXCEEDED\r\n");
	}
}

void fusionAddOutputPin(fusionGPIO_t *p)
{
	if (registeredOutputCount >= OUTPUT_PIN_COUNT_LIMIT)
	{
		// increment the registered pin count
		registeredOutputCount++;
		// copy the output pin to the buffer
		output[registeredOutputCount] = p;

	}
	// log an error if we have exceeded out pincount
	else
	{
		SEGGER_RTT_WriteString(RTT_ERROR_CHANNEL, "OUTPUT PIN COUNT EXCEEDED\r\n");
	}
}

// set an output on or off
void fusionGPIOSetOutput(fusionGPIO_t *p, enum pin_state state)
{
	if (state == LOW)
	{
		HAL_GPIO_WritePin(p->port, p->pin, 0);
	}
	else if (state == HIGH)
	{
		HAL_GPIO_WritePin(p->port, p->pin, 1);
	}
	else
	{
		SEGGER_RTT_WriteString(RTT_ERROR_CHANNEL, "INVALID OUTPUT PIN STATE SETTING\r\n");
	}
}

// sets the rate at which the gpio module is serviced in Hz
void fusionGPIOSetServiceRate(uint32_t f)
{
	// check that the requested value has not exceeded the max allowable service rate
	if (f <= MAX_SERVICE_RATE)
	{
		// update the service rate
		serviceRate = f;
		// update the service period
		servicePeriod = 1000/serviceRate;
	}
	// log an error if requested rate is larger than allowable
	// set the service rate to max allowable
	else
	{
		SEGGER_RTT_WriteString(RTT_ERROR_CHANNEL, "MAX GPIO SERVICE RATE REQUESTED IS MORE THAN MAX OF 1000 Hz\r\n");
		SEGGER_RTT_WriteString(RTT_ERROR_CHANNEL, "SETTING SERVICE RATE TO 1000 Hz\r\n");
		serviceRate = MAX_SERVICE_RATE;
	}
}

// returns the current gpio service rate in Hz
uint32_t fusionGetGPIOServiceRate(void)
{
	return serviceRate;
}
