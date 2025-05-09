#include "tb9051FTG.h"
#include <stdlib.h>
#include "main.h"

#define TIMER_FREQUENCY 5000
#define PWM_RESOLUTION 10
#define PWM_FREQUENCY 100

extern ADC_HandleTypeDef hadc1;

//note that we are short of pwm pins and initially this is architected in software pwm.
//since we are current waveform monitoring we should change to hardware pwm once we eliminate the encoder to reduce noise ripple.

//another important note, don't ever switch between forward and reverse without first shorting the brakes.  This may create
//voltages high enough to destroy the chip

//call at a regular interval.  Make sure to set the frequency define is updated to match
void updateMotor(struct DC_Motor* m)
{
	//update the measured motor current
	HAL_ADC_ConfigChannel(&hadc1, &m->sConfig);
	HAL_ADC_Start(&hadc1); // start the adc
	volatile HAL_StatusTypeDef temp = HAL_ADC_PollForConversion(&hadc1, 100);
	if (HAL_OK == temp) // poll for conversion
	{
		m->current_adc = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	//convert the value into milliamps for logging
	m->current = m->current*0.99 + (m->currentScale * m->current_adc)*0.01;

	//update the duty cycle based upon a feedforward and feedback term
	int16_t error = m->torqueSetpoint - m->current;
	//update the integral error
	m->integralError = m->integralError + error*0.004f;
	//cap the integral error
	if (m->integralError > 30) {m->integralError = 30;}
	if (m->integralError < 0) {m->integralError = 0;}
	m->currentDutyCycle = (error*0.008) + m->integralError;

	//cap the dutycycle
	if (m->currentDutyCycle > 30) {m->currentDutyCycle = 30;}
	if (m->currentDutyCycle < 0) {m->currentDutyCycle = 0;}

	//convert the duty cycle from percentage to timer units
	// since we start with the device on we subtract the duty cycle from 100 percent
	int16_t dc = 100 - abs(m->currentDutyCycle);
	//calculate when on period of the duty cycle should kick off
	m->on_cycle = dc * 0.01 * PWM_RESOLUTION;

	//check if we should move to the on time of the duty cycle
	if (m->cycle_index == 0)
	{
		//initiate the wave form in off (shorted), this will guarantee we always short the leads before switching directions.
		HAL_GPIO_WritePin(m->PWM_A_Port, m->PWM_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m->PWM_B_Port, m->PWM_B_Pin, GPIO_PIN_RESET);
	}
	else if (m->cycle_index >= m->on_cycle)
	{
		//commutate using the B pin constant and A pin pwm
		if (m->is_forward == true)
		{
			//set pwm A high and pwm B low.  This will initiate the on time of the duty cycle.
			HAL_GPIO_WritePin(m->PWM_A_Port, m->PWM_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(m->PWM_B_Port, m->PWM_B_Pin, GPIO_PIN_RESET);
		}
		//in reverse commutate with B pin pwm and A pin constant.
		else
		{
			//set pwm A low and pwm B high.  This will initiate the on time of the duty cycle.
			HAL_GPIO_WritePin(m->PWM_A_Port, m->PWM_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(m->PWM_B_Port, m->PWM_B_Pin, GPIO_PIN_SET);
		}
	}

	//increment the pwm cycle counter
	m->cycle_index++;

	//restart the counter once it gets to the end of its cycle
	if (m->cycle_index >= PWM_RESOLUTION) {m->cycle_index = 0;}
}

//set the duty cycle percentage between negative -100 to 100
void setDutyCycle(struct DC_Motor* m, int16_t dc)
{
	//set the directionality based upon duty cycle sign
	(dc < 0) ? (m->is_forward = false) : (m->is_forward = true);

	//set the desired duty cycle to reach
	m->dutyCycle = dc;
}

//sets the current setpoint in units of adc current
void setTorque(struct DC_Motor* m, int32_t t)
{
	//set the directionality based upon duty cycle sign
	(t < 0) ? (m->is_forward = false) : (m->is_forward = true);

	//pass off the setpoint
	m->torqueSetpoint = (uint16_t)abs(t);
}

//call this to update the duty cycle based on ramp filtering
//must be called faster then the ramp time
void updateDutyCycleRamp(struct DC_Motor* m)
{
	//check if its time to increment
	if (HAL_GetTick() - m->lastDCChangeTime > m->ramp_time)
	{
		//update the last time that the duty cycle was changed
		m->lastDCChangeTime = HAL_GetTick();

		//if the current duty cycle is smaller then setpoint then ramp up
		if (m->currentDutyCycle < m->dutyCycle)
		{
			m->currentDutyCycle++;
		}
		else if(m->currentDutyCycle > m->dutyCycle)
		{
			m->currentDutyCycle--;
		}

		// since we start with the device on we subtract the duty cycle from 100 percent
		int16_t dc = 100 - abs(m->currentDutyCycle);
		//calculate when on period of the duty cycle should kick off
		uint32_t temp_on_cycle = dc * 0.01 * PWM_RESOLUTION;

		//reset the cycle index to zero to have a fresh pwm counter
		//only do this if its a different duty cycle
		if (m->on_cycle != temp_on_cycle) {m->cycle_index = 0;}

		//update the on cycle
		m->on_cycle = temp_on_cycle;
	}
}

//turns on hbridge of motor driver
void enableDriver(struct DC_Motor* m)
{
	HAL_GPIO_WritePin(m->PWM_A_Port, m->PWM_A_Pin, GPIO_PIN_SET);
}

//turns off hbridge of motor driver
void disableDriver(struct DC_Motor* m)
{
	HAL_GPIO_WritePin(m->PWM_A_Port, m->PWM_A_Pin, GPIO_PIN_RESET);
}
