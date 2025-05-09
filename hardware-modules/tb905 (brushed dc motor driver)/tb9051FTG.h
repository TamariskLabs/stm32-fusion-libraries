#ifndef __TB9051FTG_H
#define __TB9051FTG_H

#include "main.h"
#include <stdbool.h>
#include "global.h"

struct DC_Motor {
  GPIO_TypeDef* PWM_A_Port;
	uint16_t	PWM_A_Pin;
  GPIO_TypeDef* PWM_B_Port;
	uint16_t PWM_B_Pin;
  GPIO_TypeDef* EN_Port;
	uint16_t EN_Pin;
  float currentScale;
  float current;
  int16_t dutyCycle;
	bool is_forward;
	uint32_t on_cycle;
	uint32_t cycle_index;
	uint16_t current_adc;
	bool reverseEncoder;
	uint32_t ramp_time;				//milliseconds per duty cycle increase
	int16_t currentDutyCycle;
	uint32_t lastDCChangeTime;
	uint16_t torqueSetpoint;
	float integralError;
	ADC_ChannelConfTypeDef sConfig;
};

void updateMotor(struct DC_Motor*);
float getCurrent(struct DC_Motor*);
void setDutyCycle(struct DC_Motor*, int16_t);
void setCurrentLimit(struct DC_Motor*, float);
void enableDriver(struct DC_Motor*);
void disableDriver(struct DC_Motor*);
void updateDutyCycleRamp(struct DC_Motor* m);
void setTorque(struct DC_Motor* m, int32_t t);

#endif
