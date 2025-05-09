#include "fusion_axis.h"
#include "fusion_setup.h"
#include "SEGGER_RTT.h"
#include "rtt_logging.h"
#include <stdlib.h>

#define MAX_AXIS_COUNT 9			// the maximum number of axis' that can be created on this device
#define INT32_MAX_LESS_200 2147483447
#define INT32_MIN_PLUS_200 -2147483448
#define DIST_MOVE_AWAY_OPTO 100

static struct tmc4361_t motion_axis_buffer[MAX_AXIS_COUNT];	// holds a reference to each of the axis that have been created						// holds total number of axis' that have been created

// --------------- static function declarations ---------------


// --------------- function definitions -----------------------

struct tmc4361_t* getMotionAxis(uint8_t ax_index)
{
	return &motion_axis_buffer[ax_index];
}

void updateAxis(struct tmc4361_t * ax)
{
	// update the motors motion states
	fetchAxisVelocityActual(ax);
	fetchAxisPositionActual(ax);

	// update the homing states
	if(ax->status == HOMING_REFERENCE_SWITCH)
	{
		// check to see if the home reference signal has tripped
		if (ax->homingReference->state == ax->homingReference->activeState)
		{
			// if the homing type is home reference switch plus index
			if (ax->homingType == REFERENCE_SWITCH_AND_INDEX)
			{
				// stop moving
				setShadowRegisters6Point(ax, ax->accelMax, ax->accelMax, 0, 0, VELOCITY_MODE);
				axisShadowTransfer(ax);

				//configure the encoder to reset the latches on the next N event
				uint32_t reg_val = tmc4361_readRegister(ax, TMC4361_ENCODER_INPUT_CONFIG_REGISTER);
				reg_val |= (1<<3) + (1<<4) + (1<<5) + (1<<9);
				tmc4361_writeRegister(ax, TMC4361_ENCODER_INPUT_CONFIG_REGISTER, reg_val);
				//clear the event register
				tmc4361_clearEvents(ax);

				// start moving at the index velocity
				setShadowRegisters(ax, ax->accelMax, ax->accelMax, ax->homingIndexVelocity*ax->distanceToPulses, 0, VELOCITY_MODE);
				axisShadowTransfer(ax);

				// update the homing state to the next step
				ax->status = HOMING_ENCODER_INDEX;

				SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, "HOME REFERENCE FOUND\r\n");
			}
			else if (ax->homingType == REFERENCE_SWITCH)
			{
				// stop moving
				setShadowRegisters6Point(ax, ax->accelMax, ax->accelMax, 0, 0, VELOCITY_MODE);
				axisShadowTransfer(ax);

				// update the current position to zero
				setAxisPositionActual(ax, ax->homeOffsetPosition);

				// move to zero position
				ax->targetPosition = 0;
				setShadowRegisters6Point(ax, ax->accelMax, ax->accelMax, ax->velMax, 0, POSITIONING_MODE);
				axisShadowTransfer(ax);

				// set the status to normal
				ax->status = NORMAL;

				SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, "HOME REFERENCE FOUND\r\n");
			}

		}
	}
	else if (ax->status == HOMING_ENCODER_INDEX)
	{
		// check if a homing index has been found
		uint32_t reg_val = tmc4361_readRegister(ax, TMC4361_EVENTS_REGISTER);
		if ((reg_val >> N_ACTIVE) & 1)
		{
			ax->status = NORMAL;
			// move to the zero position
			setShadowRegisters6Point(ax, ax->accelMax, ax->accelMax, ax->velMax, 0, POSITIONING_MODE);
			axisShadowTransfer(ax);

			//apply the offset between index and desired home position
			setAxisPositionActual(ax, ax->homeOffsetPosition);

			// move to the new zero position
			ax->targetPosition = 0;
			setShadowRegisters6Point(ax, ax->accelMax, ax->accelMax, ax->velMax, 0, POSITIONING_MODE);
			axisShadowTransfer(ax);

			SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, "Found Encoder Index\r\n");
		}


	}
	else
	{

		// now add opto switch trip check
		if (ax->lowerEndStop->state==ax->lowerEndStop->activeState)
		{
			// stop the axis

			//configure the encoder to reset the latches on the next N event
			uint32_t reg_val = tmc4361_readRegister(ax, TMC4361_ENCODER_INPUT_CONFIG_REGISTER);
			reg_val |= (1<<3) + (1<<4) + (1<<5) + (1<<9);
			tmc4361_writeRegister(ax, TMC4361_ENCODER_INPUT_CONFIG_REGISTER, reg_val);
			//clear the event register
			tmc4361_clearEvents(ax);

			// get the position
			int32_t posActual = getAxisPositionActual(ax);

			// stop the motion now
			tmc4361_setTargetPosition(ax, ax->distanceToPulses * posActual);

			// set the new target position to the current position if its not withint 100um
			if (abs(ax->targetPosition - posActual) > DIST_MOVE_AWAY_OPTO)
			{
				ax->targetPosition = posActual ;
			}

		}
		if (ax->upperEndStop->state==ax->upperEndStop->activeState)
		{
			// stop the axis

			//configure the encoder to reset the latches on the next N event
			uint32_t reg_val = tmc4361_readRegister(ax, TMC4361_ENCODER_INPUT_CONFIG_REGISTER);
			reg_val |= (1<<3) + (1<<4) + (1<<5) + (1<<9);
			tmc4361_writeRegister(ax, TMC4361_ENCODER_INPUT_CONFIG_REGISTER, reg_val);
			//clear the event register
			tmc4361_clearEvents(ax);

			// get the position
			int32_t posActual = getAxisPositionActual(ax);

			// stop the motion now
			tmc4361_setTargetPosition(ax, ax->distanceToPulses * posActual);

			// set the new target position to the current position if its not withint 100um
			if (abs(ax->targetPosition - posActual) > DIST_MOVE_AWAY_OPTO)
			{
				ax->targetPosition = posActual;
			}

		}
	}
}

// move axis to target position in (microns)
void setAxisPositionTarget(struct tmc4361_t * ax, int32_t pos)
{
	tmc4361_setTargetPosition(ax, pos * ax->distanceToPulses);
}

//set speed in microns per second
void setAxisVelocityTarget(struct tmc4361_t * ax, int32_t speed)
{
	tmc4361_setMaxSpeed(ax, speed * ax->distanceToPulses);
}

// set axis ramp acceleration in (microns per second squared)
void setAxisRampAcceleration(struct tmc4361_t *ax, int32_t accel)
{
	tmc4361_setAccelerations(ax, accel * ax->distanceToPulses, accel * ax->distanceToPulses, 0, 0);
}

// returns the last target position that was issued to the motor controller
int32_t getAxisPositionTarget(struct tmc4361_t *ax)
{
	return ax->targetPosition;
}

// returns the last target velocity that was issued to the motor controller
int32_t getAxisVelocityTarget(struct tmc4361_t *ax)
{
	return ax->targetVelocity;
}

// fetch the velocity in (microns per second) from motor controller chip, update local copy and return value
int32_t fetchAxisVelocityActual(struct tmc4361_t* ax)
{
	ax->encoderVelocity = ax->pulsesToDistance * (int32_t)tmc4361_readRegister(ax, TMC4361_ENCODER_VELOCITY_REGISTER);
	ax->openLoopVelocity = ax->pulsesToDistance * (int32_t)tmc4361_readRegister(ax, TMC4361_V_ACTUAL_REGISTER);

	if (ax->useEncoder == true)
		return ax->encoderVelocity;
	else
		return ax->openLoopVelocity;
}

// fetch the position in microns from motor controller chip, update local copy and return value
int32_t fetchAxisPositionActual(struct tmc4361_t* ax)
{
	ax->encoderPosition = ax->pulsesToDistance * (int32_t)tmc4361_readRegister(ax, TMC4361_ENCODER_POSITION_REGISTER);
	ax->openLoopPosition = ax->pulsesToDistance * (int32_t)tmc4361_readRegister(ax, TMC4361_X_ACTUAL_REGISTER);

	if (ax->useEncoder == true)
		return ax->encoderPosition;
	else
		return ax->openLoopPosition;
}

// returns the latest velocity in (microns per second) without updating from the motor controller chip
int32_t getAxisVelocityActual(struct tmc4361_t* ax)
{
	if (ax->useEncoder == true)
		return ax->encoderVelocity;
	else
		return ax->openLoopVelocity;
}

// returns the latest position in microns without updating from the motor controller chip
int32_t getAxisPositionActual(struct tmc4361_t* ax)
{
	if (ax->useEncoder == true)
		return ax->encoderPosition;
	else
		return ax->openLoopPosition;
}

void setAxisVelocityMax(struct tmc4361_t *ax, int32_t v)
{
	ax->velMax = v;
}

void setAxisPositionActual(struct tmc4361_t* ax, int32_t p)
{
	// update the local value of open loop position
	ax->openLoopPosition = p;
	// update the local value of encoder position
	ax->encoderPosition = p;
	// update the copy of the local target position as well
	ax->targetPosition = p;

	int32_t newPosition = p * ax->distanceToPulses;
	tmc4361_setTargetPosition(ax, newPosition);
	tmc4361_writeRegister(ax, TMC4361_ENCODER_POSITION_REGISTER, newPosition);
	tmc4361_writeRegister(ax, TMC4361_X_ACTUAL_REGISTER, newPosition);
}

// checks that axis for an abnormally large errors in position
bool isAxisStalled(struct tmc4361_t* ax)
{
	if (ax->useEncoder == true)
	{
		if (abs((int16_t)(ax->encoderPosition - ax->openLoopPosition)) > ax->stalledErrorTolerance)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

// sets the control gains for the encoder step correction control ssytem
void setAxisPID(struct tmc4361_t * tmc, uint32_t p, uint32_t i, uint32_t d)
{
	tmc4361_writeRegister(tmc, 0x5A, p); //P
	tmc4361_writeRegister(tmc, 0x5B, i); //I
	tmc4361_writeRegister(tmc, 0x5D, d); //D
}

//trigger shadow register load
void axisShadowTransfer(struct tmc4361_t* ax)
{
	HAL_GPIO_WritePin(ax->START_MOTION_port, ax->START_MOTION_pin, 1);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_GPIO_WritePin(ax->START_MOTION_port, ax->START_MOTION_pin, 0);
}


