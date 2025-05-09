#ifndef __MOTION_COMMANDS_PROCESSOR_H
#define __MOTION_COMMANDS_PROCESSOR_H

#include <fusion_axis.h>
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

//types of motion commands that can be executed by the motion controller
typedef enum {
	RAPID_MOVE_ABS = 0x00,
	RAPID_MOVE_DELTA = 0x01,
	LINEAR_MOVE_ABS = 0x02, 					//adds a linear absolute move to the back of the buffer
	LINEAR_MOVE_DELTA = 0x03,					//adds a linear delta move to the back of the buffer
	VELOCITY_MOVE = 0x04,						//adds a a velocity move to the back of the buffer
	DWELL = 0x05,								//dwells for a specified period of time in milliseconds
	SET_LINEAR_SPEED = 0x06,					//sets the linear feed for the tool head to move at
	START_MOTION = 0x07,						//starts processing the motion buffer commands on a fifo basis
	STOP_MOTION = 0x08,							//stops process the motion buffer commands.  Will finish last issued command.
	HARD_STOP_MOTION = 0x09,					//stops processing the motion buffer commands and immediately stops motion with ramp down
	CLEAR_MOTION_BUFFER = 0x0A,					//removes all commands in the buffer
	SET_LINEAR_ACCELERATION = 0x0B,				//sets the acceleration to use for linear moves
	ACTUAL_POSITION = 0x0C,						//returns the x y z positions as reported by the encoder in units of microns
	ACUTAL_VELOCITY = 0x0D,						//returns the z y z velocities as reported by the encoder in units of microns per second
	BUFFER_LENGTH = 0x0E,						//returns the remaining length of the motion buffer
	COMMAND_SYNC_VALUE = 0x0F,					//increments with every motion command it has completed
	STATUS_REG = 0x10,							//returns the status of the controller | bit 0 (is ) | bit 1
	X_POS_MOVE = 0x11,							//updates the x position goal
	Y_POS_MOVE = 0x12,							//updates the y position goal
	Z_POS_MOVE = 0x13,							//updates the z position goal
	C_POS_MOVE = 0x14,							//updates the c position goal
	X_VEL_MOVE = 0x15,							//updates the x velocity goal
	Y_VEL_MOVE = 0x16,							//updates the y velocity goal
	Z_VEL_MOVE = 0x17,							//updates the z velocity goal
	C_VEL_MOVE = 0x18							//updates the c velocity goal

} motion_command_id_Type;

//types of motion control modes that we can be in
typedef enum {
	INSTANT_POS_MODE = 0x00,
	INSTANT_VEL_MODE = 0x01,
	BUFFERED_POS_MODE = 0x02,
	BUFFERED_VEL_MODE = 0x03
} motion_control_Type;

//motion commands to be inserted into the buffer
typedef struct {
	motion_command_id_Type type;
	int32_t value1;
	int32_t value2;
	int32_t value3;
} motion_command_t;

void motionControllerInit(void);
void processMotionBuffer(void);
void startMotion(void);
void stopMotion(void);
void updateMotionTelemetry(void);
void addCommandToMotionBuffer(motion_command_t);
uint16_t getBufferLength(void);
bool getMotionComplete(void);
uint8_t getMotionStatus(void);
void startXHoming(void);
void startYHoming(void);
void startZHoming(void);
void stopXHoming(void);
void stopYHoming(void);
void clearBuffer(void);

#endif
