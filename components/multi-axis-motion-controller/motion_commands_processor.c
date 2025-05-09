#include "fusion_configurator.h"
#include "rtt_logging.h"
#include "motion_commands_processor.h"
#include "fusion_setup.h"
#include "jetson.h"
#include <stdlib.h>

#define MOTION_BUFFER_LEN 1024

//stores the commands to be processed
static motion_command_t command_buffer[MOTION_BUFFER_LEN];

//holds the current command being processed
static motion_command_t current_command;

// hold a local reference to the motion axis
static struct tmc4361_t *xAxis;
static struct tmc4361_t *yAxis;
static struct tmc4361_t *zAxis;
static struct tmc4361_t *cAxis;

//define a timer handler for pwm generation of the tmc clocks
extern TIM_HandleTypeDef htim1;

//holds the current working motion parameters
static int32_t toolSpeed = 0;			//desired toolspeed in microns per second (there are 1000 microns in a mm)
static int32_t toolAccel = 0;			//desired tool acceleration in microns per second sq
static int32_t xSpeed = 0;				//required speed for the axis to move in pulses per second
static int32_t ySpeed = 0;				//required speed for the axis to move in pulses per second
static int32_t zSpeed = 0;				//required speed for the axis to move in pulses per second
static int32_t xAccel = 0;				//required acceleration for the axis to move in pulses per second sq
static int32_t yAccel = 0;				//required acceleration for the axis to move in pulses per second sq
static int32_t zAccel = 0;				//required acceleration for the axis to move in pulses per second sq

//holds what control mode the controller is operating in
static uint8_t controlMode = INSTANT_POS_MODE;
static uint8_t motionStatus = 0;

//states used to process the command buffer
static uint16_t motion_buffer_length = 0;				//remaining commands in the buffer to be processed
static uint16_t motion_buffer_index = 0;				//the index of the next motion command to process
static bool start_motion = false;								//dictates if we should process commands from the buffer
static bool motion_command_complete = true;			//true if its time to process another motion command from the buffer
static bool x_pos_reached = true;
static bool y_pos_reached = true;
static bool z_pos_reached = true;
static bool c_pos_reached = true;

//used for updating telemetry data from the driver chips
uint32_t lastTelemetryUpdateTime = 0;
uint32_t telemetryUpdatePeriod = 14;

// --------------- static function declarations ------
void shadowTransferAll(void);
void shadowTransferGantry(void);


// --------------- static function definitions -------

void shadowTransferAll(void)
{
	HAL_GPIO_WritePin(xAxis->START_MOTION_port, xAxis->START_MOTION_pin, 1);
	HAL_GPIO_WritePin(yAxis->START_MOTION_port, yAxis->START_MOTION_pin, 1);
	HAL_GPIO_WritePin(zAxis->START_MOTION_port, zAxis->START_MOTION_pin, 1);
	HAL_GPIO_WritePin(cAxis->START_MOTION_port, cAxis->START_MOTION_pin, 1);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_GPIO_WritePin(xAxis->START_MOTION_port, xAxis->START_MOTION_pin, 0);
	HAL_GPIO_WritePin(yAxis->START_MOTION_port, yAxis->START_MOTION_pin, 0);
	HAL_GPIO_WritePin(zAxis->START_MOTION_port, zAxis->START_MOTION_pin, 0);
	HAL_GPIO_WritePin(cAxis->START_MOTION_port, cAxis->START_MOTION_pin, 0);
}

void shadowTransferGantry(void)
{
	HAL_GPIO_WritePin(xAxis->START_MOTION_port, xAxis->START_MOTION_pin, 1);
	HAL_GPIO_WritePin(yAxis->START_MOTION_port, yAxis->START_MOTION_pin, 1);
	HAL_GPIO_WritePin(zAxis->START_MOTION_port, zAxis->START_MOTION_pin, 1);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_GPIO_WritePin(xAxis->START_MOTION_port, xAxis->START_MOTION_pin, 0);
	HAL_GPIO_WritePin(yAxis->START_MOTION_port, yAxis->START_MOTION_pin, 0);
	HAL_GPIO_WritePin(zAxis->START_MOTION_port, zAxis->START_MOTION_pin, 0);
}


// --------------- function definitions --------------

void motionControllerInit(void)
{
	// since The TMC needs an external CLK, we generate a 9MHz square wave using TIM1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	TIM1->CCR1 = 1;

	// map the local axis pointers to their global pointer from the motion axis buffer
	xAxis = getMotionAxis(X_AXIS_INDEX);
	yAxis = getMotionAxis(Y_AXIS_INDEX);
	zAxis = getMotionAxis(Z_AXIS_INDEX);
	cAxis = getMotionAxis(C_AXIS_INDEX);

	// initialize the status to be unreferenced for gantry axis'
	xAxis->status = UNREFERENCED;
	yAxis->status = UNREFERENCED;
	zAxis->status = UNREFERENCED;
	// initialize the conveyer
	cAxis->status = NORMAL;

	// de-assert SS pins
	HAL_GPIO_WritePin(xAxis->SS_port, xAxis->SS_pin, 1);
	HAL_GPIO_WritePin(yAxis->SS_port, yAxis->SS_pin, 1);
	HAL_GPIO_WritePin(zAxis->SS_port, zAxis->SS_pin, 1);
	HAL_GPIO_WritePin(cAxis->SS_port, cAxis->SS_pin, 1);

	// perform a reset of all axis
	HAL_GPIO_WritePin(TMC_RESET_GPIO_Port, TMC_RESET_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(TMC_RESET_GPIO_Port, TMC_RESET_Pin, 1);

	// configure driver chip clock settings
	tmc4361_begin(xAxis);
	tmc4361_begin(yAxis);
	tmc4361_begin(zAxis);
	tmc4361_begin(cAxis);

	// set motion profile to position trapazoidal mode
	tmc4361_setRampMode(xAxis, POSITIONING_MODE, TRAPEZOIDAL_RAMP);
	tmc4361_setRampMode(yAxis, POSITIONING_MODE, TRAPEZOIDAL_RAMP);
	tmc4361_setRampMode(zAxis, POSITIONING_MODE, TRAPEZOIDAL_RAMP);
	tmc4361_setRampMode(cAxis, POSITIONING_MODE, TRAPEZOIDAL_RAMP);

	// set encoders to be single ended
	tmc4361_set_encoder_single_ended(xAxis);
	tmc4361_set_encoder_single_ended(yAxis);
	tmc4361_set_encoder_single_ended(zAxis);
	tmc4361_set_encoder_single_ended(cAxis);

	// 256 microsteps per step, 200 steps per revolution
	tmc4361_setStepsPerRev(xAxis, 0x00000C80);
	tmc4361_setStepsPerRev(yAxis, 0x00000C80);
	tmc4361_setStepsPerRev(zAxis, 0x00000C80);
	tmc4361_setStepsPerRev(cAxis, 0x00000C80);

	// encoder pulses per revolution
	// note that the full steps must be set to 256 so encoder may need a fudge factor
	tmc4361_writeRegister(xAxis, TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER, 280000);
	tmc4361_writeRegister(yAxis, TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER, 280000);
	tmc4361_writeRegister(zAxis, TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER, 64000);
	tmc4361_writeRegister(cAxis, TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER, 280000);

	// enable closed loop
	// 1 - open loop, 2 pid, 3 pid with feedforward
	tmc4361_setClosedLoopSetting(xAxis, 3);
	tmc4361_setClosedLoopSetting(yAxis, 3);
	tmc4361_setClosedLoopSetting(zAxis, 3);
	tmc4361_setClosedLoopSetting(cAxis, 0);

	//sets the tolerance for target reached interrupt
	tmc4361_writeRegister(xAxis, 0x52, 0x09);
	tmc4361_writeRegister(yAxis, 0x52, 0x09);
	tmc4361_writeRegister(zAxis, 0x52, 0x09);
	tmc4361_writeRegister(cAxis, 0x52, 0x09);

	//set pid parameters for the axis'
	setAxisPID(xAxis, 3200, 0x00000010, 1);
	setAxisPID(yAxis, 3200, 0x00000010, 1);
	setAxisPID(zAxis, 5, 0x00000020, 0);    //5,20,0
	setAxisPID(cAxis, 0, 0x00000000, 0);

	//zone where PID feedback is floored to zero
	tmc4361_writeRegister(xAxis, 0x5F, 0x00000008);
	tmc4361_writeRegister(yAxis, 0x5F, 0x00000008);
	tmc4361_writeRegister(zAxis, 0x5F, 0x00000008);
	tmc4361_writeRegister(cAxis, 0x5F, 0x00000008);

	//set the max additional velocity that PID can set
	tmc4361_writeRegister(xAxis, 0x5E, 18000);
	tmc4361_writeRegister(yAxis, 0x5E, 18000);
	tmc4361_writeRegister(zAxis, 0x5E, 9000);
	tmc4361_writeRegister(cAxis, 0x5E, 18000);

	//check if the X axis encoder direction should be flipped
#if INVERT_X_ENCODER == true
	tmc4361_invertEncoder(xAxis, 1);
#endif
	//check if the Y axis encoder direction should be flipped
#if INVERT_Y_ENCODER == true
	tmc4361_invertEncoder(yAxis, 1);
#endif
#if INVERT_Z_ENCODER == true
	//check if the Z axis encoder direction should be flipped
	tmc4361_invertEncoder(zAxis, 1);
#endif

	//configure the encoder counts x
	uint32_t mscnt = tmc4361_readRegister(xAxis, TMC4361_MICROSTEP_READ_REGISTER);
	uint32_t xactual = tmc4361_readRegister(xAxis, TMC4361_X_ACTUAL_REGISTER);
	tmc4361_writeRegister(xAxis, TMC4361_X_TARGET_REGISTER, xactual + 384 - (mscnt % 256));
	mscnt = tmc4361_readRegister(xAxis, TMC4361_MICROSTEP_READ_REGISTER);
	tmc4361_writeRegister(xAxis, TMC4361_X_ACTUAL_REGISTER, mscnt);

	//configure the encoder counts y
	mscnt = tmc4361_readRegister(yAxis, TMC4361_MICROSTEP_READ_REGISTER);
	xactual = tmc4361_readRegister(yAxis, TMC4361_X_ACTUAL_REGISTER);
	tmc4361_writeRegister(yAxis, TMC4361_X_TARGET_REGISTER, xactual + 384 - (mscnt % 256));
	mscnt = tmc4361_readRegister(yAxis, TMC4361_MICROSTEP_READ_REGISTER);
	tmc4361_writeRegister(yAxis, TMC4361_X_ACTUAL_REGISTER, mscnt);

	//configure the encoder counts z
	mscnt = tmc4361_readRegister(zAxis, TMC4361_MICROSTEP_READ_REGISTER);
	xactual = tmc4361_readRegister(zAxis, TMC4361_X_ACTUAL_REGISTER);
	tmc4361_writeRegister(zAxis, TMC4361_X_TARGET_REGISTER, xactual + 384 - (mscnt % 256));
	mscnt = tmc4361_readRegister(zAxis, TMC4361_MICROSTEP_READ_REGISTER);
	tmc4361_writeRegister(zAxis, TMC4361_X_ACTUAL_REGISTER, mscnt);

	//configure the encoder counts c
	mscnt = tmc4361_readRegister(cAxis, TMC4361_MICROSTEP_READ_REGISTER);
	xactual = tmc4361_readRegister(cAxis, TMC4361_X_ACTUAL_REGISTER);
	tmc4361_writeRegister(cAxis, TMC4361_X_TARGET_REGISTER, xactual + 384 - (mscnt % 256));
	mscnt = tmc4361_readRegister(cAxis, TMC4361_MICROSTEP_READ_REGISTER);
	tmc4361_writeRegister(cAxis, TMC4361_X_ACTUAL_REGISTER, mscnt);

	//enable shadow registers to use the external start pin
	//set the start condition to be the external start pin
	//set the start delay to be immediate
	//0x20 - external start pin is active
	//0x10 - shadow registers are assigned as ramp registers on internal start
	//0x200 - start pin is active high
	//0x400 - start pin immediately activates shadow register change
	//0x1000 - xtarget is feed with xpipeline
	//0b0010001000110000
	tmc4361_writeRegister(xAxis, TMC4361_START_CONFIG_REGISTER, 0x00001630);
	tmc4361_writeRegister(yAxis, TMC4361_START_CONFIG_REGISTER, 0x00001630);
	tmc4361_writeRegister(zAxis, TMC4361_START_CONFIG_REGISTER, 0x00001630);
	tmc4361_writeRegister(cAxis, TMC4361_START_CONFIG_REGISTER, 0x00001630);
}

void calcMotionParameters(int32_t xStart, int32_t xStop, int32_t yStart, int32_t yStop, int32_t zStart, int32_t zStop)
{
	//calculate the component distances traveled
	float xDistance = (float)(xStop - xStart);
	float yDistance = (float)(yStop - yStart);
	float zDistance = (float)(zStop - zStart);

	//calculate the total distance traveled
	float distanceTotal = sqrt(xDistance*xDistance + yDistance*yDistance + zDistance*zDistance);

	//calculate the unit vectors
	float xUnitVector = xDistance/distanceTotal;
	float yUnitVector = yDistance/distanceTotal;
	float zUnitVector = zDistance/distanceTotal;

	//calculate the required axis speed to move at
	xSpeed = xUnitVector * toolSpeed * xAxis->distanceToPulses;
	ySpeed = yUnitVector * toolSpeed * yAxis->distanceToPulses;
	zSpeed = zUnitVector * toolSpeed * zAxis->distanceToPulses;

	//calculate the required axis accel to move at
	xAccel = xUnitVector * toolAccel * xAxis->distanceToPulses;
	yAccel = yUnitVector * toolAccel * yAxis->distanceToPulses;
	zAccel = zUnitVector * toolAccel * zAxis->distanceToPulses;

	//check if the speed needs to be throttled by a particular axis
	if (xAccel> xAxis->accelMax) {xAccel = xAxis->accelMax;}
	else if (xAccel < -xAxis->accelMax){xAccel = -xAxis->accelMax;}
	if (yAccel > yAxis->accelMax) {yAccel = yAxis->accelMax;}
	else if (yAccel < -yAxis->accelMax){yAccel = -yAxis->accelMax;}
	if (zAccel > zAxis->accelMax) {zAccel = zAxis->accelMax;}
	else if (zAccel < -zAxis->accelMax){zAccel = -zAxis->accelMax;}


	//check if the speed needs to be throttled by a particular axis
	if (xSpeed > xAxis->velMax) {xSpeed = xAxis->velMax;}
	else if (xSpeed < -xAxis->velMax){xSpeed = -xAxis->velMax;}
	if (ySpeed > yAxis->velMax) {ySpeed = yAxis->velMax;}
	else if (ySpeed < -yAxis->velMax){ySpeed = -yAxis->velMax;}
	if (zSpeed > zAxis->velMax) {zSpeed = zAxis->velMax;}
	else if (zSpeed < -zAxis->velMax){zSpeed = -zAxis->velMax;}
}

void processMotionBuffer(void)
{
	// service each of the motor axis tasks
	updateAxis(xAxis);
	updateAxis(yAxis);
	updateAxis(zAxis);
	updateAxis(cAxis);

	// store a local copy of the axis positions
	int32_t xPosActual = getAxisPositionActual(xAxis);
	int32_t yPosActual = getAxisPositionActual(yAxis);
	int32_t zPosActual = getAxisPositionActual(zAxis);

	//make sure that start motion is true
	if (start_motion == false) {

		zBreak(BREAK_ON); //on

		//set the target position to the current position
		tmc4361_setTargetPosition(xAxis, xAxis->distanceToPulses * xPosActual);
		xAxis->targetPosition = getAxisPositionActual(xAxis);
		tmc4361_setTargetPosition(yAxis, yAxis->distanceToPulses * yPosActual);
		yAxis->targetPosition = getAxisPositionActual(yAxis);
		tmc4361_setTargetPosition(zAxis, zAxis->distanceToPulses * zPosActual);
		zAxis->targetPosition = getAxisPositionActual(zAxis);

		setShadowRegisters6Point(xAxis, xAxis->accelMax, xAxis->accelMax, xAxis->velMax, xAxis->targetPosition*xAxis->distanceToPulses, POSITIONING_MODE);
		setShadowRegisters6Point(yAxis, yAxis->accelMax, yAxis->accelMax, yAxis->velMax, yAxis->targetPosition*yAxis->distanceToPulses, POSITIONING_MODE);
		setShadowRegisters6Point(zAxis, zAxis->accelMax, zAxis->accelMax, zAxis->velMax, zAxis->targetPosition*zAxis->distanceToPulses, POSITIONING_MODE);
		setShadowRegisters6Point(cAxis, cAxis->accelMax, cAxis->accelMax, 0, 0, VELOCITY_MODE);
		shadowTransferAll();

		clearBuffer();

		return;
	}

	zBreak(BREAK_OFF); //off



	// run the controller for position commands (both buffered and instant)
	if (controlMode == INSTANT_POS_MODE || controlMode == BUFFERED_POS_MODE)
	{
		//calculate the distance from target
		int32_t xPosDelta = abs(xAxis->targetPosition - xPosActual);
		int32_t yPosDelta = abs(yAxis->targetPosition - yPosActual);
		int32_t zPosDelta = abs(zAxis->targetPosition - zPosActual);

		//check if the axis have reached the intended goal
		if (xPosDelta < 20) {x_pos_reached=true;}
		if (yPosDelta < 20) {y_pos_reached=true;}
		if (zPosDelta < 20) {z_pos_reached=true;}

		//if we are within a certain distance of goal then mark motion complete
		if (x_pos_reached==true && y_pos_reached==true && z_pos_reached==true) {
			if (motion_command_complete != true)
			{
				SEGGER_RTT_WriteString(RTT_TRACE_CHANNEL, "POS TARGET REACHED\r\n");
				motion_command_complete = true;
			}
		}
	}

	//feedback controller if we are in instant pos mode
	if (controlMode == INSTANT_POS_MODE)
	{
		//right now no action needs to be taken
	}
	//feedback controller if we are in instant vel mode
	else if (controlMode == INSTANT_VEL_MODE)
	{
		//right now no action needs to be taken
	}
	//feedback controller if we are in buffered pos mode
	else if (controlMode == BUFFERED_POS_MODE && motion_command_complete == false)
	{
		calcMotionParameters(xPosActual, xAxis->targetPosition, yPosActual, yAxis->targetPosition, zPosActual, zAxis->targetPosition);

		//if its the last command use position mode
		if (motion_buffer_length == 0)
		{
			//set the shadow registers with the new values
			setShadowRegisters6Point(xAxis, xAccel, xAccel, xSpeed, xAxis->targetPosition*xAxis->distanceToPulses, POSITIONING_MODE);
			setShadowRegisters6Point(yAxis, yAccel, yAccel, ySpeed, yAxis->targetPosition*yAxis->distanceToPulses, POSITIONING_MODE);
			setShadowRegisters6Point(zAxis, zAccel, zAccel, zSpeed, zAxis->targetPosition*zAxis->distanceToPulses, POSITIONING_MODE);
			x_pos_reached = true;
			y_pos_reached = true;
			z_pos_reached = true;
		}
		//if this is not the last motion command use velocity mode
		else
		{
			//set the shadow registers with the new values
			setShadowRegisters6Point(xAxis, xAxis->accelMax, xAxis->accelMax, xSpeed, xAxis->targetPosition, VELOCITY_MODE);
			setShadowRegisters6Point(yAxis, yAxis->accelMax, yAxis->accelMax, ySpeed, yAxis->targetPosition, VELOCITY_MODE);
			setShadowRegisters6Point(zAxis, zAxis->accelMax, zAxis->accelMax, zSpeed, zAxis->targetPosition, VELOCITY_MODE);
		}
		//transfer all registers to active at the same time
		shadowTransferGantry();
	}
	else if (controlMode == BUFFERED_VEL_MODE && motion_command_complete == false)
	{
		//right now no action needs to be taken
	}

	//if there is no data in the buffer then dont parse packets
	if (motion_buffer_length == 0) {return;}

	//if the previous command has finished then load the next command
	if (controlMode == INSTANT_POS_MODE || controlMode == INSTANT_VEL_MODE || motion_command_complete == true)
	{
		//process the next motion task
		current_command = command_buffer[motion_buffer_index];

		//execute the command
		if (current_command.type == LINEAR_MOVE_ABS)
		{
			//update motion parameters to send
			xAxis->targetPosition = current_command.value1;
			yAxis->targetPosition = current_command.value2;
			zAxis->targetPosition = current_command.value3;

			calcMotionParameters(xPosActual, xAxis->targetPosition, yPosActual, yAxis->targetPosition, zPosActual, zAxis->targetPosition);

			setShadowRegisters6Point(xAxis, xAxis->accelMax, xAxis->accelMax, xSpeed, xAxis->targetPosition*xAxis->distanceToPulses, POSITIONING_MODE);
			setShadowRegisters6Point(yAxis, yAxis->accelMax, yAxis->accelMax, ySpeed, yAxis->targetPosition*yAxis->distanceToPulses, POSITIONING_MODE);
			setShadowRegisters6Point(zAxis, zAxis->accelMax, zAxis->accelMax, zSpeed, zAxis->targetPosition*zAxis->distanceToPulses, POSITIONING_MODE);
			shadowTransferGantry();

			//set motion to active
			motion_command_complete = false;

			//update the goal reached
			x_pos_reached = false;
			y_pos_reached = false;
			z_pos_reached = false;

			//set the control mode
			controlMode = INSTANT_POS_MODE;
		}
		else if (current_command.type == RAPID_MOVE_ABS)
		{
			//update motion parameters to send
			xAxis->targetPosition = current_command.value1;
			yAxis->targetPosition = current_command.value2;
			zAxis->targetPosition = current_command.value3;

			setShadowRegisters6Point(xAxis, xAxis->accelMax, xAxis->accelMax, xAxis->velMax, xAxis->targetPosition*xAxis->distanceToPulses, POSITIONING_MODE);
			setShadowRegisters6Point(yAxis, yAxis->accelMax, yAxis->accelMax, yAxis->velMax, yAxis->targetPosition*yAxis->distanceToPulses, POSITIONING_MODE);
			setShadowRegisters6Point(zAxis, zAxis->accelMax, zAxis->accelMax, zAxis->velMax, zAxis->targetPosition*zAxis->distanceToPulses, POSITIONING_MODE);
			shadowTransferGantry();

			//set motion to active
			motion_command_complete = false;

			//update the goal reached
			x_pos_reached = false;
			y_pos_reached = false;
			z_pos_reached = false;

			//set the control mode
			controlMode = INSTANT_POS_MODE;
		}
		else if (current_command.type == X_POS_MOVE)
		{
			//update motion parameters to send
			xAxis->targetPosition = current_command.value1;

			setShadowRegisters6Point(xAxis, xAxis->accelMax, xAxis->accelMax, xAxis->velMax, xAxis->targetPosition*xAxis->distanceToPulses, POSITIONING_MODE);
			axisShadowTransfer(xAxis);

			//set motion to active
			motion_command_complete = false;

			//update the goal reached
			x_pos_reached = false;

			//set the control mode
			controlMode = INSTANT_POS_MODE;
		}
		else if (current_command.type == Y_POS_MOVE)
		{
			//update motion parameters to send
			yAxis->targetPosition = current_command.value1;

			setShadowRegisters6Point(yAxis, yAxis->accelMax, yAxis->accelMax, yAxis->velMax, yAxis->targetPosition*yAxis->distanceToPulses, POSITIONING_MODE);
			axisShadowTransfer(yAxis);

			//set motion to active
			motion_command_complete = false;

			//update the goal reached
			y_pos_reached = false;

			//set the control mode
			controlMode = INSTANT_POS_MODE;
		}
		else if (current_command.type == Z_POS_MOVE)
		{
			//update motion parameters to send
			zAxis->targetPosition = current_command.value1;

			setShadowRegisters6Point(zAxis, zAxis->accelMax, zAxis->accelMax, zAxis->velMax, zAxis->targetPosition*zAxis->distanceToPulses, POSITIONING_MODE);
			axisShadowTransfer(zAxis);

			//set motion to active
			motion_command_complete = false;

			//update the goal reached
			z_pos_reached = false;

			//set the control mode
			controlMode = INSTANT_POS_MODE;
		}
		else if (current_command.type == C_POS_MOVE)
		{
			//update motion parameters to send
			cAxis->targetPosition = current_command.value1;

			setShadowRegisters6Point(cAxis, cAxis->accelMax, cAxis->accelMax, cAxis->velMax, cAxis->targetPosition*cAxis->distanceToPulses, POSITIONING_MODE);
			axisShadowTransfer(cAxis);

			//set motion to active
			motion_command_complete = false;

			//update the goal reached
			c_pos_reached = false;
		}
		else if (current_command.type == VELOCITY_MOVE)
		{
			//update our target positions to report back to the host computer
			xAxis->targetVelocity = current_command.value1;
			yAxis->targetVelocity = current_command.value2;
			zAxis->targetVelocity = current_command.value3;

			//set the shadow registers with the new values
			setShadowRegisters6Point(xAxis, xAxis->accelMax, xAxis->accelMax, xAxis->targetVelocity*xAxis->distanceToPulses, 0, VELOCITY_MODE);
			setShadowRegisters6Point(yAxis, yAxis->accelMax, yAxis->accelMax, yAxis->targetVelocity*yAxis->distanceToPulses, 0, VELOCITY_MODE);
			setShadowRegisters6Point(zAxis, zAxis->accelMax, zAxis->accelMax, zAxis->targetVelocity*zAxis->distanceToPulses, 0, VELOCITY_MODE);
			shadowTransferGantry();

			//Immediately set motion to completed
			motion_command_complete = true;
			x_pos_reached = true;
			y_pos_reached = true;
			z_pos_reached = true;

			//set the control mode
			controlMode = INSTANT_VEL_MODE;
		}
		else if (current_command.type == X_VEL_MOVE)
		{
			//update our target positions to report back to the host computer
			xAxis->targetVelocity = current_command.value1;

			//set the shadow registers with the new values
			setShadowRegisters(xAxis, xAxis->accelMax, xAxis->accelMax, xAxis->targetVelocity*xAxis->distanceToPulses, 0, VELOCITY_MODE);
			// transfer only the x axis
			axisShadowTransfer(xAxis);

			//Immediately set motion to completed
			motion_command_complete = true;
			x_pos_reached = true;

			//set the control mode
			controlMode = INSTANT_VEL_MODE;
		}
		else if (current_command.type == Y_VEL_MOVE)
		{
			//update our target positions to report back to the host computer
			yAxis->targetVelocity = current_command.value1;

			//set the shadow registers with the new values
			setShadowRegisters(yAxis, yAxis->accelMax, yAxis->accelMax, yAxis->targetVelocity*yAxis->distanceToPulses, 0, VELOCITY_MODE);
			// transfer only the y axis
			axisShadowTransfer(yAxis);

			//Immediately set motion to completed
			motion_command_complete = true;
			y_pos_reached = true;

			//set the control mode
			controlMode = INSTANT_VEL_MODE;
		}
		else if (current_command.type == Z_VEL_MOVE)
		{
			//update our target positions to report back to the host computer
			zAxis->targetVelocity = current_command.value1;

			//set the shadow registers with the new values
			setShadowRegisters(zAxis, zAxis->accelMax, zAxis->accelMax, zAxis->targetVelocity*zAxis->distanceToPulses, 0, VELOCITY_MODE);
			// transfer only the x axis
			axisShadowTransfer(zAxis);

			//Immediately set motion to completed
			motion_command_complete = true;
			z_pos_reached = true;

			//set the control mode
			controlMode = INSTANT_VEL_MODE;
		}
		else if (current_command.type == C_VEL_MOVE)
		{
			//update our target positions to report back to the host computer
			cAxis->targetVelocity = current_command.value1;

			//set the shadow registers with the new values
			setShadowRegisters(cAxis, cAxis->accelMax, cAxis->accelMax, cAxis->targetVelocity*cAxis->distanceToPulses, 0, VELOCITY_MODE);
			axisShadowTransfer(cAxis);

			//Immediately set motion to completed
			motion_command_complete = true;
			c_pos_reached = true;
		}
		else if (current_command.type == SET_LINEAR_SPEED)
		{
			//set the tool speed
			toolSpeed = current_command.value1;
			motion_command_complete = true;
		}
		else if (current_command.type == SET_LINEAR_ACCELERATION)
		{
			//set the tool accel
			toolAccel = current_command.value1;
			motion_command_complete = true;
		}

		//update the motion buffer index
		motion_buffer_index++;
		if(motion_buffer_index == MOTION_BUFFER_LEN)
		{
			motion_buffer_index = 0;
		}

		//update the buffer length
		motion_buffer_length--;
	}
}

void startMotion(void)
{
	start_motion = true;

	//set the axis status back to normal
	xAxis->status = NORMAL;
	yAxis->status = NORMAL;
	zAxis->status = NORMAL;
	cAxis->status = NORMAL;

	zBreak(BREAK_OFF); // off
}

void stopMotion(void)
{
	zBreak(BREAK_ON); // on
	start_motion = false;
}

/*
// updates all 4 of the axis tasks at a regular rate
void updateMotionTasks(void)
{
	if (HAL_GetTick() - lastTelemetryUpdateTime > telemetryUpdatePeriod)
	{
		updateAxis(xAxis);
		updateAxis(yAxis);
		updateAxis(zAxis);
		updateAxis(cAxis);

		lastTelemetryUpdateTime = HAL_GetTick();
	}
}*/

// adds a motion command to the motion controllers FIFO buffer
void addCommandToMotionBuffer(motion_command_t c)
{
	if (motion_buffer_length < MOTION_BUFFER_LEN)
	{
		//calculate the end position of the ring buffer
		uint16_t buffer_tail = (motion_buffer_index + motion_buffer_length) % MOTION_BUFFER_LEN;
		//add the command to the end of the ring buffer
		command_buffer[buffer_tail] = c;

		//increment the motion buffer length
		motion_buffer_length++;
	}
}

uint16_t getBufferLength(void)
{
	return motion_buffer_length;
}

void clearBuffer(void)
{
	motion_buffer_length = 0;
}

bool getMotionComplete(void)
{
	return motion_command_complete;
}

void startXHoming(void)
{
	// move the axis at the home reference homing velocity
	setShadowRegisters(xAxis, xAxis->accelMax, xAxis->accelMax, xAxis->homingReferenceSwitchVelocity*xAxis->distanceToPulses, 0, VELOCITY_MODE);
	axisShadowTransfer(xAxis);

	//set the axis status to homing
	xAxis->status = HOMING_REFERENCE_SWITCH;

	SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, "X AXIS HOMING STARTED\r\n");
}

void startYHoming(void)
{
	// move the axis at the home reference homing velocity
	setShadowRegisters(yAxis, yAxis->accelMax, yAxis->accelMax, yAxis->homingReferenceSwitchVelocity*yAxis->distanceToPulses, 0, VELOCITY_MODE);
	axisShadowTransfer(yAxis);

	//set the axis status to homing
	yAxis->status = HOMING_REFERENCE_SWITCH;

	SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, "Y AXIS HOMING STARTED\r\n");
}

void startZHoming(void)
{
	zBreak(BREAK_OFF);
	// move the axis at the home reference homing velocity
	setShadowRegisters(zAxis, zAxis->accelMax, zAxis->accelMax, zAxis->homingReferenceSwitchVelocity*zAxis->distanceToPulses, 0, VELOCITY_MODE);
	axisShadowTransfer(zAxis);

	//set the axis status to homing
	zAxis->status = HOMING_REFERENCE_SWITCH;

	SEGGER_RTT_TerminalOut(RTT_LOG_CHANNEL, "Z AXIS HOMING STARTED\r\n");
}

void stopXHoming(void)
{
	xAxis->status = NORMAL;
}

void stopYHoming(void)
{
	yAxis->status = NORMAL;
}

uint8_t getMotionStatus(void)
{
	return motionStatus;
}
