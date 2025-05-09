#ifndef __FUSION_USER_SETUP_H
#define __FUSION_USER_SETUP_H


// -------------- CONTROLLER SETTINGS ---------------
#define FUSION_SERVICE_RATE_HZ 100			// sets the rate at which all fusion modules update their tasks


// -------------- GPIO SETTINGS ---------------------

// -------------- MOTION CONTROLLER SETTINGS --------

// maps axis to axis buffer
#define X_AXIS_INDEX	 0
#define Y_AXIS_INDEX	 1
#define Z_AXIS_INDEX	 2
#define C_AXIS_INDEX	 3
// sets the directionality of the encoder (which way is positive)
#define INVERT_X_ENCODER true				// set to true to flip the encoder direction
#define INVERT_Y_ENCODER true				// set to true to flip the encoder direction
#define INVERT_Z_ENCODER false				// set to true to flip the encoder direction
// sets homing parameters
#define X_HOME_OFFSET 					1500
#define Y_HOME_OFFSET 					1500
#define Z_HOME_OFFSET 					1000
#define X_HOMING_OPTO_SEARCH_SPEED		1000
#define Y_HOMING_OPTO_SEARCH_SPEED		1000
#define Z_HOMING_OPTO_SEARCH_SPEED		1000
#define X_HOMING_INDEX_SEARCH_SPEED		1000
#define Y_HOMING_INDEX_SEARCH_SPEED		1000
#define Z_HOMING_INDEX_SEARCH_SPEED		1000

#endif
