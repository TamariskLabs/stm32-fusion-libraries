#include "fusion_configurator.h"
#include "fusion_setup.h"
#include "fusion_gpio.h"
#include "fusion_axis.h"
#include "TMC4361A.h"
#include "main.h"

// hold a local reference to the motion axis
static struct tmc4361_t *xAxis;
static struct tmc4361_t *yAxis;
static struct tmc4361_t *zAxis;
static struct tmc4361_t *cAxis;

// create instances for each input pin
fusionGPIO_t inputPin1;
fusionGPIO_t inputPin2;
fusionGPIO_t inputPin3;
fusionGPIO_t inputPin4;
fusionGPIO_t inputPin5;
fusionGPIO_t inputPin6;
fusionGPIO_t inputPin7;
fusionGPIO_t inputPin8;

// create instances for each output pin
fusionGPIO_t outputPin1;
fusionGPIO_t outputPin2;
fusionGPIO_t outputPin3;
fusionGPIO_t outputPin4;

void fusionInit(void)
{
	// --------------- GPIO CONFIGURATION ----------------

	// add input pins

	inputPin1.pin = INPUT_1_Pin;
	inputPin1.port = GPIOB;
	inputPin1.activeState = HIGH;
	fusionAddInputPin(&inputPin1);

	inputPin2.pin = INPUT_2_Pin;
	inputPin2.port = GPIOB;
	inputPin2.activeState = HIGH;
	fusionAddInputPin(&inputPin2);

	inputPin3.pin = INPUT_3_Pin;
	inputPin3.port = GPIOB;
	inputPin3.activeState = HIGH;
	fusionAddInputPin(&inputPin3);

	inputPin4.pin = INPUT_4_Pin;
	inputPin4.port = GPIOB;
	inputPin4.activeState = HIGH;
	fusionAddInputPin(&inputPin4);

	inputPin5.pin = INPUT_5_Pin;
	inputPin5.port = GPIOB;
	inputPin5.activeState = HIGH;
	fusionAddInputPin(&inputPin5);

	inputPin6.pin = INPUT_6_Pin;
	inputPin6.port = GPIOB;
	inputPin6.activeState = HIGH;
	fusionAddInputPin(&inputPin6);

	inputPin7.pin = INPUT_7_Pin;
	inputPin7.port = GPIOA;
	inputPin7.activeState = LOW;
	fusionAddInputPin(&inputPin7);

	inputPin8.pin = INPUT_8_Pin;
	inputPin8.port = GPIOC;
	inputPin8.activeState = LOW;
	fusionAddInputPin(&inputPin8);

	// add output pins
	outputPin1.pin = RELAY_1_Pin;
	outputPin1.port = GPIOB;
	fusionAddOutputPin(&outputPin1);

	outputPin2.pin = RELAY_2_Pin;
	outputPin2.port = GPIOB;
	fusionAddOutputPin(&outputPin2);

	outputPin3.pin = RELAY_3_Pin;
	outputPin3.port = GPIOA;
	fusionAddOutputPin(&outputPin3);

	outputPin4.pin = RELAY_4_Pin;
	outputPin4.port = GPIOA;
	fusionAddOutputPin(&outputPin4);

	// set the gpio serivce rate to 100 hz
	fusionGPIOSetServiceRate(FUSION_SERVICE_RATE_HZ);

	// ------------- MOTION CONTROLLER CONFIGURATION ----------

	// map the local axis pointer to their global pointer from the motion axis buffer
	xAxis = getMotionAxis(X_AXIS_INDEX);
	xAxis->SS_port = X_AXIS_NSS_GPIO_Port;			// gpio pin connected to the controller spi NSS signal.
	xAxis->SS_pin = X_AXIS_NSS_Pin;
	xAxis->START_MOTION_port = START_X_GPIO_Port;	// gpio pin connected to the controller ic start motion signal.
	xAxis->START_MOTION_pin = START_X_Pin;
	xAxis->distanceToPulses = 0.18148;				// (microns or microradians) moved with each step.
	// calculate pulses to distance
	xAxis->pulsesToDistance = \
			1.0/xAxis->distanceToPulses;
	xAxis->stalledErrorTolerance = 1500;			// max axis error in pulses before issuing a stalled error.
	xAxis->useEncoder = true;						// enables the use of encoder feedback.
	xAxis->velMax = 100000;							// max velocity the axis can move in microns per second.
	xAxis->accelMax = 1150000;						// max acceleration the axis can move in pulses per second sq.
	xAxis->lowerEndStop = &inputPin1;				// input pin to use for lower end stop sensor.
	xAxis->upperEndStop = &inputPin2;				// input pin to use for upper end stop sensor.
	xAxis->homingReference = &inputPin1;			// input pin to use for the home reference sensor.
	xAxis->homingType = REFERENCE_SWITCH_AND_INDEX;	// use a reference switch first, then encoder index to home.
	xAxis->homingReferenceSwitchVelocity = -100000;	// velocity to use when searching for the reference home switch in (microns or microradians).
	xAxis->homingIndexVelocity = 80000;				// velocity to use when searching for the encoder index in (microns or microradians).
	xAxis->homeOffsetPosition = 85000;				// zero position offset from the home reference in (microns or microradians).

	// map the local axis pointer to their global pointer from the motion axis buffer
	yAxis = getMotionAxis(Y_AXIS_INDEX);
	yAxis->SS_port = Y_AXIS_NSS_GPIO_Port;			// gpio pin connected to the controller spi NSS signal.
	yAxis->SS_pin = Y_AXIS_NSS_Pin;
	yAxis->START_MOTION_port = START_Y_GPIO_Port;	// gpio pin connected to the controller ic start motion signal.
	yAxis->START_MOTION_pin = START_Y_Pin;
	yAxis->distanceToPulses = 0.18148;				// (microns or microradians) moved with each step.
	// calculate pulses to distance
	yAxis->pulsesToDistance = \
			1.0/yAxis->distanceToPulses;
	yAxis->stalledErrorTolerance = 1500;			// max axis error in pulses before issuing a stalled error.
	yAxis->useEncoder = true;						// enables the use of encoder feedback.
	yAxis->velMax = 85000;							// max velocity the axis can move in microns per second.
	yAxis->accelMax = 500000;						// max acceleration the axis can move in pulses per second sq.
	yAxis->lowerEndStop = &inputPin3;				// input pin to use for lower end stop sensor.
	yAxis->upperEndStop = &inputPin4;				// input pin to use for upper end stop sensor.
	yAxis->homingReference = &inputPin3;			// input pin to use for the home reference sensor.
	yAxis->homingType = REFERENCE_SWITCH_AND_INDEX;	// use a reference switch first, then encoder index to home.
	yAxis->homingReferenceSwitchVelocity = -100000;	// velocity to use when searching for the reference home switch in (microns or microradians).
	yAxis->homingIndexVelocity = 80000;				// velocity to use when searching for the encoder index in (microns or microradians).
	yAxis->homeOffsetPosition = 40000;				// zero position offset from the home reference in (microns or microradians).

	// map the local axis pointers to their global pointer from the motion axis buffer
	zAxis = getMotionAxis(Z_AXIS_INDEX);
	zAxis->SS_port = Z_AXIS_NSS_GPIO_Port;			// gpio pin connected to the controller spi NSS signal.
	zAxis->SS_pin = Z_AXIS_NSS_Pin;
	zAxis->START_MOTION_port = START_Z_GPIO_Port;	// gpio pin connected to the controller ic start motion signal.
	zAxis->START_MOTION_pin = START_Z_Pin;
	zAxis->distanceToPulses = 0.6400;				// (microns or microradians) moved with each step.
	// calculate pulses to distance
	zAxis->pulsesToDistance = \
			1.0/zAxis->distanceToPulses;
	zAxis->stalledErrorTolerance = 1500;			// max axis error in pulses before issuing a stalled error.
	zAxis->useEncoder = true;						// enables the use of encoder feedback.
	zAxis->velMax = 65000;							// max velocity the axis can move in microns per second.
	zAxis->accelMax = 700000;						// max acceleration the axis can move in pulses per second sq.
	zAxis->lowerEndStop = &inputPin5;				// input pin to use for lower end stop sensor.
	zAxis->upperEndStop = &inputPin6;				// input pin to use for upper end stop sensor.
	zAxis->homingReference = &inputPin5;			// input pin to use for the home reference sensor.
	zAxis->homingType = REFERENCE_SWITCH;			// use a reference switch first, then encoder index to home.
	zAxis->homingReferenceSwitchVelocity = -25000;	// velocity to use when searching for the reference home switch in (microns or microradians).
	zAxis->homeOffsetPosition = -5000;				// zero position offset from the home reference in (microns or microradians).

	// map the local axis pointers to their global pointer from the motion axis buffer
	cAxis = getMotionAxis(C_AXIS_INDEX);
	cAxis->SS_port = C_AXIS_NSS_GPIO_Port;			// gpio pin connected to the controller spi NSS signal.
	cAxis->SS_pin = C_AXIS_NSS_Pin;
	cAxis->START_MOTION_port = START_C_GPIO_Port;	// gpio pin connected to the controller ic start motion signal.
	cAxis->START_MOTION_pin = START_C_Pin;
	cAxis->distanceToPulses = 0.18148;				// (microns or microradians) moved with each step.
	// calculate pulses to distance
	cAxis->pulsesToDistance = \
			1.0/cAxis->distanceToPulses;
	cAxis->stalledErrorTolerance = 1500;			// max axis error in pulses before issuing a stalled error.
	cAxis->useEncoder = false;						// enables the use of encoder feedback.
	cAxis->velMax = 300000;							// max velocity the axis can move in microns per second.
	cAxis->accelMax = 40000;						// max acceleration the axis can move in pulses per second sq.
	cAxis->homingType = NONE;
}


// called in the main loop as fast as possible.  This allows each of the
// fusion modules to service tasks at a regular rate.  This should be called
// faster then the service rate setting.
void fusionService(void)
{
	// service the gpio module
	fusionGPIOService();

}
