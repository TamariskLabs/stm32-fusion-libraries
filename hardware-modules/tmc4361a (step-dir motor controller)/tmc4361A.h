#ifndef INC_TMC4361A_H_
#define INC_TMC4361A_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

typedef struct {
	//the slave select pin definitions
	GPIO_TypeDef *SS_port;
	uint16_t SS_pin;
	//the reset pin definitions
	GPIO_TypeDef *RST_port;
	uint16_t RST_pin;

	//axis motion parameters
	uint16_t pulsesPerRev;		//number of pulses required to rotate the motor one rotation
	float distancePerRev;			//distance the axis moves (microns or microradians) per single rotations
	float pulsesToDistance;		//converts pulses to distance (microns)
	float distanceToPulses;	  //converts distance (microns) to pulses

} tmc4361_t;

#define TMC4361_GENERAL_CONFIG_REGISTER 0x0
#define TMC4361_REFERENCE_CONFIG_REGISTER 0x01
#define TMC4361_START_CONFIG_REGISTER 0x2
#define TMC4361_INPUT_FILTER_REGISTER 0x3
#define TMC4361_SPIOUT_CONF_REGISTER 0x04
#define TMC4361_CURRENT_CONF_REGISTER 0x05
#define TMC4361_SCALE_VALUES_REGISTER 0x06
#define TMC4361_ENCODER_INPUT_CONFIG_REGISTER 0x07
#define TMC4361_ENC_IN_DATA 0x08
#define TMC4361_ENC_OUT_DATA 0x09
#define TMC4361_STEP_CONF_REGISTER 0x0A
#define TMC4361_SPI_STATUS_SELECTION 0x0B
#define TMC4361_EVENT_CLEAR_CONF_REGISTER 0x0C
#define TMC4361_INTERRUPT_CONFIG_REGISTER 0x0D
#define TMC4361_EVENTS_REGISTER 0x0E
#define TMC4361_STATUS_REGISTER 0x0F
#define TMC4361_STP_LENGTH_ADD 0x10
#define TMC4361_START_OUT_ADD_REGISTER 0x11
#define TMC4361_GEAR_RATIO_REGISTER 0x12
#define TMC4361_START_DELAY_REGISTER 0x13
#define TMC4361_STDBY_DELAY_REGISTER 0x15
#define TMC4361_FREEWHEEL_DELAY_REGISTER 0x16
#define TMC4361_VRDV_SCALE_LIMIT_REGISTER 0x17
#define TMC4361_UP_SCALE_DELAY_REGISTER 0x18
#define TMC4361_HOLD_SCALE_DELAY_REGISTER 0x19
#define TMC4361_DRV_SCALE_DELAY_REGISTER 0x1A
#define TMC4361_BOOST_TIME_REGISTER 0x1B
#define TMC4361_CLOSE_LOOP_REGISTER 0x1C
#define TMC4361_DAC_ADDR_REGISTER 0x1D
#define TMC4361_HOME_SAFETY_MARGIN_REGISTER 0x1E
#define TMC4361_PWM_FREQ_CHOPSYNC_REGISTER 0x
#define TMC4361_RAMP_MODE_REGISTER 0x20
#define TMC4361_X_ACTUAL_REGISTER 0x21
#define TMC4361_V_ACTUAL_REGISTER 0x22
#define TMC4361_A_ACTUAL_REGISTER 0x23
#define TMC4361_V_MAX_REGISTER 0x24
#define TMC4361_V_START_REGISTER 0x25
#define TMC4361_V_STOP_REGISTER 0x26
#define TMC4361_V_BREAK_REGISTER 0x27
#define TMC4361_A_MAX_REGISTER 0x28
#define TMC4361_D_MAX_REGISTER 0x29
#define TMC4361_A_START_REGISTER 0x2A
#define TMC4361_D_FINAL_REGISTER 0x2B
#define TMC4361_D_STOP_REGISTER 0x2C
#define TMC4361_BOW_1_REGISTER 0x2D
#define TMC4361_BOW_2_REGISTER 0x2E
#define TMC4361_BOW_3_REGISTER 0x2F
#define TMC4361_BOW_4_REGISTER 0x30
#define TMC4361_CLK_FREQ_REGISTER 0x31
#define TMC4361_POSITION_COMPARE_REGISTER 0x32
#define TMC4361_VIRTUAL_STOP_LEFT_REGISTER 0x33
#define TMC4361_VIRTUAL_STOP_RIGHT_REGISTER 0x34
#define TMC4361_X_HOME_REGISTER 0x35
#define TMC4361_X_LATCH_REGISTER 0x36
#define TMC4361_X_TARGET_REGISTER 0x37
#define TMC4361_X_TARGET_PIPE_0_REGSISTER 0x38
#define TMC4361_SH_V_MAX_REGISTER 0x40
#define TMC4361_SH_A_MAX_REGISTER 0x41
#define TMC4361_SH_D_MAX_REGISTER 0x42
#define TMC4361_SH_A_START_REGISTER 0x43
#define TMC4361_SH_D_FINAL_REGISTER 0x44
#define TMC4361_SH_VBREAK_REGISTER 0x45
#define TMC4361_SH_V_START_REGISTER 0x46
#define TMC4361_SH_V_STOP_REGISTER 0x47
#define TMC4361_SH_BOW_1_REGISTER 0x48
#define TMC4361_SH_BOW_2_REGISTER 0x49
#define TMC4361_SH_BOW_3_REGISTER 0x4A
#define TMC4361_SH_BOW_4_REGISTER 0x4B
#define TMC4361_SH_RAMP_MODE_REGISTER 0x4C
#define TMC4361_D_FREEZE_REGISTER 0x4E
#define TMC4361_RESET_CLK_GATING_REGISTER 0x4F
#define TMC4361_ENCODER_POSITION_REGISTER 0x50
#define TMC4361_ENCODER_HOME_VALUE_REGISTER 0x51
#define TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER 0x54
#define TMC4361_ENCODER_VELOCITY_REGISTER 0x65
#define TMC4361_COVER_LOW_REGISTER 0x6c
#define TMC4361_COVER_HIGH_REGISTER 0x6d
#define TMC4361_MICROSTEP_READ_REGISTER 0x79
#define TMC4361_VERSION_REGISTER 0x7f

typedef enum {
	VELOCITY_MODE = 0x00, POSITIONING_MODE = (0x01 << 2)
} tmc4361_RampMode;

typedef enum {
	HOLD_RAMP = 0x00, //Follow max speed (rectangle shape)
	TRAPEZOIDAL_RAMP = 0x01,
	S_SHAPED_RAMP = 0x02
} tmc4361_RampType;

//See Status Events Register description for details
typedef enum {
	TARGET_REACHED = 0,
	POS_COMP_REACHED,
	VEL_REACHED,
	VEL_STATE_ZERO,
	VEL_STATE_POS,
	VEL_STATE_NEG,
	RAMP_STATE_ACCEL_ZERO,
	RAMP_STATE_ACCEL_POS,
	RAMP_STATE_ACCEL_NEG,
	MAX_PHASE_TRAP,
	FROZEN,
	STOPL,
	STOPR,
	VSTOPL_ACTIVE,
	VSTOPR_ACTIVE,
	HOME_ERROR,
	XLATCH_DONE,
	FS_ACTIVE,
	ENC_FAIL,
	N_ACTIVE,
	ENC_DONE,
	SER_ENC_DATA_FAIL,
	SER_DATA_DONE = 23,
	SERIAL_ENC_FLAG,
	COVER_DONE,
	ENC_VEL_ZERO,
	CL_MAX,
	CL_FIT,
	STOP_ON_STALL_EV,
	MOTOR_EV,
	RST_EV
} tmc4361_EventType;

typedef enum {
	TARGET_REACHED_F = 0,
	POS_COMP_REACHED_F,
	VEL_REACHED_F,
	VEL_STATE_F0,
	VEL_STATE_F1,
	RAMP_STATE_F0,
	RAMP_STATE_F1,
	STOPL_ACTIVE_F,
	STOPR_ACTIVE_F,
	VSTOPL_ACTIVE_F,
	VSTOPR_ACTIVE_F,
	ACTIVE_STALL_F,
	HOME_ERROR_F,
	FS_ACTIVE_F,
	ENC_FAIL_F,
	N_ACTIVE_F,
	ENC_LATCH_F,
} tmc4361_FlagType;

//Reference switch configuration register
typedef enum {
	STOP_LEFT_EN = 0,
	STOP_RIGHT_EN,
	POL_STOP_LEFT,
	POL_STOP_RIGHT,
	INVERT_STOP_DIRECTION,
	SOFT_STOP_EN,
	VIRTUAL_LEFT_LIMIT_EN,
	VIRTUAL_RIGHT_LIMIT_EN,
	VIRT_STOP_MODE,
	LATCH_X_ON_INACTIVE_L = 10,
	LATCH_X_ON_ACTIVE_L,
	LATCH_X_ON_INACTIVE_R,
	LATCH_X_ON_ACTIVE_R,
	STOP_LEFT_IS_HOME,
	STOP_RIGHT_IS_HOME,
	HOME_EVENT,
	START_HOME_TRACKING = 20,
	CLR_POS_AT_TARGET,
	CIRCULAR_MOVEMENT_EN,
	POS_COMP_OUTPUT,
	POS_COMP_SOURCE = 25,
	STOP_ON_STALL,
	DRV_AFTER_STALL,
	MODIFIED_POS_COMPARE,
	AUTOMATIC_COVER = 30,
	CIRCULAR_ENC_EN
} tmc4361_ReferenceConfRegisterFields;

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitWrite(value, bit) ((value) |= (1UL << (bit)))

uint32_t tmc4361_readRegister(tmc4361_t *tmc, uint8_t reg_address);
void tmc4361_writeRegister(tmc4361_t *tmc, uint8_t reg_address, uint32_t data);
void tmc4361_begin(tmc4361_t * tmc);
bool tmc4361_checkFlag(tmc4361_t * tmc, tmc4361_FlagType flag);
bool tmc4361_isTargetReached(tmc4361_t * tmc);
void tmc4361_clearEvents(tmc4361_t * tmc);
bool tmc4361_checkEvent(tmc4361_t * tmc, tmc4361_EventType event);
void tmc4361_setOutputsPolarity(tmc4361_t * tmc, bool stepInverted, bool dirInverted);
void tmc4361_setOutputTimings(tmc4361_t * tmc, int stepWidth, int dirSetupTime);
void tmc4361_setRampMode(tmc4361_t * tmc, tmc4361_RampMode mode, tmc4361_RampType type);
long tmc4361_getCurrentPosition(tmc4361_t * tmc);
void tmc4361_setStepsPerRev(tmc4361_t * tmc, uint32_t);
void tmc4361_setClosedLoopSetting(tmc4361_t * tmc, uint8_t);
void tmc4361_setCurrentPosition(tmc4361_t * tmc, long position);
float tmc4361_getCurrentSpeed(tmc4361_t * tmc);
float tmc4361_getCurrentAcceleration(tmc4361_t * tmc);
void tmc4361_setMaxSpeed(tmc4361_t * tmc, float speed);
void tmc4361_setRampSpeeds(tmc4361_t * tmc, float startSpeed, float stopSpeed, float breakSpeed);
void tmc4361_setAccelerations(tmc4361_t * tmc, float maxAccel, float maxDecel, float startAccel, float finalDecel);
void tmc4361_setBowValues(tmc4361_t * tmc, long bow1, long bow2, long bow3, long bow4);
long tmc4361_getTargetPosition(tmc4361_t * tmc);
void tmc4361_setTargetPosition(tmc4361_t * tmc, long position);
void tmc4361_stop(tmc4361_t * tmc);
void tmc4361_set_encoder_single_ended(tmc4361_t * tmc);
void tmc4361_invertEncoder(tmc4361_t * tmc, uint8_t);
void setShadowRegisters(tmc4361_t * tmc, float, float, float, int32_t, tmc4361_RampMode);
void setShadowRegisters6Point(tmc4361_t * tmc, float, float, float, int32_t, tmc4361_RampMode);

long floatToFixedPoint(float value, int decimalPlaces);
float fixedPointToFloat(long value, int decimalPlaces);
#endif /* INC_TMC4361A_H_ */
