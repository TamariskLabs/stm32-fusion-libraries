#include "tmc4361A.h"
#include "main.h"
#include <string.h>

#define CLK_FREQ 9000000		// FCLK frequency is set to 9Mhz

extern SPI_HandleTypeDef hspi3;

static uint8_t tx_buffer[5];
static uint8_t rx_buffer[5];
static bool bus_busy = false;

void setRegisterBit(tmc4361_t * tmc, const uint8_t address, const uint8_t bit);
void clearRegisterBit(tmc4361_t * tmc, const uint8_t address, const uint8_t bit);
bool readRegisterBit(tmc4361_t * tmc, const uint8_t address, const uint8_t bit);

void tmc4361_begin(tmc4361_t * tmc){
	//Trigger software reset
	tmc4361_writeRegister(tmc, TMC4361_RESET_CLK_GATING_REGISTER, 0x525354 << 8);
	tmc4361_writeRegister(tmc, TMC4361_CLK_FREQ_REGISTER, CLK_FREQ);
	tmc4361_setOutputTimings(tmc, 5, 5);
	tmc4361_writeRegister(tmc, TMC4361_EVENT_CLEAR_CONF_REGISTER, 0xFFFFFFFF);
	tmc4361_clearEvents(tmc);
}

void tmc4361_writeRegister(tmc4361_t * tmc, uint8_t reg_address, uint32_t data){
	bus_busy = true;

	tx_buffer[0] = reg_address | 0x80;
	tx_buffer[1] = (uint8_t) ((data >> 24) & 0xFF);
	tx_buffer[2] = (uint8_t) ((data >> 16) & 0xFF);
	tx_buffer[3] = (uint8_t) ((data >> 8) & 0xFF);
	tx_buffer[4] = (uint8_t) ((data >> 0) & 0xFF);

	HAL_GPIO_WritePin(tmc->SS_port, tmc->SS_pin, 0);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3, tx_buffer, 5, 1000);
	HAL_GPIO_WritePin(tmc->SS_port, tmc->SS_pin, 1);
}

uint32_t tmc4361_readRegister(tmc4361_t * tmc, uint8_t reg_address){
	bus_busy = true;
	bzero(tx_buffer, 5);
	bzero(rx_buffer, 5);
	tx_buffer[0] = reg_address & 0x7F;

	HAL_GPIO_WritePin(tmc->SS_port, tmc->SS_pin, 0);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_SPI_TransmitReceive(&hspi3, tx_buffer, rx_buffer, 5, 1000);
	HAL_GPIO_WritePin(tmc->SS_port, tmc->SS_pin, 1);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_GPIO_WritePin(tmc->SS_port, tmc->SS_pin, 0);
	HAL_SPI_TransmitReceive(&hspi3, tx_buffer, rx_buffer, 5, 1000);
	HAL_GPIO_WritePin(tmc->SS_port, tmc->SS_pin, 1);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	uint32_t retval = 0;
	retval |= ((uint32_t)rx_buffer[1]) << 24;
	retval |= ((uint32_t)rx_buffer[2]) << 16;
	retval |= ((uint32_t)rx_buffer[3]) << 8;
	retval |= ((uint32_t)rx_buffer[4]) << 0;
	bus_busy = false;

	return retval;
}


bool tmc4361_checkFlag(tmc4361_t * tmc, tmc4361_FlagType flag)
{
  return readRegisterBit(tmc, TMC4361_STATUS_REGISTER, flag);
}

bool tmc4361_isTargetReached(tmc4361_t * tmc)
{
  return tmc4361_checkFlag(tmc, TARGET_REACHED_F);
}

void tmc4361_clearEvents(tmc4361_t * tmc)
{
  tmc4361_writeRegister(tmc, TMC4361_EVENTS_REGISTER, 0xFFFFFFFF);
}

bool tmc4361_checkEvent(tmc4361_t * tmc, tmc4361_EventType event)
{
  bool value = readRegisterBit(tmc, TMC4361_EVENTS_REGISTER, event);

  if (value)
    tmc4361_writeRegister(tmc, TMC4361_EVENTS_REGISTER, 1 << event);

  return value;
}

void tmc4361_set_encoder_single_ended(tmc4361_t * tmc)
{
  long generalConfigReg = tmc4361_readRegister(tmc, TMC4361_GENERAL_CONFIG_REGISTER);

  bitWrite(generalConfigReg, 12);

  tmc4361_writeRegister(tmc, TMC4361_GENERAL_CONFIG_REGISTER, generalConfigReg);
}

void tmc4361_setOutputsPolarity(tmc4361_t * tmc, bool stepInverted, bool dirInverted)
{
  long generalConfigReg = tmc4361_readRegister(tmc, TMC4361_GENERAL_CONFIG_REGISTER);
  tmc4361_writeRegister(tmc, TMC4361_GENERAL_CONFIG_REGISTER, generalConfigReg);
}

/*
 * Step width is how long each step pulse should be (ambigious as to whether that refers to high time or entire pulse cycle)
 * dirSetupTime is how long to wait after switching direction before issuing another step pulse
 *
 * Both values are in uS
 */
void tmc4361_setOutputTimings(tmc4361_t * tmc, int stepWidth, int dirSetupTime)
{
  long registerValue =
    ((stepWidth * CLK_FREQ / 1000000L - 1) & 0xFFFF) |
    (((dirSetupTime * CLK_FREQ / 1000000L) & 0xFFFF) << 16);
  tmc4361_writeRegister(tmc, TMC4361_STP_LENGTH_ADD, registerValue);
}

void tmc4361_setRampMode(tmc4361_t * tmc, tmc4361_RampMode mode, tmc4361_RampType type)
{
  tmc4361_writeRegister(tmc, TMC4361_RAMP_MODE_REGISTER, mode | type);
  tmc4361_writeRegister(tmc, TMC4361_SH_RAMP_MODE_REGISTER, mode | type);
}

long tmc4361_getCurrentPosition(tmc4361_t * tmc)
{
  return tmc4361_readRegister(tmc, TMC4361_X_ACTUAL_REGISTER);
}

void tmc4361_setCurrentPosition(tmc4361_t * tmc, long position)
{
  tmc4361_writeRegister(tmc, TMC4361_X_ACTUAL_REGISTER, position);
}

float tmc4361_getCurrentSpeed(tmc4361_t * tmc)
{
  return (float)tmc4361_readRegister(tmc, TMC4361_V_ACTUAL_REGISTER);
}

float tmc4361_getCurrentAcceleration(tmc4361_t * tmc)
{
  return (float)tmc4361_readRegister(tmc, TMC4361_A_ACTUAL_REGISTER);
}

/*
 * POSITIVE VALUES ONLY
 */
void tmc4361_setMaxSpeed(tmc4361_t * tmc, float speed)
{
  tmc4361_writeRegister(tmc, TMC4361_V_MAX_REGISTER, floatToFixedPoint((float)speed, 8));
}


/*
 * POSITIVE VALUES ONLY
 */
void tmc4361_setRampSpeeds(tmc4361_t * tmc, float startSpeed, float stopSpeed, float breakSpeed)
{
  tmc4361_writeRegister(tmc, TMC4361_V_START_REGISTER, floatToFixedPoint((float)(startSpeed), 8));
  tmc4361_writeRegister(tmc, TMC4361_V_STOP_REGISTER, floatToFixedPoint((float)(stopSpeed), 8));
  tmc4361_writeRegister(tmc, TMC4361_V_BREAK_REGISTER, floatToFixedPoint((float)(breakSpeed), 8));
}


/*
 * POSITIVE VALUES ONLY
 */
void tmc4361_setAccelerations(tmc4361_t * tmc, float maxAccel, float maxDecel, float startAccel, float finalDecel)
{
  tmc4361_writeRegister(tmc, TMC4361_A_MAX_REGISTER, floatToFixedPoint((float)(maxAccel), 2) & 0xFFFFFF);
  tmc4361_writeRegister(tmc, TMC4361_D_MAX_REGISTER, floatToFixedPoint((float)(maxDecel), 2) & 0xFFFFFF);
  tmc4361_writeRegister(tmc, TMC4361_A_START_REGISTER, floatToFixedPoint((float)(startAccel), 2) & 0xFFFFFF);
  tmc4361_writeRegister(tmc, TMC4361_D_FINAL_REGISTER, floatToFixedPoint((float)(finalDecel), 2) & 0xFFFFFF);
}


/*
 * POSITIVE VALUES ONLY
 */
void tmc4361_setBowValues(tmc4361_t * tmc, long bow1, long bow2, long bow3, long bow4)
{
  tmc4361_writeRegister(tmc, TMC4361_BOW_1_REGISTER, (bow1) & 0xFFFFFF);
  tmc4361_writeRegister(tmc, TMC4361_BOW_2_REGISTER, (bow2) & 0xFFFFFF);
  tmc4361_writeRegister(tmc, TMC4361_BOW_3_REGISTER, (bow3) & 0xFFFFFF);
  tmc4361_writeRegister(tmc, TMC4361_BOW_4_REGISTER, (bow4) & 0xFFFFFF);
}

long tmc4361_getTargetPosition(tmc4361_t * tmc)
{
  return tmc4361_readRegister(tmc, TMC4361_X_TARGET_REGISTER);
}

void tmc4361_setTargetPosition(tmc4361_t * tmc, long position)
{
  tmc4361_writeRegister(tmc, TMC4361_X_TARGET_REGISTER, position);
}

void tmc4361_setStepsPerRev(tmc4361_t * tmc, uint32_t spr)
{
	uint32_t reg_val = tmc4361_readRegister(tmc, TMC4361_STEP_CONF_REGISTER);
	reg_val &= 0xFFFF0000;
	reg_val |= spr;
	tmc4361_writeRegister(tmc, TMC4361_STEP_CONF_REGISTER, reg_val);
}

void tmc4361_setClosedLoopSetting(tmc4361_t * tmc, uint8_t cls)
{
	uint32_t reg_val = tmc4361_readRegister(tmc, TMC4361_ENCODER_INPUT_CONFIG_REGISTER);
	reg_val |= cls << 22;
	tmc4361_writeRegister(tmc, TMC4361_ENCODER_INPUT_CONFIG_REGISTER, reg_val);
}

void tmc4361_invertEncoder(tmc4361_t * tmc, uint8_t invert)
{
	uint32_t reg_val = tmc4361_readRegister(tmc, TMC4361_ENCODER_INPUT_CONFIG_REGISTER);
	reg_val |= invert << 29;
	tmc4361_writeRegister(tmc, TMC4361_ENCODER_INPUT_CONFIG_REGISTER, reg_val);
}

void tmc4361_stop(tmc4361_t * tmc)
{
  tmc4361_setMaxSpeed(tmc, 0.0);
}

void setRegisterBit(tmc4361_t * tmc, const uint8_t address, const uint8_t bit)
{
  uint32_t value = tmc4361_readRegister(tmc, address);
  //bitSet(value, bit);
  tmc4361_writeRegister(tmc, address, value);
}

void clearRegisterBit(tmc4361_t * tmc, const uint8_t address, const uint8_t bit)
{
  uint32_t value = tmc4361_readRegister(tmc, address);
  //bitClear(value, bit);
  tmc4361_writeRegister(tmc, address, value);
}

bool readRegisterBit(tmc4361_t * tmc, const uint8_t address, const uint8_t bit)
{
	uint32_t read_reg = tmc4361_readRegister(tmc, address);
	return bitRead(read_reg, bit);
}

long floatToFixedPoint(float value, int decimalPlaces)
{
  value *= (float)((uint32_t)(((uint32_t)1u )<< decimalPlaces));
  return (int32_t)((value > 0.0) ? (value + 0.5) : (value - 0.5));
}

float fixedPointToFloat(long value, int decimalPlaces)
{
  return (float)(value) / (float)(1 << decimalPlaces);
}

void setShadowRegisters(tmc4361_t * tmc, float maxAccel, float maxDecel, float speed, int32_t position, tmc4361_RampMode m)
{
	tmc4361_writeRegister(tmc, TMC4361_SH_RAMP_MODE_REGISTER, TRAPEZOIDAL_RAMP | m);
	tmc4361_writeRegister(tmc, TMC4361_SH_A_MAX_REGISTER, floatToFixedPoint((float)(maxAccel*0.5), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_D_MAX_REGISTER, floatToFixedPoint((float)(maxDecel*0.5), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_V_MAX_REGISTER, floatToFixedPoint((float)speed, 8));
	tmc4361_writeRegister(tmc, TMC4361_SH_A_START_REGISTER, floatToFixedPoint((float)(0.0), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_D_FINAL_REGISTER, floatToFixedPoint((float)(0.0), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_VBREAK_REGISTER, floatToFixedPoint((float)0.0, 8));
	tmc4361_writeRegister(tmc, TMC4361_X_TARGET_PIPE_0_REGSISTER, position);
}

void setShadowRegisters6Point(tmc4361_t * tmc, float maxAccel, float maxDecel, float speed, int32_t position, tmc4361_RampMode m)
{
	tmc4361_writeRegister(tmc, TMC4361_SH_RAMP_MODE_REGISTER, TRAPEZOIDAL_RAMP | m);
	tmc4361_writeRegister(tmc, TMC4361_SH_A_MAX_REGISTER, floatToFixedPoint((float)(maxAccel*0.5), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_D_MAX_REGISTER, floatToFixedPoint((float)(maxDecel*0.5), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_V_MAX_REGISTER, floatToFixedPoint((float)speed, 8));
	tmc4361_writeRegister(tmc, TMC4361_SH_A_START_REGISTER, floatToFixedPoint((float)(maxAccel), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_D_FINAL_REGISTER, floatToFixedPoint((float)(maxDecel), 2) & 0xFFFFFF);
	tmc4361_writeRegister(tmc, TMC4361_SH_VBREAK_REGISTER, floatToFixedPoint((float)speed*0.5, 8));
	tmc4361_writeRegister(tmc, TMC4361_X_TARGET_PIPE_0_REGSISTER, position);
}

