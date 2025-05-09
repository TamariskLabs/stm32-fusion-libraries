#include "bq76905.h"

// ---------- settings ----------------------


// ---------- working variables -------------


// ---------- static declarations -----------
static HAL_StatusTypeDef BQ76905_WriteRegister(BQ76905_HandleTypeDef *hbq, uint8_t reg, uint16_t data);
static HAL_StatusTypeDef BQ76905_ReadRegister(BQ76905_HandleTypeDef *hbq, uint8_t reg, uint16_t *data);
static HAL_StatusTypeDef BQ76905_ReadSubcommand(BQ76905_HandleTypeDef *hbq, uint16_t address, uint8_t  *data, uint8_t len);
static HAL_StatusTypeDef BQ76905_WriteSubcommand(BQ76905_HandleTypeDef *hbq, uint16_t address, uint8_t  *data, uint8_t len);
static HAL_StatusTypeDef BQ76905_WriteDataMemory(BQ76905_HandleTypeDef *hbq, uint16_t address, uint8_t  *data, uint8_t len);


// ---------- function definitions ----------
HAL_StatusTypeDef BQ76905_fetchSystemStatus(BQ76905_HandleTypeDef *hbq)
{
	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	// read the device status
	uint16_t data = 0;
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_BATTERY_STATUS_REG, &data);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// map the device status from bitmap to bool's
	hbq->systemStatus.sleep = (data & BQ76905_SLEEP_BIT);
	hbq->systemStatus.deepsleep = (data & BQ76905_DEEPSLEEP_BIT);
	hbq->systemStatus.sa = (data & BQ76905_SA_BIT);
	hbq->systemStatus.ss = (data & BQ76905_SS_BIT);
	hbq->systemStatus.fetEnable = (data & BQ76905_FET_EN_BIT);
	hbq->systemStatus.por = (data & BQ76905_POR_BIT);
	hbq->systemStatus.sleepEnabled = (data & BQ76905_SLEEP_EN_BIT);
	hbq->systemStatus.cfgUpdateMode = (data & BQ76905_CFGUPDATE_BIT);
	hbq->systemStatus.alertPin = (data & BQ76905_ALERTPIN_BIT);
	hbq->systemStatus.chargeActive = (data & BQ76905_CHG_BIT);
	hbq->systemStatus.dischargeActive = (data & BQ76905_DIS_BIT);
	hbq->systemStatus.chargerDetected = (data & BQ76905_CHGDET_FLAG_BIT);
	// parse the access mode
	if ((data & BQ76905_ACCCESS_BITS) == BQ76905_FULLACCESS_MODE_BITS)
	{
		hbq->systemStatus.fullAccessMode = true;
		hbq->systemStatus.sealedMode = false;
		hbq->systemStatus.uninitializedMode = false;
	}
	else if ((data & BQ76905_ACCCESS_BITS) == BQ76905_SEALED_MODE_BITS)
	{
		hbq->systemStatus.fullAccessMode = false;
		hbq->systemStatus.sealedMode = true;
		hbq->systemStatus.uninitializedMode = false;
	}
	else
	{
		hbq->systemStatus.fullAccessMode = false;
		hbq->systemStatus.sealedMode = false;
		hbq->systemStatus.uninitializedMode = true;
	}

	// return the successful result ------------------------------------
	return i2c_read_result;
}


HAL_StatusTypeDef BQ76905_fetchSafetyStatus(BQ76905_HandleTypeDef *hbq)
{
	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	// read the safety alert a and safety status a ----------------------
	uint16_t safety_a = 0;
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_SAFETY_ALERT_A_REG, &safety_a);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// split the uint into its individual register values
	uint8_t alert_a = safety_a & 0xFF;
	uint8_t status_a = (safety_a >> 8) & 0xFF;


	// read the safety alert b and safety status b -----------------------
	uint16_t safety_b = 0;
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_SAFETY_ALERT_B_REG, &safety_b);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// split the uint into its individual register values
	uint8_t alert_b = safety_b & 0xFF;
	uint8_t status_b = (safety_b >> 8) & 0xFF;

	// clear any faults
	if ((safety_a != 0) || (safety_b != 0))
	{
		BQ76905_ClearFaults(hbq);
	}


	// parse the results into individual alert values -------------------
	hbq->faultAlerts.cellOvervoltage = alert_a & 0x80;
	hbq->faultAlerts.cellUndervoltage = alert_a & 0x40;
	hbq->faultAlerts.dischargeShortCircuit = alert_a & 0x20;
	hbq->faultAlerts.dischargeOvercurrent_1 = alert_a & 0x10;
	hbq->faultAlerts.dischargeOvercurrent_2 = alert_a & 0x08;
	hbq->faultAlerts.chargeOvercurrent = alert_a & 0x04;
	hbq->faultAlerts.dischargeOvertemperature = alert_b & 0x80;
	hbq->faultAlerts.chargeOvertemperature = alert_b & 0x40;
	hbq->faultAlerts.dischargeUndertemperature = alert_b & 0x20;
	hbq->faultAlerts.dischargeUndertemperature = alert_b & 0x10;
	hbq->faultAlerts.internalOvertemperature = alert_b & 0x08;
	hbq->faultAlerts.hostWatchdog = alert_b & 0x04;
	hbq->faultAlerts.vrefMeasurement = alert_b & 0x02;
	hbq->faultAlerts.vssMeasurement = alert_b & 0x01;

	// parse the results into individual alert values -------------------
	hbq->faultStatus.cellOvervoltage = status_a & 0x80;
	hbq->faultStatus.cellUndervoltage = status_a & 0x40;
	hbq->faultStatus.dischargeShortCircuit = status_a & 0x20;
	hbq->faultStatus.dischargeOvercurrent_1 = status_a & 0x10;
	hbq->faultStatus.dischargeOvercurrent_2 = status_a & 0x08;
	hbq->faultStatus.chargeOvercurrent = status_a & 0x04;
	hbq->faultStatus.currentProtectionLatch = status_a & 0x02;
	hbq->faultStatus.regout = status_a & 0x01;
	hbq->faultStatus.dischargeOvertemperature = status_b & 0x80;
	hbq->faultStatus.chargeOvertemperature = status_b & 0x40;
	hbq->faultStatus.dischargeUndertemperature = status_b & 0x20;
	hbq->faultStatus.chargeUndertemperature = status_b & 0x10;
	hbq->faultStatus.internalOvertemperature = status_b & 0x08;
	hbq->faultStatus.hostWatchdog = status_b & 0x04;
	hbq->faultStatus.vrefMeasurement = status_b & 0x02;
	hbq->faultStatus.vssMeasurement = status_b & 0x01;

	// return the successful result ------------------------------------
	return i2c_read_result;
}


HAL_StatusTypeDef BQ76905_fetchMeasurements(BQ76905_HandleTypeDef *hbq)
{
	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	// read the measurement data
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_PACK_VOLTAGE_REG, &hbq->measurements.packVoltage);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_CELL1_VOLT_REG, &hbq->measurements.cellVoltage_1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_CELL2_VOLT_REG, &hbq->measurements.cellVoltage_2);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_CELL3_VOLT_REG, &hbq->measurements.cellVoltage_3);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_CELL4_VOLT_REG, &hbq->measurements.cellVoltage_4);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_CELL5_VOLT_REG, &hbq->measurements.cellVoltage_5);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_CC1_REG, (uint16_t*)&hbq->measurements.packCurrent);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_INTERNAL_TEMP_REG, &hbq->measurements.internalTemperature);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_ReadRegister(hbq, BQ76905_TS_TEMP_REG, &hbq->measurements.externalTemperature);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// return the successful result ------------------------------------
	return i2c_read_result;
}


HAL_StatusTypeDef BQ76905_writeSettings(BQ76905_HandleTypeDef *hbq)
{
	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	// hardware and feature configuration ----------------------------
	// set the battery cell count
	i2c_read_result = BQ76905_WriteDataMemory(hbq, BQ76905_V_CELL_MODE, &hbq->hardwareSettings.seriesCellCount, 1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	// set the cell balancing register based upon seriesCellCount and cellBalancingEnabled
	if (hbq->hardwareSettings.cellBalancingEnabled == true)
	{
		// create a value where the bit position equal to the seriesCellCount is equal to 1
		uint16_t cell_balance_register_value = (0x0001 << (hbq->hardwareSettings.seriesCellCount));
		// subtract 1 from the value.  this will result in a value where all bit
		// positions from bit 0 to the bit position equal to one less then the cell count is equal to 1.
		cell_balance_register_value = cell_balance_register_value - 1;
		// shift the value over by 1 bit position since bit 0 is a reserved bit
		cell_balance_register_value = cell_balance_register_value << 1;

		// send the cell balance data to the battery monitor using the cell balance active cells subcommand
		i2c_read_result = BQ76905_WriteSubcommand(hbq, BQ76905_SUBCOMMAND_CELL_BALANCING, (uint8_t*)&cell_balance_register_value, 1);
		if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	}

	// set the protections to have enabled ----------------------------
	//construct the setting bytes based upon user configuration
	uint8_t protections_a = 0 << 7; //((uint8_t)hbq->protectionFeatures.cellOvervoltage << 7);
	protections_a |= 0 << 6; //((uint8_t)hbq->protectionFeatures.cellUndervoltage << 6);
	protections_a |= ((uint8_t)hbq->protectionFeatures.dischargeShortCircuit << 5);
	protections_a |= ((uint8_t)hbq->protectionFeatures.dischargeOvercurrent1 << 4);
	protections_a |= ((uint8_t)hbq->protectionFeatures.dischargeOvercurrent2 << 3);
	protections_a |= ((uint8_t)hbq->protectionFeatures.chargeOvercurrent << 2);
	protections_a |= (uint8_t)(hbq->protectionFeatures.currentFaultLatching << 1);
	protections_a |= (uint8_t)hbq->protectionFeatures.regout;
	uint8_t protections_b = ((uint8_t)hbq->protectionFeatures.dischargeOvertemperature << 5);
	protections_b |= ((uint8_t)hbq->protectionFeatures.chargeOvertemperature << 4);
	protections_b |= ((uint8_t)hbq->protectionFeatures.dischargeUndertemperature << 3);
	protections_b |= ((uint8_t)hbq->protectionFeatures.chargeUndertemperature << 2);
	protections_b |= ((uint8_t)hbq->protectionFeatures.internalOvertemperature << 1);
	protections_b |= (uint8_t)hbq->protectionFeatures.hostWatchdog;
	uint8_t protections_dsg = ((uint8_t)hbq->protectionFeatures.cellUndervoltage << 7);
	protections_dsg |= ((uint8_t)hbq->protectionFeatures.dischargeShortCircuit << 6);
	protections_dsg |= ((uint8_t)hbq->protectionFeatures.dischargeOvercurrent1 << 5);
	protections_dsg |= ((uint8_t)hbq->protectionFeatures.dischargeOvercurrent2 << 4);
	protections_dsg |= ((uint8_t)hbq->protectionFeatures.hostWatchdog << 3);
	protections_dsg |= ((uint8_t)hbq->protectionFeatures.dischargeOvertemperature << 2);
	protections_dsg |= ((uint8_t)hbq->protectionFeatures.dischargeUndertemperature << 1);
	protections_dsg |= (uint8_t)hbq->protectionFeatures.internalOvertemperature;
	uint8_t protections_chg = ((uint8_t)hbq->protectionFeatures.cellOvervoltage << 7);
	protections_chg |= ((uint8_t)hbq->protectionFeatures.dischargeShortCircuit << 6);
	protections_chg |= ((uint8_t)hbq->protectionFeatures.chargeOvercurrent << 5);
	protections_chg |= ((uint8_t)hbq->protectionFeatures.hostWatchdog << 3);
	protections_chg |= ((uint8_t)hbq->protectionFeatures.chargeOvertemperature << 2);
	protections_chg |= ((uint8_t)hbq->protectionFeatures.chargeUndertemperature << 1);
	protections_chg |= (uint8_t)hbq->protectionFeatures.internalOvertemperature;
	uint8_t protections_both = ((uint8_t)hbq->protectionFeatures.vrefMeasurement << 2);
	protections_both |= ((uint8_t)hbq->protectionFeatures.vssMeasurement << 1);
	protections_both = (uint8_t)hbq->protectionFeatures.regout;
	// send the configs on the battery monitor
	i2c_read_result = BQ76905_WriteDataMemory(hbq, BQ76905_ENABLED_PROTECTIONS_A, &protections_a, 1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_WriteDataMemory(hbq, BQ76905_ENABLED_PROTECTIONS_B, &protections_b, 1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_WriteDataMemory(hbq, BQ76905_DSG_FET_PROTECTIONS, &protections_dsg, 1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_WriteDataMemory(hbq, BQ76905_CHG_FET_PROTECTIONS, &protections_chg, 1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }
	i2c_read_result = BQ76905_WriteDataMemory(hbq, BQ76905_BOTH_FET_PROTECTIONS, &protections_both, 1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// set the protection thresholds -----------------------------------
	// TODO: finish adding all protection threshold features
	// update the discharge short circuit protection value
	// TODO: create defines for what values map to in current sense mV, or even better would be to map from current in amps based upon gain settings
	i2c_read_result = BQ76905_WriteDataMemory(hbq, BQ76905_DSG_SHORT_CIRCUIT_THRESHOLD, &hbq->protectionThresholds.shortCircuitThreshold, 1);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// set the battery monitor calibration values ----------------------
	// TODO: add in battery calibration value settings

	// return the successful result ------------------------------------
	return i2c_read_result;
}


HAL_StatusTypeDef BQ76905_ClearFaults(BQ76905_HandleTypeDef *hbq)
{
	uint8_t clearFaultMask = 0xFF;
	return BQ76905_WriteSubcommand(hbq, BQ76905_SUBCOMMAND_PROT_RECOVERY, &clearFaultMask, 1);
}


HAL_StatusTypeDef BQ76905_WakeUp(BQ76905_HandleTypeDef *hbq)
{
	return BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_SLEEP_DISABLE);
}


HAL_StatusTypeDef BQ76905_Deepsleep(BQ76905_HandleTypeDef *hbq)
{
	// set the cell balancing to 0
	uint8_t cb = 0;
	BQ76905_WriteSubcommand(hbq, BQ76905_SUBCOMMAND_CELL_BALANCING, &cb, 1);
	HAL_Delay(3);
	BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_DEEPSLEEP_ENABLE);
	HAL_Delay(2);
	return BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_DEEPSLEEP_ENABLE);
}


HAL_StatusTypeDef BQ76905_ExitDeepsleep(BQ76905_HandleTypeDef *hbq)
{
	return BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_DEEPSLEEP_DISABLE);
}


HAL_StatusTypeDef BQ76905_Sleep(BQ76905_HandleTypeDef *hbq)
{
	return BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_SLEEP_ENABLE);
}


HAL_StatusTypeDef BQ76905_FET_EN(BQ76905_HandleTypeDef *hbq, bool state)
{
	if (hbq->systemStatus.fetEnable != state)
	{
		return BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_FET_ENABLE);
	}
	return HAL_OK;
}


HAL_StatusTypeDef BQ76905_WriteRegister(BQ76905_HandleTypeDef *hbq, uint8_t reg, uint16_t data)
{

	// combine the register address and the data into one array
	uint8_t buffer[3] = {reg, (data & 0xFF), ((data >> 8) & 0xFF)};

	// send the 16 bit command
	return HAL_I2C_Master_Transmit(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, (uint8_t*)(&buffer), 3, 100);
}


HAL_StatusTypeDef BQ76905_ReadRegister(BQ76905_HandleTypeDef *hbq, uint8_t reg, uint16_t *data) {

	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	i2c_read_result = HAL_I2C_Master_Transmit(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, &reg, 1, 100);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	i2c_read_result = HAL_I2C_Master_Receive(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, (uint8_t*)data, 2, 300);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// return the successful result ------------------------------------
	return i2c_read_result;
}


HAL_StatusTypeDef BQ76905_WriteSubcommand(BQ76905_HandleTypeDef *hbq, uint16_t address, uint8_t  *data, uint8_t len)
{
	// cap the length to the max buffer size of 32
	if (len > 32) { len = 32; }

	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	// write the 16-bit subcommand address into the subcommand register
	i2c_read_result = HAL_I2C_Mem_Write(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, BQ76905_SUBCOMMAND_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(&address), 2, 100);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// write the data to the transfer buffer
	i2c_read_result = HAL_I2C_Mem_Write(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, BQ76905_SUBCOMMAND_BUFFER_REG, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// calculate the checksum value (8-bit modulo-256 sum and then invert all the bits of the result)
	uint8_t checksum = (uint8_t)(0xFF & address);
	checksum += (uint8_t )((address >> 8) & 0xFF);
	for (uint8_t i = 0; i < len; i++)
	{
		checksum += *(data+i);
	}
	checksum = ~checksum;

	// write the checksum into the checksum register
	i2c_read_result = HAL_I2C_Mem_Write(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, BQ76905_SUBCOMMAND_CHECKSUM_REG, I2C_MEMADD_SIZE_8BIT, &checksum, 1, 100);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// write the length into the length register
	i2c_read_result = HAL_I2C_Mem_Write(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, BQ76905_SUBCOMMAND_LENGTH_REG, I2C_MEMADD_SIZE_8BIT, &len+4, 1, 100);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// return the successful result ------------------------------------
	return i2c_read_result;
}


HAL_StatusTypeDef BQ76905_ReadSubcommand(BQ76905_HandleTypeDef *hbq, uint16_t address, uint8_t *data, uint8_t len)
{
	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	// write the 16-bit subcommand address into the subcommand register
	i2c_read_result = HAL_I2C_Mem_Write(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, BQ76905_SUBCOMMAND_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(&address), 2, 100);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// read the data in the transfer buffer
	HAL_I2C_Mem_Read(hbq->hardwareSettings.hi2c, BQ76905_I2C_ADDR, BQ76905_SUBCOMMAND_REG, I2C_MEMADD_SIZE_8BIT, data, len, 100);

	// return the successful result ------------------------------------
	return i2c_read_result;
}


HAL_StatusTypeDef BQ76905_WriteDataMemory(BQ76905_HandleTypeDef *hbq, uint16_t address, uint8_t  *data, uint8_t len)
{

	// holds the success/fault result of the i2c transaction.
	HAL_StatusTypeDef i2c_read_result;

	// disable fets
	i2c_read_result = BQ76905_FET_EN(hbq, false);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// send the config mode command
	i2c_read_result = BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_ENTER_CFG);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// update memory setting
	i2c_read_result = BQ76905_WriteSubcommand(hbq, address, data, len);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// exit config mode
	i2c_read_result = BQ76905_WriteRegister(hbq, BQ76905_SUBCOMMAND_REG, BQ76905_SUBCOMMAND_EXIT_CFG);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// re-enable the FET's
	i2c_read_result = BQ76905_FET_EN(hbq, true);
	if (i2c_read_result != HAL_OK) { return i2c_read_result; }

	// return the successful result ------------------------------------
	return i2c_read_result;
}




