#ifndef __BQ76905_H__
#define __BQ76905_H__

#include "main.h"
#include <stdbool.h>

// device i2c address ----------------------------------------
#define BQ76905_I2C_ADDR       			0x10

// system status register and bit mask values -----------------
#define BQ76905_BATTERY_STATUS_REG  	0x12
#define BQ76905_SLEEP_BIT				(1 << 15)		// 1 : device is in SLEEP mode.  0 : device is not in SLEEP mode.
#define BQ76905_DEEPSLEEP_BIT			(1 << 14)		// 1 : . 0 : .
#define BQ76905_SA_BIT					(1 << 13)		// 1 : . 0 : .
#define BQ76905_SS_BIT					(1 << 12)		// 1 : . 0 : .
#define BQ76905_ACCCESS_BITS			(3 << 10)
#define BQ76905_UNINITIALIZED_MODE_BITS	(0 << 10)		// 1 : . 0 : .
#define BQ76905_FULLACCESS_MODE_BITS	(1 << 10)		// 1 : . 0 : .
#define BQ76905_SEALED_MODE_BITS		(3 << 10)		// 1 : . 0 : .
#define BQ76905_FET_EN_BIT				(1 << 8)		// 1 : . 0 : .
#define BQ76905_POR_BIT					(1 << 7)		// 1 : device was reset and should be reconfigured.
#define BQ76905_SLEEP_EN_BIT			(1 << 6)		// 1 : device is allowed to enter sleep mode.
#define BQ76905_CFGUPDATE_BIT			(1 << 5)		// 1 : device is in CONFIG_UPDATE mode.
#define BQ76905_ALERTPIN_BIT			(1 << 4)		// 1 : alert pin is asserted.
#define BQ76905_CHG_BIT					(1 << 3)		// 1 : charge gate driver is enabled.
#define BQ76905_DIS_BIT					(1 << 2)		// 1 : discharge gate driver is enabled.
#define BQ76905_CHGDET_FLAG_BIT			(1 << 1)		// 1 : charge detector de-bounce signal is high.


// battery pack measurement registers -------------------------
#define BQ76905_PACK_VOLTAGE_REG 	0x26
#define BQ76905_CELL1_VOLT_REG 		0x14
#define BQ76905_CELL2_VOLT_REG		0x16
#define BQ76905_CELL3_VOLT_REG 		0x18
#define BQ76905_CELL4_VOLT_REG 		0x1A
#define BQ76905_CELL5_VOLT_REG		0x1C
#define BQ76905_RAW_CURRENT_REG 	0x36
#define BQ76905_CC1_REG 			0x3C
#define BQ76905_CC2_REG 			0x3A
#define BQ76905_INTERNAL_TEMP_REG	0x28
#define BQ76905_TS_TEMP_REG			0x2A

// safety alert/status addresses and bitmasks -----------------
#define BQ76905_SAFETY_ALERT_A_REG		0x02
#define BQ76905_BIT						0x2A
#define BQ76905_SAFETY_ALERT_B_REG		0x04

// direct command addresses -----------------------------------
#define BQ76905_FET_CONTROL_REG			0x68				// when in manual fet control, sets the CHG and DSG FET state
// fault clearing address and bitmask
#define BQ76905_SUBCOMMAND_PROT_RECOVERY	0x0022
#define BQ76905_VOLTREC_BIT					(1 << 7)		// cell overvoltage/undervoltage recovery
#define BQ76905_DIAGREC_BIT					(1 << 6)		// VSS or VREF fault recovery
#define BQ76905_SDCREC_BIT					(1 << 5)		// short Circuit Discharge fault recovery
#define BQ76905_OCD1REC_BIT					(1 << 4)		// discharge level 1 overcurrent recovery
#define BQ76905_OCD2REC_BIT					(1 << 3)		// discharge level 2 overcurrent recovery
#define BQ76905_OCCREC_BIT					(1 << 2)		// charge overcurrent recovery
#define BQ76905_TEMPPREC_BIT				(1 << 1)		// temperature fault recover

// subcommand execution addresses -----------------------------
#define BQ76905_SUBCOMMAND_REG				0x3E			// address to send subcommands to
#define BQ76905_SUBCOMMAND_BUFFER_REG		0x40			// starting address (0x40 to 0x5F) of the subcommand data buffer
#define BQ76905_SUBCOMMAND_CHECKSUM_REG 	0x60			// checksum register for when issuing subcommands with data
#define BQ76905_SUBCOMMAND_LENGTH_REG		0x61			// data length for when issuing subcommands with data

// subcommand addresses and bitmasks --------------------------
#define BQ76905_SUBCOMMAND_FET_ENABLE		0x0022			// toggles between autonomous and manual fet control
#define BQ76905_SUBCOMMAND_CELL_BALANCING	0x0083			// configures which cells are being actively balanced
#define BQ76905_SUBCOMMAND_ENTER_CFG		0x0090			// places the battery monitor in UPDATE_CONFIG mode
#define BQ76905_SUBCOMMAND_EXIT_CFG			0x0092			// takes the device out of UPDATE_CONFIG mode
#define BQ76905_SUBCOMMAND_SLEEP_DISABLE 	0x009A			// takes the battery monitor out of sleep mode
#define BQ76905_SUBCOMMAND_SLEEP_ENABLE 	0x0099			// places the battery monitor into sleep mode
#define BQ76905_SUBCOMMAND_DEEPSLEEP_DISABLE	0x000F			// takes the battery monitor out of sleep mode
#define BQ76905_SUBCOMMAND_DEEPSLEEP_ENABLE 0x000E			// places the battery monitor into sleep mode

// data memory addresses and bitmasks -------------------
#define BQ76905_CELL_1_GAIN					0x9000
#define BQ76905_PACK_VOLTAGE_GAIN			0x9002
#define BQ76905_CELL_2_DELTA_GAIN			0x9004
#define BQ76905_CELL_3_DELTA_GAIN			0x9005
#define BQ76905_CELL_4_5_DELTA_GAIN			0x9071
#define BQ76905_CELL_6_7_DELTA_GAIN			0x9072
#define BQ76905_CURRENT_GAIN				0x9006
#define BQ76905_CURRENT_OFFSET				0x9008
#define BQ76905_CC1_GAIN					0x900A
#define BQ76905_CC1_OFFSET					0x9008
#define BQ76905_TS_OFFSET					0x900E
#define BQ76905_INTERNAL_TEMP_GAIN			0x9010
#define BQ76905_INTERNAL_TEMP_OFFSET		0x9012
#define BQ76905_V_CELL_MODE					0x901B			// configure the battery pack's series cell count
#define BQ76905_FET_OPTIONS					0x901E
#define BQ76905_BALANCING_CONFIGURATION		0x9020
#define BQ76905_MIN_TS_TEMP_THRESHOLD		0x9021
#define BQ76905_MAX_TS_TEMP_THRESHOLD		0x9022
#define BQ76905_MAX_INTERNAL_TEMP_THRESHOLD 0x9023
#define BQ76905_ENABLED_PROTECTIONS_A		0x9024
#define BQ76905_ENABLED_PROTECTIONS_B		0x9025
#define BQ76905_DSG_FET_PROTECTIONS			0x9026
#define BQ76905_CHG_FET_PROTECTIONS			0x9027
#define BQ76905_BOTH_FET_PROTECTIONS		0x9028
#define BQ76905_CELL_UNDERVOLTAGE_THRESHOLD 0x902E
#define BQ76905_CELL_OVERVOLTAGE_THRESHOLD	0x9032
#define BQ76905_CHG_OVERCURRENT_THRESHOLD	0x9036
#define BQ76905_DSG_OVERCURRENT_1_THRESHOLD 0x9038
#define BQ76905_DSG_OVERCURRENT_2_THRESHOLD	0x903A
#define BQ76905_DSG_SHORT_CIRCUIT_THRESHOLD	0x903C
#define BQ76905_AUTO_RESET_LATCH_LIMIT		0x903E
#define BQ76905_CHG_OVERTEMP_THRESHOLD		0x9040
#define BQ76905_CHG_UNDERTEMP_THRESHOLD		0x9043
#define BQ76905_DSG_OVERTEMP_THRESHOLD		0x9046
#define BQ76905_DSG_UNDERTEMP_THRESHOLD		0x9049
#define BQ76905_INT_OVERTEMP_THRESHOLD		0x904D
#define BQ76905_INT_OVERTEMP_REC_THRESHOLD 	0x904E
#define BQ76905_SHUTDOWN_CELL_VOLTAGE		0x9053
#define BQ76905_SHUTDOWN_PACK_VOLTAGE		0x9055
#define BQ76905_SHUTDOWN_TEMP				0x9057


// protection threshold addresses and bitmasks ----------------
#define BQ76905_SCD_THRESHOLD 0x903C

typedef struct {
    bool sleep;
    bool deepsleep;
    bool sa;
    bool ss;
    bool uninitializedMode;
    bool fullAccessMode;
    bool sealedMode;
    bool fetEnable;
    bool por;
    bool sleepEnabled;
    bool cfgUpdateMode;
    bool alertPin;
    bool chargeActive;
    bool dischargeActive;
    bool chargerDetected;
} BQ76905_StatusTypeDef;

typedef struct {
    uint16_t packVoltage;
    uint16_t cellVoltage_1;
    uint16_t cellVoltage_2;
    uint16_t cellVoltage_3;
    uint16_t cellVoltage_4;
    uint16_t cellVoltage_5;
    uint16_t cellVoltage_6;
    uint16_t cellVoltage_7;
    int16_t packCurrent;
    uint16_t internalTemperature;
    uint16_t externalTemperature;
} BQ76905_MeasurementTypeDef;

typedef struct {
    bool cellOvervoltage;
    bool cellUndervoltage;
    bool dischargeShortCircuit;
    bool dischargeOvercurrent_1;
    bool dischargeOvercurrent_2;
    bool chargeOvercurrent;
    bool currentProtectionLatch;
    bool regout;
    bool dischargeOvertemperature;
    bool chargeOvertemperature;
    bool dischargeUndertemperature;
    bool chargeUndertemperature;
    bool internalOvertemperature;
    bool hostWatchdog;
    bool vrefMeasurement;
    bool vssMeasurement;
} BQ76905_SafetyStatusTypeDef;

typedef struct {
    bool cellOvervoltage;
    bool cellUndervoltage;
    bool dischargeShortCircuit;
    bool dischargeOvercurrent_1;
    bool dischargeOvercurrent_2;
    bool chargeOvercurrent;
    bool dischargeOvertemperature;
    bool chargeOvertemperature;
    bool dischargeUndertemperature;
    bool chargeUndertemperature;
    bool internalOvertemperature;
    bool hostWatchdog;
    bool vrefMeasurement;
    bool vssMeasurement;
} BQ76905_SafetyAlertTypeDef;

typedef struct {
    bool cellOvervoltage;
    bool cellUndervoltage;
    bool dischargeShortCircuit;
    bool dischargeOvercurrent1;
    bool dischargeOvercurrent2;
    bool chargeOvercurrent;
    bool currentFaultLatching;
    bool regout;
    bool dischargeOvertemperature;
    bool dischargeUndertemperature;
    bool chargeOvertemperature;
    bool chargeUndertemperature;
    bool internalOvertemperature;
    bool hostWatchdog;
    bool vrefMeasurement;
    bool vssMeasurement;
} BQ76905_ProtectionsEnabledTypeDef;

typedef struct {
	// protection feature settings
	uint8_t maxAutoRecoveryAttempts;

	// voltage fault thresholds
    int16_t cellUnderVoltageThreshold;
    int16_t cellOverVoltageThreshold;

    // current fault thresholds
    uint8_t shortCircuitThreshold;
    uint8_t dischargeOvercurrent1Threshold;
    uint8_t dischargeOvercurrent2Threshold;
    uint8_t chargeOvercurrentThreshold;

    // external temperature fault thresholds
    uint8_t chargeOvertemperatureThreshold;
    uint8_t chargeOvertemperatureRecoveryThreshold;
    uint8_t chargeUndertemperatureThreshold;
    uint8_t chargeUndertemperatureRecoveryThreshold;
    uint8_t dischargeOvertemperatureThreshold;
	uint8_t dischargeOvertemperatureRecoveryThreshold;
	uint8_t dischargeUndertemperatureThreshold;
	uint8_t dischargeUndertemperatureRecoveryThreshold;
} BQ76905_ProtectionThresholdsTypeDef;

typedef struct {
	// current sensing calibration settings
    uint16_t currentGain;
    uint16_t currentOffset;
    uint16_t CC1Gain;
    uint16_t CC1Offset;

    // temperature sensing calibration settings
    uint16_t externalTemperatureGain;
    uint16_t externalTemperatureOffset;
    uint16_t internalTemperatureGain;
    uint16_t internalTemperatureOffset;

    // voltage sensing calibration settings
    uint16_t packVoltageGain;
    uint16_t cell1VoltageGain;
    uint8_t cell2DeltaVoltageGain;
    uint8_t cell3DeltaVoltageGain;
    uint8_t cell4And5DeltaVoltageGain;
    uint8_t cell6And7DeltaVoltageGain;
} BQ76905_MeasurementCalibrationTypeDef;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	GPIO_TypeDef *TSPort;
	uint16_t TSPin;
	uint8_t seriesCellCount;
	bool cellBalancingEnabled;
	bool externalTemperatureSensorEnabled;
} BQ76905_HardwareSettingsTypeDef;

typedef struct {

    // battery monitor status values
    BQ76905_StatusTypeDef systemStatus;
    BQ76905_MeasurementTypeDef measurements;
    BQ76905_SafetyAlertTypeDef faultAlerts;
    BQ76905_SafetyStatusTypeDef faultStatus;

    // user configured battery monitor settings
	BQ76905_HardwareSettingsTypeDef hardwareSettings;					// host and battery monitor hardware settings
	BQ76905_ProtectionsEnabledTypeDef protectionFeatures; 				// safety protection enabled settings
	BQ76905_ProtectionThresholdsTypeDef protectionThresholds;			// safety protection thresholds
	BQ76905_MeasurementCalibrationTypeDef calibrationValues;			// measurement calibration values

} BQ76905_HandleTypeDef;

// commands for updating battery monitor status
HAL_StatusTypeDef BQ76905_fetchSystemStatus(BQ76905_HandleTypeDef *hbq);
HAL_StatusTypeDef BQ76905_fetchSafetyStatus(BQ76905_HandleTypeDef *hbq);
HAL_StatusTypeDef BQ76905_fetchMeasurements(BQ76905_HandleTypeDef *hbq);
HAL_StatusTypeDef BQ76905_writeSettings(BQ76905_HandleTypeDef *hbq);

// commands for configuring the battery monitor
HAL_StatusTypeDef BQ76905_ClearFaults(BQ76905_HandleTypeDef *hbq);
HAL_StatusTypeDef BQ76905_WakeUp(BQ76905_HandleTypeDef *hbq);
HAL_StatusTypeDef BQ76905_Sleep(BQ76905_HandleTypeDef *hbq);
HAL_StatusTypeDef BQ76905_FET_EN(BQ76905_HandleTypeDef *hbq, bool);

#endif /* __BQ76905_H__ */
