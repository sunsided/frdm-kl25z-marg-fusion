/*
 * mma8451q.c
 *
 *  Created on: Nov 5, 2013
 *      Author: Markus
 */

#include "endian.h"
#include "inertial/mma8451q.h"

#define CTRL_REG1_DR_MASK 		0x38
#define CTRL_REG1_DR_SHIFT 		0x3
#define CTRL_REG1_LNOISE_MASK 	0x3
#define CTRL_REG1_LNOISE_SHIFT 	0x2

/**
 * @brief Reads the accelerometer data in 14bit no-fifo mode
 * @param[out] The accelerometer data; Must not be null. 
 */
void MMA8451Q_ReadAcceleration14bitNoFifo(mma8451q_acc_t *const data)
{
	/* in 14bit mode there are 7 registers to be read (1 status + 6 data) */
	static const uint8_t registerCount = 7;
	
	/* address the buffer by skipping the padding field */
	uint8_t *buffer = &data->status;
	
	/* read the register data */
	I2C_ReadRegisters(MMA8451Q_I2CADDR, MMA8451Q_REG_STATUS, registerCount, buffer);
	
	/* apply fix for endianness */
	if (endianCorrectionRequired(FROM_BIG_ENDIAN))
	{
		data->x = ENDIANSWAP_16(data->x);
		data->y = ENDIANSWAP_16(data->y);
		data->z = ENDIANSWAP_16(data->z);
	}
	
	/* correct the 14bit layout to 16bit layout */
	data->x >>= 2;
	data->y >>= 2;
	data->z >>= 2;
}

/**
 * @brief Sets the data rate and the active mode
 */
void MMA8451Q_SetDataRate(register mma8451q_datarate_t datarate, register mma8451q_lownoise_t lownoise)
{
	const register uint8_t mask = ~(CTRL_REG1_DR_MASK | CTRL_REG1_LNOISE_MASK);
	const register uint8_t value = ((datarate << CTRL_REG1_DR_SHIFT) & CTRL_REG1_DR_MASK) | ((lownoise << CTRL_REG1_LNOISE_SHIFT) & CTRL_REG1_LNOISE_MASK);
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG1, mask, value);
}

/**
 * @brief Configures the sensitivity and the high pass filter
 * @param[in] sensitivity The sensitivity
 * @param[in] highpassEnabled Set to 1 to enable the high pass filter or to 0 otherwise (default)
 */
void MMA8451Q_SetSensitivity(register mma8451q_sensitivity_t sensitivity, register mma8451q_hpo_t highpassEnabled)
{
	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_XZY_DATA_CFG, (sensitivity & 0x03) | ((highpassEnabled << 4) & 0x10));
}

/**
 * @brief Reads the SYSMOD register from the MMA8451Q.
 * @return Current system mode. 
 */
uint8_t MMA8451Q_SystemMode()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_SYSMOD);
}

/**
 * @brief Reads the REG_PL_CFG register from the MMA8451Q.
 * @return Current portrait/landscape mode. 
 */
uint8_t MMA8451Q_LandscapePortraitConfig()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_PL_CFG);
}
