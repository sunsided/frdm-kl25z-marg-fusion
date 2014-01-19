/*
 * hmc5883l.c
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#include "imu/hmc5883l.h"
#include "endian.h"
#include "i2c/i2c.h"

/**
 * @brief Helper macro to set bits in configuration->REGISTER_NAME
 * @param[in] reg The register name
 * @param[in] bits The name of the bit to set
 * @param[in] value The value to set 
 * 
 * Requires the macros HMC5883L_regname_bitname_MASK and
 * HMC5883L_regname_bitname_SHIFT to be set at expansion time. 
 */
#define HMC5883L_CONFIG_SET(reg, bits, value) \
	configuration->reg &= (uint8_t)~(HMC5883L_ ## reg ## _ ## bits ## _MASK); \
	configuration->reg |= (value << HMC5883L_## reg ## _ ## bits ## _SHIFT) & HMC5883L_ ## reg ## _ ## bits ## _MASK

/**
 * @brief Reads the Identification registers from the HMC5883L.
 * @return Device identification code; Should be 0x00483433 ('\0H43' sequential Memory!)
 */
uint32_t HMC5883L_Identification()
{
	uint32_t result = 0;
	uint8_t* ptr = (uint8_t*)&result + 1;
	I2C_ReadRegisters(HMC5883L_I2CADDR, HMC5883L_REG_IRA, 3, ptr);
	return endianCorrect32(result, FROM_BIG_ENDIAN);
}

/**
 * @brief Fetches the data from the HMC5883L
 * @param[inout] data The sensor data
 */
void HMC5883L_ReadData(hmc5883l_data_t *const data)
{
	assert_not_null(data);
	register uint8_t bufferA, bufferB;
	
	/* fetch the data */
#if 0
	I2C_SendStart();
	I2C_InitiateRegisterReadAt(HMC5883L_I2CADDR, HMC5883L_REG_SR);
	I2C_DisableAck();
	bufferA = I2C_ReceiveAndStop();
	if ((bufferA & HMC5883L_SR_LOCK_MASK) || !(bufferA & HMC5883L_SR_RDY_MASK))
	{
		/* data is either locked - a write process is in process -
		 * or not ready. 
		 */
		data->status = bufferA;
		return;
	}
#endif
	/* TODO: find a way to keep the wire here using repeated start */
	
	/* if there is data available and no lock, read data */
	I2C_SendStart();
	I2C_InitiateRegisterReadAt(HMC5883L_I2CADDR, HMC5883L_REG_DXRA);
	
	/* read x */
	bufferA = I2C_ReceiveDriving();
	bufferB = I2C_ReceiveDriving();
	data->x = (int16_t)(((bufferA << 8) & 0xFF00) | ((bufferB) & 0x00FF));
		
	/* read z - note that this is not a bug but the 
    * actual ordering of the sensor data registers */
    bufferA = I2C_ReceiveDriving();
    bufferB = I2C_ReceiveDriving();
	data->z = (int16_t)(((bufferA << 8) & 0xFF00) | ((bufferB) & 0x00FF));

    /* read y */
    bufferA = I2C_ReceiveDrivingWithNack();
    bufferB = I2C_ReceiveAndStop();
    data->y = (int16_t)(((bufferA << 8) & 0xFF00) | ((bufferB)& 0x00FF));
}

/**
 * @brief Fetches the HMC5883L configuration
 * @param[inout] configuration The configuration
 */
void HMC5883L_FetchConfiguration(hmc5883l_confreg_t *const configuration)
{
	assert_not_null(configuration);
	
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();
	
	/* start register addressing */
	I2C_SendStart();
	I2C_InitiateRegisterReadAt(HMC5883L_I2CADDR, HMC5883L_REG_CRA);
	configuration->CRA = I2C_ReceiveDriving();
	configuration->CRB = I2C_ReceiveDrivingWithNack();
	configuration->MR = I2C_ReceiveAndStop();
}

/**
 * @brief Stores the HMC5883L configuration
 * @param[in] configuration The configuration
 */
void HMC5883L_StoreConfiguration(const hmc5883l_confreg_t *const configuration)
{
	assert_not_null(configuration);
	
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();
	
	/* start register addressing */
	I2C_SendStart();
	I2C_SendBlocking(I2C_WRITE_ADDRESS(HMC5883L_I2CADDR));
	I2C_SendBlocking(HMC5883L_REG_CRA);
	I2C_SendBlocking(configuration->CRA);
	I2C_SendBlocking(configuration->CRB);
	I2C_SendBlocking(configuration->MR);
	I2C_SendStop();
}

/**
 * Sets the averaging mode
 * @param[inout] configuration The configuration
 * @param[in] averaging The averaging mode
 */
void HMC5883L_SetAveraging(hmc5883l_confreg_t *const configuration, register hmc5883l_ma_t averaging)
{
	assert_not_null(configuration);
	HMC5883L_CONFIG_SET(CRA, MA, averaging);
}

/**
 * Sets the output rate
 * @param[inout] configuration The configuration
 * @param[in] rate The output rate
 */
void HMC5883L_SetOutputRate(hmc5883l_confreg_t *const configuration, register hmc5883l_do_t rate)
{
	assert_not_null(configuration);
	HMC5883L_CONFIG_SET(CRA, DO, rate);
}

/**
 * Sets the measurement mode
 * @param[inout] configuration The configuration
 * @param[in] mode The measurement mode
 */
void HMC5883L_SetMeasurementMode(hmc5883l_confreg_t *const configuration, register hmc5883l_ms_t mode)
{
	assert_not_null(configuration);
	HMC5883L_CONFIG_SET(CRA, MS, mode);
}

/**
 * Sets the gain
 * @param[inout] configuration The configuration
 * @param[in] gain The sensor gain
 */
void HMC5883L_SetGain(hmc5883l_confreg_t *const configuration, register hmc5883l_gain_t gain)
{
	assert_not_null(configuration);
	HMC5883L_CONFIG_SET(CRB, GN, gain);
}
/**
 * Sets the operating mode
 * @param[inout] configuration The configuration
 * @param[in] mode The operating mode
 */
void HMC5883L_SetOperatingMode(hmc5883l_confreg_t *const configuration, register hmc5883l_mode_t mode)
{
	assert_not_null(configuration);
	HMC5883L_CONFIG_SET(MR, MD, mode);
}
