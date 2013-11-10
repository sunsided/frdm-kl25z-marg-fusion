/*
 * hmc5883l.c
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#include "imu/hmc5883l.h"
#include "endian.h"

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
