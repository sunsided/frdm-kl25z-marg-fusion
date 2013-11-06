/*
 * i2c.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#ifndef I2C_H_
#define I2C_H_

#include "derivative.h"

/**
 * @brief Encodes the read address from the 7-bit slave address
 */
#define I2C_READ_ADDRESS(slaveAddress) 		((uint8_t)((slaveAddress << 1) | 1))

/**
 * @brief Encodes the write address from the 7-bit slave address
 */
#define I2C_WRITE_ADDRESS(slaveAddress) 	((uint8_t)((slaveAddress << 1) | 0))

/**
 * @brief A mask that describes a no-or modify operation
 */
#define I2C_MOD_NO_OR_MASK	(0x0)

/**
 * @brief A mask that describes a no-and modify operation
 */
#define I2C_MOD_NO_AND_MASK	(~0x0)

/**
 * @brief Initializes the SPI interface
 */
void InitI2C();

/**
 * @brief Reads an 8-bit register from an I2C slave 
 * @param[in] slaveId The device's I2C slave id
 * @param[in] registerAddress Address of the device register to read
 * @return The value at the register
 */
uint8_t I2C_ReadRegister(register uint8_t slaveId, register uint8_t registerAddress);


/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be larger than zero.
 * @param[out] buffere The buffer to write into
 */
void I2C_ReadRegisters(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, uint8_t *buffer);

/**
 * @brief Writes an 8-bit value to an 8-bit register on an I2C slave
 * @param[in] slaveId The device's I2C slave id
 * @param[in] registerAddress Address of the device register to read
 * @param[in] value The value to write
 */
void I2C_WriteRegister(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t value);

/**
 * @brief Reads an 8-bit register from an I2C slave, modifies it and writes it back
 * @param[in] slaveId The slave id
 * @param[in] registerAddress The register to modify
 * @param[in] orMask The mask to OR the register with
 * @param[in] andMask The mask to AND the register with
 * @return The register after modification
 */
uint8_t I2C_ModifyRegister(register uint8_t slaveId, uint8_t register registerAddress, register uint8_t orMask, register uint8_t andMask);

#endif /* I2C_H_ */
