/*
 * i2c.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#ifndef I2C_H_
#define I2C_H_

#include "derivative.h"

#define I2C_READ_ADDRESS(slaveAddress) 		((uint8_t)((slaveAddress << 1) | 1))
#define I2C_WRITE_ADDRESS(slaveAddress) 	((uint8_t)((slaveAddress << 1) | 0))

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
uint8_t I2C_ReadRegister(uint8_t slaveId, uint8_t registerAddress);

#endif /* I2C_H_ */
