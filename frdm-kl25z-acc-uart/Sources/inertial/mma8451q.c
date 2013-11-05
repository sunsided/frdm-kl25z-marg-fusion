/*
 * mma8451q.c
 *
 *  Created on: Nov 5, 2013
 *      Author: Markus
 */

#include "derivative.h"
#include <stdint.h>
#include "i2c/i2c.h"
#include "inertial/mma8451q.h"

/**
 * @brief WHO_A_I register for device identification
 */
#define MMA8451Q_REG_WHOAMI	(0x0D)

/**
 * @brief Reads the WHO_AM_I register from the MMA8451Q.
 * @return Device identification code; Should be 0b00011010. 
 */
uint8_t MMA8451Q_WhoAmI()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_WHOAMI);
}
