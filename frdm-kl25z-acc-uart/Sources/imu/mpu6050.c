/*
 * mpu6050.c
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#include "imu/mpu6050.h"

/**
 * @brief Reads the WHO_AM_I register from the MPU6050.
 * @return Device identification code; Should be 0b0110100 (0x68)
 */
uint8_t MPU6050_WhoAmI()
{
	return I2C_ReadRegister(MPU6050_I2CADDR, MPU6050_REG_WHO_AM_I);
}
