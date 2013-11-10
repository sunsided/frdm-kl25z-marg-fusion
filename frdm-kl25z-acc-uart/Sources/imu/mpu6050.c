/*
 * mpu6050.c
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#include "imu/mpu6050.h"
#include "i2c/i2c.h"

/**
 * @brief Reads the WHO_AM_I register from the MPU6050.
 * @return Device identification code; Should be 0b0110100 (0x68)
 */
uint8_t MPU6050_WhoAmI()
{
	return I2C_ReadRegister(MPU6050_I2CADDR, MPU6050_REG_WHO_AM_I);
}

/**
 * @brief Fetches the configuration into a {@see mpu6050_confreg_t} data structure
 * @param[inout] The configuration data data; Must not be null.
 */
void MPU6050_FetchConfiguration(mpu6050_confreg_t *const configuration)
{
	assert(configuration != 0x0);
		
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();
	
	/* start register addressing */
	I2C_SendStart();
	I2C_InitiateRegisterReadAt(MPU6050_I2CADDR, MPU6050_REG_SMPLRT_DIV);
	
	/* read the registers */
	configuration->SMPLRT_DIV = I2C_ReceiveDriving();
	configuration->CONFIG = I2C_ReceiveDriving();
	configuration->GYRO_CONFIG = I2C_ReceiveDrivingWithNack();
	configuration->ACCEL_CONFIG = I2C_ReceiveAndRestart();
	
	/* restart read at 0x23 */
	I2C_InitiateRegisterReadAt(MPU6050_I2CADDR, MPU6050_REG_FIFO_EN);
	configuration->FIFO_EN = I2C_ReceiveDriving();
	configuration->I2C_MST_CTRL = I2C_ReceiveDriving();
	configuration->I2C_SLV0_ADDR = I2C_ReceiveDriving();
	configuration->I2C_SLV0_CTRL = I2C_ReceiveDriving();
	configuration->I2C_SLV0_REG = I2C_ReceiveDriving();
	
	configuration->I2C_SLV1_ADDR = I2C_ReceiveDriving();
	configuration->I2C_SLV1_REG = I2C_ReceiveDriving();
	configuration->I2C_SLV1_CTRL = I2C_ReceiveDriving();
	configuration->I2C_SLV2_ADDR = I2C_ReceiveDriving();
	configuration->I2C_SLV2_REG = I2C_ReceiveDriving(); /* 2C */
	configuration->I2C_SLV2_CTRL = I2C_ReceiveDriving();
	configuration->I2C_SLV3_ADDR = I2C_ReceiveDriving();
	configuration->I2C_SLV3_REG = I2C_ReceiveDriving();
	configuration->I2C_SLV3_CTRL = I2C_ReceiveDriving();
	configuration->I2C_SLV4_ADDR = I2C_ReceiveDriving();
	configuration->I2C_SLV4_REG = I2C_ReceiveDriving();
	configuration->I2C_SLV4_DO = I2C_ReceiveDriving();
	configuration->I2C_SLV4_CTRL = I2C_ReceiveDriving();
	*(uint8_t*)&configuration->I2C_SLV4_DI = I2C_ReceiveDriving();
	*(uint8_t*)&configuration->I2C_MST_STATUS = I2C_ReceiveDriving();
	configuration->INT_PIN_CFG = I2C_ReceiveDrivingWithNack();
	configuration->INT_ENABLE = I2C_ReceiveAndRestart(); /* 0x38 */
	
	/* restart read at 0x63 */
	I2C_InitiateRegisterReadAt(MPU6050_I2CADDR, MPU6050_REG_I2C_SLV0_DO);
	configuration->I2C_SLV0_DO = I2C_ReceiveDriving();
	configuration->I2C_SLV1_DO = I2C_ReceiveDriving();
	configuration->I2C_SLV2_DO = I2C_ReceiveDriving();
	configuration->I2C_SLV3_DO = I2C_ReceiveDriving();
	configuration->I2C_MST_DELAY_CTRL = I2C_ReceiveDriving();
	configuration->SIGNAL_PATH_RESET = I2C_ReceiveDriving();
	configuration->MOT_DETECT_CTRL = I2C_ReceiveDriving();
	configuration->USER_CTRL = I2C_ReceiveDriving();
	configuration->PWR_MGMT_1 = I2C_ReceiveDrivingWithNack();
	configuration->PWR_MGMT_2 = I2C_ReceiveAndRestart();
	
	/* restart read at 0x6D */
	I2C_InitiateRegisterReadAt(MPU6050_I2CADDR, MPU6050_REG_FIFO_COUNTH);
	configuration->FIFO_COUNTH = I2C_ReceiveDriving();
	configuration->FIFO_COUNTL = I2C_ReceiveDriving();
	configuration->FIFO_R_W = I2C_ReceiveDrivingWithNack();
	*(uint8_t*)&configuration->WHO_AM_I = I2C_ReceiveAndStop();
}

/**
 * @brief Stores the configuration from a {@see mpu6050_confreg_t} data structure
 * @param[in] The configuration data data; Must not be null.
 */
void MPU6050_StoreConfiguration(const mpu6050_confreg_t *const configuration)
{
	assert(0);
}
