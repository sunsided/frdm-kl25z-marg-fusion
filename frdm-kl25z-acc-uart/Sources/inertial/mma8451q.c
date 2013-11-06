/*
 * mma8451q.c
 *
 *  Created on: Nov 5, 2013
 *      Author: Markus
 */

#include "endian.h"
#include "inertial/mma8451q.h"

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
