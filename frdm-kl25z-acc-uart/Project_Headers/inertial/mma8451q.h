/*
 * mma8451q.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#ifndef MMA8451Q_H_
#define MMA8451Q_H_

#include "derivative.h"
#include "i2c/i2c.h"

/**
 * @brief I2C slave address of the MMA8451Q accelerometer
 */
#define MMA8451Q_I2CADDR	(0b0011101) /* on the FRDM-KL25Z, the SA0 pin of the MMA8451Q is pulled high */

#define MMA8451Q_STATUS_ZYXOW(status)	(status & 0b10000000)	/*< X/Y/Z data overwritten before read */
#define MMA8451Q_STATUS_ZOW(status)		(status & 0b01000000)	/*< X data overwritten before read */
#define MMA8451Q_STATUS_YOW(status)		(status & 0b00100000)	/*< Y data overwritten before read */
#define MMA8451Q_STATUS_XOW(status)		(status & 0b00010000)	/*< Z data overwritten before read */
#define MMA8451Q_STATUS_XYZDR(status)	(status & 0b00001000)	/*< X/Y/Z data ready */
#define MMA8451Q_STATUS_X(status)		(status & 0b00000100)	/*< X data ready */
#define MMA8451Q_STATUS_Y(status)		(status & 0b00000010)	/*< Y data ready */
#define MMA8451Q_STATUS_ZDR(status)		(status & 0b00000001)	/*< Z data ready */

#define MMA8451Q_REG_STATUS				(0x00)	/*< STATUS register */
#define MMA8451Q_REG_SYSMOD				(0x0B)	/*< SYSMOD register for system mode identification */
#define MMA8451Q_REG_PL_CFG				(0x11)	/*< PL_CFG register for portrait/landscape detection configuration */
#define MMA8451Q_REG_WHOAMI				(0x0D)	/*< WHO_AM_I register for device identification */
#define MMA8451Q_REG_CTRL_REG1			(0x2A)	/*< CTRL_REG1 System Control 1 Register */

/**
 * @brief Accelerometer data
 */
#pragma pack(1)
typedef struct __attribute__ ((__packed__))
{
	uint8_t :8; 		/*< padding byte */
	uint8_t status;		/*< the status register contents */
	int16_t x;			/*< the x acceleration */
	int16_t y;			/*< the y acceleration */
	int16_t z;			/*< the z acceleration */
} mma8451q_acc_t;

/**
 * @brief Initializes a {@see mma8451q_acc_t} data structure
 * @param[inout] The accelerometer data; Must not be null.
 */
static inline void MMA8451Q_InitializeData(mma8451q_acc_t *const data)
{
	uint32_t* ptr = (uint32_t*)data;
	ptr[0] = 0;
	ptr[1] = 0;
}

/**
 * @brief Reads the accelerometer data in 14bit no-fifo mode
 * @param[inout] The accelerometer data; Must not be null. 
 */
void MMA8451Q_ReadAcceleration14bitNoFifo(mma8451q_acc_t *const data);

/**
 * @brief Reads the STATUS register from the MMA8451Q.
 * @return Status bits, see MMA8451Q_STATUS_XXXX defines. 
 */
static inline uint8_t MMA8451Q_Status()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_STATUS);
}

/**
 * @brief Reads the SYSMOD register from the MMA8451Q.
 * @return Current system mode. 
 */
static inline uint8_t MMA8451Q_SystemMode()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_SYSMOD);
}

/**
 * @brief Reads the SYSMOD register from the MMA8451Q.
 * @return Current system mode. 
 */
static inline uint8_t MMA8451Q_LandscapePortraitConfig()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_PL_CFG);
}

/**
 * @brief Brings the MMA8451Q into passive mode
 */
static inline void MMA8451Q_EnterPassiveMode()
{
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG1, ~0b00000001, I2C_MOD_NO_OR_MASK);
}

/**
 * @brief Brings the MMA8451Q into active mode
 */
static inline void MMA8451Q_EnterActiveMode()
{
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG1, I2C_MOD_NO_AND_MASK, 0b00000001);
}

/**
 * @brief Reads the WHO_AM_I register from the MMA8451Q.
 * @return Device identification code; Should be 0b00011010. 
 */
static inline uint8_t MMA8451Q_WhoAmI()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_WHOAMI);
}

#endif /* MMA8451Q_H_ */
