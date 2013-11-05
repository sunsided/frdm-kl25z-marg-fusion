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
 * @brief Reads the SYSMOD register from the MMA8451Q.
 * @return Current system mode. 
 */
static inline void MMA8451Q_SetDefaultActiveMode()
{
	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG1, 0b00000001);
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
