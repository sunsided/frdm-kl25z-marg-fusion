/*
 * mma8451q.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#ifndef MMA8451Q_H_
#define MMA8451Q_H_

/**
 * @brief I2C slave address of the MMA8451Q accelerometer
 */
#define MMA8451Q_I2CADDR	(0b0011101) /* on the FRDM-KL25Z, the SA0 pin of the MMA8451Q is pulled high */

/**
 * @brief Reads the WHO_AM_I register from the MMA8451Q.
 * @return Device identification code; Should be 0b00011010. 
 */
uint8_t MMA8451Q_WhoAmI();

#endif /* MMA8451Q_H_ */
