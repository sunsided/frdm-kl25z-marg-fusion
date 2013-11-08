/*
 * i2cint.h
 *
 * Interrupt driven I2C master
 *
 *  Created on: Nov 8, 2013
 *      Author: Markus
 */

#ifndef I2CMINT_H_
#define I2CMINT_H_

/**
 * @brief State definitions of the I2C master interrupt handler
 * 
 * Possible transitions:
 *  [*]					--> START
 *  START 				-> SEND_WRITE_ADDRESS
 *  SEND_WRITE_ADDRESS	-> SEND_COMMAND
 *  SEND_COMMAND		-> RESTART
 *  SEND_COMMAND		-> SEND_DATA
 *  SEND_DATA			-> SEND_DATA
 *  SEND_DATA			-> SEND_LAST_DATA
 *  SEND_LAST_DATA		-> STOP
 *  SEND_LAST_DATA		-> RESTART
 *  RESTART				-> SEND_WRITE_ADDRESS
 *  RESTART				-> SEND_READ_ADDRESS
 *  SEND_READ_ADDRESS	-> READ_DATA
 *  READ_DATA			-> READ_DATA
 *  READ_DATA			-> READ_2ND_LAST_DATA
 *  READ_2ND_LAST_DATA	-> READ_LAST_DATA
 *  READ_LAST_DATA		-> RESTART
 *  READ_LAST_DATA		-> STOP
 *  STOP				--> [*]
 */
typedef enum
{
	START,					/*< Start condition */
	SEND_WRITE_ADDRESS,		/*< I2C slave id + W bit */
	SEND_COMMAND,			/*< send command/register */
	SEND_DATA,				/*< send data byte 1..(N-1) */
	SEND_LAST_DATA,			/*< send data byte N */
	RESTART,				/*< Restart condition */
	SEND_READ_ADDRESS,		/*< I2C slave id + R bit */
	READ_DATA,				/*< read data byte 1..(N-2) and send ACK */
	READ_2ND_LAST_DATA,		/*< read data byte N-1 and send NACK */
	READ_LAST_DATA,			/*< read data byte N */
	STOP,					/*< stop */
} i2c_master_state_t;

#endif /* I2CMINT_H_ */
