/*
 * i2c.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#ifndef I2C_H_
#define I2C_H_

/**
 * @brief Enables or disables decorated storage support using the
 * Bit Manipulation Engine.
 */
#define I2C_USE_BME 1

/**
 *  @brief According to KINETIS_L_2N97F errata (e6070), repeated start condition can not be sent if prescaler is any other than 1 (0x0). 
 *  Setting this define to a nonzero value activates the proposed workaround (temporarily disabling the multiplier).
 */
#define I2C_ENABLE_E6070_SPEEDHACK 	(1)

#include "ARMCM0plus.h"
#include "derivative.h"
#include "nice_names.h"

#if I2C_USE_BME
#include "bme.h"
#endif

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
void I2C_Init();

/**
 * @brief Resets the bus by toggling master mode if the bus is busy. This will interrupt ongoing traffic, so use with caution.
 */
void I2C_ResetBus();

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
 * @brief Reads an 8-bit register from an I2C slave, modifies it by FIRST and-ing with {@see andMask} and THEN or-ing with {@see orMask} and writes it back
 * @param[in] slaveId The slave id
 * @param[in] registerAddress The register to modify
 * @param[in] orMask The mask to OR the register with
 * @param[in] andMask The mask to AND the register with
 * @return The register after modification
 */
uint8_t I2C_ModifyRegister(register uint8_t slaveId, uint8_t register registerAddress, register uint8_t andMask, register uint8_t orMask);


/**
 * @brief Waits for an I2C bus operation to complete
 */
__STATIC_INLINE void I2C_Wait()
{
	while((I2C0->S & I2C_S_IICIF_MASK)==0) {}	/* loop until interrupt is detected */
	
#if !I2C_USE_BME
	I2C0->S |= I2C_S_IICIF_MASK; /* clear interrupt flag */
#else
	BME_OR_B(&I2C0->S, ((1 << I2C_S_IICIF_SHIFT) << I2C_S_IICIF_MASK));
#endif
}

/**
 * @brief Waits for an I2C bus operation to complete
 */
__STATIC_INLINE void I2C_WaitWhileBusy()
{
	while((I2C0->S & I2C_S_BUSY_MASK)!=0) {}
}


/**
 * @brief Sends a start condition and enters TX mode.
 */
__STATIC_INLINE void I2C_SendStart()
{
#if !I2C_USE_BME	
	I2C0->C1 |= ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK) 
				| ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
#else
	BME_OR_B(&I2C0->C1, 
			  ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK) 
			| ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
		);
#endif
}

/**
 * @brief Enters transmit mode.
 */
__STATIC_INLINE void I2C_EnterTransmitMode()
{
#if !I2C_USE_BME
	I2C0->C1 |= ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
#else
	BME_OR_B(&I2C0->C1, 
			((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
		);
#endif
}

/**
 * @brief Enters receive mode.
 */
__STATIC_INLINE void I2C_EnterReceiveMode()
{
#if !I2C_USE_BME
	I2C0->C1 &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
#else
	BME_AND_B(&I2C0->C1,
			(uint8_t)
			~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
		);
#endif
}

/**
 * @brief Enters receive mode and enables ACK.
 * 
 * Enabling ACK may be required when more than one data byte will be read.
 */
__STATIC_INLINE void I2C_EnterReceiveModeWithAck()
{
#if !I2C_USE_BME
	I2C0->C1 &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)	
			& ~((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
#else
	BME_AND_B(&I2C0->C1,
			(uint8_t) ~(
				  ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
				| ((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK)
			)
		);
#endif
}

/**
 * @brief Enters receive mode and disables ACK.
 * 
 * Disabling ACK may be required when only one data byte will be read.
 */
__STATIC_INLINE void I2C_EnterReceiveModeWithoutAck()
{
	/* Straightforward method of clearing TX mode and
	 * setting NACK bit sending.
	 */
#if !I2C_USE_BME
	register uint8_t reg = I2C0->C1;
	reg &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
	reg |=  ((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
	I2C0->C1 = reg;
#else
	
	/* Alternative using the Bit Manipulation Engine
	 * and decorated Logic AND/OR stores */
#if 0
	BME_AND_B(&I2C0->C1, ~(1 << I2C_C1_TX_SHIFT));
	BME_OR_B(&I2C0->C1,   1 << I2C_C1_TXAK_SHIFT);
#endif
	
	/* Even better alternative: BME Bit Field Insert
	 * - TX   bit is 0x10 (5th bit, 0b00010000)
	 * - TXAK bit is 0x08 (4th bit, 0b00001000)
	 * Thus the following can be deduced:
	 * - The mask for clearing both bits is 0x18 (0b00011000)
	 *   This corresponds to a 2 bit wide mask, shifted by 3 
	 * - The mask for setting  TXAK bit  is 0x08 (0b00001000)
	 */
	BME_BFI_B(&I2C0->C1, 0x08, 3, 2);

#endif /* USE_BME */
}

/**
 * @brief Sends a repeated start condition and enters TX mode.
 */
__STATIC_INLINE void I2C_SendRepeatedStart()
{
#if I2C_ENABLE_E6070_SPEEDHACK
	register uint8_t reg = I2C0->F;
	I2C0->F = reg & ~I2C_F_MULT_MASK; /* NOTE: According to KINETIS_L_2N97F errata (e6070), repeated start condition can not be sent if prescaler is any other than 1 (0x0). A solution is to temporarily disable the multiplier. */
#endif
	
#if !I2C_USE_BME
	I2C0->C1 |= ((1 << I2C_C1_RSTA_SHIFT) & I2C_C1_RSTA_MASK)
			  | ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
#else
	BME_OR_B(&I2C0->C1, 
			  ((1 << I2C_C1_RSTA_SHIFT) & I2C_C1_RSTA_MASK)	
			| ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
		);
#endif

#if I2C_ENABLE_E6070_SPEEDHACK
	I2C0->F = reg;
#endif	
}

/**
 * @brief Sends a stop condition (also leaves TX mode)
 */
__STATIC_INLINE void I2C_SendStop()
{
#if !I2C_USE_BME
	I2C0->C1 &= ~((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK)
			& ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
#else
	BME_AND_B(&I2C0->C1,
			(uint8_t) ~(
				  ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK)
				| ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
			)
		);
#endif
}

/**
 * @brief Enables sending of ACK
 * 
 * Enabling ACK may be required when more than one data byte will be read.
 */
__STATIC_INLINE void I2C_EnableAck()
{
#if !I2C_USE_BME
	I2C0->C1 &= ~((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
#else
	BME_AND_B(&I2C0->C1, 
			(uint8_t)
			~((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK)
	);
#endif
}

/**
 * @brief Enables sending of NACK (disabling ACK)
 * 
 * Enabling NACK may be required when no more data byte will be read.
 */
__STATIC_INLINE void I2C_DisableAck()
{
#if !I2C_USE_BME
	I2C0->C1 |= ((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
#else
	BME_OR_B(&I2C0->C1,
			((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK)
		);
#endif
}

/**
 * @brief Sends a byte over the I2C bus and waits for the operation to complete
 * @param[in] value The byte to send
 */
__STATIC_INLINE void I2C_SendBlocking(const uint8_t value)
{
	I2C0->D = value;
	I2C_Wait();
}

/**
 * @brief Reads a byte over the I2C bus and drives the clock for another byte
 * @return There received byte
 */
__STATIC_INLINE uint8_t I2C_ReceiveDriving()
{
	register uint8_t value = I2C0->D;
	I2C_Wait();
	return value;
}

/**
 * @brief Reads a byte over the I2C bus and drives the clock for another byte, while sending NACK
 * @return There received byte
 */
__STATIC_INLINE uint8_t I2C_ReceiveDrivingWithNack()
{
	I2C_DisableAck();
	return I2C_ReceiveDriving();
}

/**
 * @brief Reads the last byte over the I2C bus and sends a stop condition
 * @return There received byte
 */
__STATIC_INLINE uint8_t I2C_ReceiveAndStop()
{
	I2C_SendStop();
	return I2C0->D;
}

/**
 * @brief Reads a byte over the I2C bus and sends a repeated start condition.
 * @return There received byte
 * 
 * The I2C module is in transmit mode afterwards.
 */
__STATIC_INLINE uint8_t I2C_ReceiveAndRestart()
{
	I2C_SendRepeatedStart();
	return I2C0->D;
}

/**
 * @brief Drives the clock in receiver mode in order to receive the first byte.
 */
__STATIC_INLINE void I2C_ReceiverModeDriveClock()
{
	INTENTIONALLY_UNUSED(register uint8_t) = I2C0->D;
	I2C_Wait();
}

#endif /* I2C_H_ */
