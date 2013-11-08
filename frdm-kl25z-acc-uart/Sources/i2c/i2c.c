/*
 * spi.c
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#include "ARMCM0plus.h"
#include "derivative.h"
#include "bme.h"

#include "nice_names.h"

#include "i2c/i2c.h"

/**
 *  @brief According to KINETIS_L_2N97F errata (e6070), repeated start condition can not be sent if prescaler is any other than 1 (0x0). 
 *  Setting this define to a nonzero value activates the proposed workaround (temporarily disabling the multiplier).
 */
#define ENABLE_SPEEDHACK 	(1)

/**
 * @brief Initialises the I2C interface
 */
void I2C_Init()
{
	/* enable clock gating to I2C0 */
	SIM->SCGC4 |= (1 << SIM_SCGC4_I2C0_SHIFT) & SIM_SCGC4_I2C0_MASK;
	
	/* enable the clock gate to port E */
	SIM->SCGC5 |= (1 << SIM_SCGC5_PORTE_SHIFT) & SIM_SCGC5_PORTE_MASK;
	
	/* configure port E pins to I2C operation */
	PORTE->PCR[24] = PORT_PCR_MUX(5); /* SCL */
	PORTE->PCR[25] = PORT_PCR_MUX(5); /* SDA */
	
	/* configure the I2C clock */
	/* 
	 * I2C0 is clocked by the bus clock, that is core/2.
	 * For the MMA8451Q inertial sensor on the FRDM-25KLZ board the
	 * maximum SCL frequency is 400 kHz.
	 * Assuming PEE mode with core=48MHz, 400 kHz = 48MHz/2 / 60,
	 * which means a prescaler (SCL divider) of 60.
	 * According to table 38-41, I2C divider and hold values,
	 * the closest SCL diver is 64 (375 kHz SCL), which is ICR value 0x12.
	 * Alternatively, the SCL divider of 30 can be used (ICR=0x05) in combination
	 * with a multiplicator of 2 (MULT=0x01).
	 * A note states that ICR values lower than 0x10 might result in a varying
	 * SCL divider (+/- 4). However the data sheet does not state anything
	 * useful about that.
	 */
#if ENABLE_SPEEDHACK
	I2C0->F = I2C_F_MULT(0x01) | I2C_F_ICR(0x05); /* NOTE: According to KINETIS_L_2N97F errata (e6070), repeated start condition can not be sent if prescaler is any other than 1 (0x0). */
#else
	I2C0->F = I2C_F_MULT(0x00) | I2C_F_ICR(0x12); /* divide by 64 instead, so 375 kHz */
#endif
	
	/* enable the I2C module */
	I2C0->C1 = (1 << I2C_C1_IICEN_SHIFT) & I2C_C1_IICEN_MASK;
}

/**
 * @brief Waits for an I2C bus operation to complete
 */
__STATIC_INLINE void I2C_Wait()
{
	while((I2C0->S & I2C_S_IICIF_MASK)==0) {}	/* loop until interrupt is detected */
	I2C0->S |= I2C_S_IICIF_MASK; /* clear interrupt flag */
}

/**
 * @brief Waits for an I2C bus operation to complete
 */
__STATIC_INLINE void I2C_WaitWhileBusy()
{
	while((I2C0->S & I2C_S_BUSY_MASK)!=0) {}
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
 * @brief Sends a start condition and enters TX mode.
 */
__STATIC_INLINE void I2C_SendStart()
{
#if 0	
	I2C0->C1 |= ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK) 
				| ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
#endif
	BME_OR_B(&I2C0->C1, 
			  ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK) 
			| ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
		);
}

/**
 * @brief Sends a stop condition (also leaves TX mode)
 */
__STATIC_INLINE void I2C_SendStop()
{
#if 0
	I2C0->C1 &= ~((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK)
			& ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
#endif
	BME_AND_B(&I2C0->C1,
			(uint8_t) ~(
				  ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK)
				| ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)
			)
		);
}

/**
 * @brief Sends a repeated start condition.
 */
__STATIC_INLINE void I2C_SendRepeatedStart()
{
#if ENABLE_SPEEDHACK
	register uint8_t reg = I2C0->F;
	I2C0->F = reg & ~I2C_F_MULT_MASK; /* NOTE: According to KINETIS_L_2N97F errata (e6070), repeated start condition can not be sent if prescaler is any other than 1 (0x0). A solution is to temporarily disable the multiplier. */
#endif
	
#if 0
	I2C0->C1 |= ((1 << I2C_C1_RSTA_SHIFT) & I2C_C1_RSTA_MASK);
#endif
	BME_OR_B(&I2C0->C1, 
			((1 << I2C_C1_RSTA_SHIFT) & I2C_C1_RSTA_MASK)	
		);

#if ENABLE_SPEEDHACK
	I2C0->F = reg;
#endif	
}

/**
 * @brief Enters receive mode.
 */
__STATIC_INLINE void I2C_EnterTransmitMode()
{
	I2C0->C1 |= ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK); /* TODO: use BME in all these places */
}

/**
 * @brief Enters receive mode.
 */
__STATIC_INLINE void I2C_EnterReceiveMode()
{
	I2C0->C1 &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
}

/**
 * @brief Enters receive mode and enables ACK.
 * 
 * Enabling ACK may be required when more than one data byte will be read.
 */
__STATIC_INLINE void I2C_EnterReceiveModeWithAck()
{
	I2C0->C1 &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK)	
			& ~((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
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
#if 0
	register uint8_t reg = I2C0->C1;
	reg &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
	reg |=  ((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
	I2C0->C1 = reg;
#endif
	
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
}

/**
 * @brief Enables sending of ACK
 * 
 * Enabling ACK may be required when more than one data byte will be read.
 */
__STATIC_INLINE void I2C_EnableAck()
{
	I2C0->C1 &= ~((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
}

/**
 * @brief Enables sending of NACK (disabling ACK)
 * 
 * Enabling NACK may be required when no more data byte will be read.
 */
__STATIC_INLINE void I2C_DisableAck()
{
	I2C0->C1 |= ((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
}

/**
 * @brief Drives the clock in receiver mode in order to receive the first byte.
 */
__STATIC_INLINE void I2C_ReceiverModeDriveClock()
{
	INTENTIONALLY_UNUSED(register uint8_t) = I2C0->D;
	I2C_Wait();
}

/**
 * @brief Reads an 8-bit register from an I2C slave 
 */
uint8_t I2C_ReadRegister(register uint8_t slaveId, register uint8_t registerAddress)
{
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();
	
	/* send I2C start signal and set write direction, also enables ACK */
	I2C_SendStart();
	
	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	I2C_SendBlocking(registerAddress);
	
	/* signal a repeated start condition */
	I2C_SendRepeatedStart();

	/* send the read address */
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId));
	
	/* switch to receive mode but disable ACK because only one data byte will be read */
	I2C_EnterReceiveModeWithoutAck();
	
	/* read a dummy byte to drive the clock */
	I2C_ReceiverModeDriveClock();
	
	/* stop signal */
	I2C_SendStop();
	
	/* fetch the last received byte */
	register uint8_t result = I2C0->D;
	return result;
}

/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be greater than or equal to two.
 * @param[out] buffere The buffer to write into
 */
static void I2C_ReadRegistersInternal(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, uint8_t *buffer)
{
	assert(registerCount >= 2);
	
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();
	
	/* send I2C start signal and set write direction, also enables ACK */
	I2C_SendStart();
	
	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	I2C_SendBlocking(startRegisterAddress);
	
	/* signal a repeated start condition */
	I2C_SendRepeatedStart();

	/* send the read address */
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId));
	
	/* switch to receive mode and assume more than one register */
	I2C_EnterReceiveModeWithAck();
	
	/* read a dummy byte to drive the clock */
	I2C_ReceiverModeDriveClock();
	
	/* for all remaining bytes, read */
	--registerCount;
	uint8_t index = 0;
	while (--registerCount > 0)
	{
		/* fetch and store value */
		register uint8_t value = I2C0->D;
		buffer[index++] = value;
		
		/* wait for completion */
		I2C_Wait();
	}
	
	/* disable ACK and read second-to-last byte */
	I2C_DisableAck();
	
	/* fetch and store value */
	buffer[index++] = I2C0->D;
	
	/* wait for completion */
	I2C_Wait();
	
	/* stop signal */
	I2C_SendStop();
	
	/* fetch the last received byte */
	buffer[index++] = I2C0->D; 
}

/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be larger than zero.
 * @param[out] buffer The buffer to write into
 */
void I2C_ReadRegisters(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, register uint8_t *buffer)
{
	assert(registerCount > 0);
	
	if (registerCount >= 2)
	{
		I2C_ReadRegistersInternal(slaveId, startRegisterAddress, registerCount, buffer);
	}
	else
	{
		assert(1 == registerCount);
		register uint8_t result = I2C_ReadRegister(slaveId, startRegisterAddress);
		buffer[0] = result;
	}
}

/**
 * @brief Reads an 8-bit register from an I2C slave 
 */
void I2C_WriteRegister(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t value)
{
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();	
	
	/* send I2C start signal and set write direction*/
	I2C_SendStart();
		
	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	I2C_SendBlocking(registerAddress);
		
	/* send the register address */
	I2C_SendBlocking(value);
	
	/* issue stop signal by clearing master mode. */
	I2C_SendStop();
}

/**
 * @brief Reads an 8-bit register from an I2C slave, modifies it by FIRST and-ing with {@see andMask} and THEN or-ing with {@see orMask} and writes it back
 * @param[in] slaveId The slave id
 * @param[in] registerAddress The register to modify
 * @param[in] orMask The mask to OR the register with
 * @param[in] andMask The mask to AND the register with
 * @return The register after modification
 */
uint8_t I2C_ModifyRegister(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t andMask, register uint8_t orMask)
{
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();

	/* send the slave address and register */
	I2C_SendStart();
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));
	I2C_SendBlocking(registerAddress);
	
	/* signal a repeated start condition */
	I2C_SendRepeatedStart();
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId));
	
	/* switch to receive mode but disable ACK because only one data byte will be read */
	I2C_EnterReceiveModeWithoutAck();
	I2C_ReceiverModeDriveClock();
	
	/* instead of a stop signal, send repeated start again */
	I2C_SendRepeatedStart();
	
	/* fetch the last received byte */
	register uint8_t value = I2C0->D;
	
	/* modify the register */
	value &= andMask;
	value |= orMask;

	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_EnterTransmitMode();
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	I2C_SendBlocking(registerAddress);
		
	/* send the register address */
	I2C_SendBlocking(value);
	
	/* issue stop signal by clearing master mode. */
	I2C_SendStop();
	return value;
}

/**
 * @brief Resets the bus by toggling master mode if the bus is busy. This will interrupt ongoing traffic, so use with caution.
 */
void I2C_ResetBus()
{
	if (I2C0->S & I2C_S_BUSY_MASK)
	{
		I2C_SendStart();
		__WFI();
		I2C_SendStop();
	}
}
