/*
 * spi.c
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#include "derivative.h"
#include "nice_names.h"

#include "i2c/i2c.h"

/**
 * @brief Initialises the I2C interface
 */
void InitI2C()
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
	/*I2C0->F = I2C_F_MULT(0x01) | I2C_F_ICR(0x05);*/
	I2C0->F = I2C_F_MULT(0x00) | I2C_F_ICR(0x12);
	
	/* enable the I2C module */
	I2C0->C1 = (1 << I2C_C1_IICEN_SHIFT) & I2C_C1_IICEN_MASK;
}

/**
 * @brief Waits for an I2C bus operation to complete
 */
static inline void I2C_Wait()
{
	while((I2C0->S & I2C_S_IICIF_MASK)==0) {}	/* loop until interrupt is detected */
	I2C0->S |= I2C_S_IICIF_MASK; /* clear interrupt flag */
}

/**
 * @brief Sends a byte over the I2C bus and waits for the operation to complete
 * @param[in] value The byte to send
 */
static inline void I2C_SendBlocking(const uint8_t value)
{
	I2C0->D = value;
	I2C_Wait();
}

/**
 * @brief Reads an 8-bit register from an I2C slave 
 */
uint8_t I2C_ReadRegister(uint8_t slaveId, uint8_t registerAddress)
{
	/* send I2C start signal and set write direction*/
	I2C0->C1 |= ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK) 
			  | ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
	
	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId)); /* TODO: why are we even entering TX mode if we are sending the write ID? */
	
	/* send the register address */
	I2C_SendBlocking(registerAddress);
	
	/* signal a repeated start condition */
	I2C0->C1 |= (1 << I2C_C1_RSTA_SHIFT) & I2C_C1_RSTA_MASK;

	/* send the read address */
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId)); /* TODO: why are we even entering TX mode if we are sending the read ID? */
	
	/* switch to receive mode */
	I2C0->C1 &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
	
	/* disable ACK because "this is the second to last byte being read" */
	I2C0->C1 |= (1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK;
	
	/* read a dummy byte to drive the clock */
	uint8_t result = I2C0->D;
	
	/* wait for another cycle, then issue stop signal
	 * by clearing master mode.
	 */
	I2C_Wait();
	I2C0->C1 &= ~((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK)
			 | ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK); /* this shouldn't be necessary */

	/* fetch the received byte */
	result = I2C0->D;
	return result;
}
