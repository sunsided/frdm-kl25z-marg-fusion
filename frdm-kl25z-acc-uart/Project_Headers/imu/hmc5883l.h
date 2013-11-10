/*
 * hmc5883l.h
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_

/**
 * @brief I2C slave address of the HMC5883L magnetometer
 */
#define HMC5883L_I2CADDR	(0b0011110)

#define HMC5883L_REG_CRA	(0x00)	/*< Configuration Register A */
#define HMC5883L_REG_CRB	(0x01)	/*< Configuration Register B */
#define HMC5883L_REG_MR		(0x02)	/*< Mode Register */
#define HMC5883L_REG_DXRA	(0x03)	/*< Data Output X Register A */
#define HMC5883L_REG_DXRB	(0x04)	/*< Data Output X Register B */
#define HMC5883L_REG_DYRA	(0x05)	/*< Data Output Y Register A */
#define HMC5883L_REG_DYRB	(0x06)	/*< Data Output Y Register B */
#define HMC5883L_REG_DZRA	(0x07)	/*< Data Output Z Register A */
#define HMC5883L_REG_DZRB	(0x08)	/*< Data Output Z Register B */
#define HMC5883L_REG_SR		(0x09)	/*< Status Register */
#define HMC5883L_REG_IRA	(0x0A)	/*< Identification Register A */
#define HMC5883L_REG_IRB	(0x0B)	/*< Identification Register B */
#define HMC5883L_REG_IRC	(0x0C)	/*< Identification Register C */

#define HMC5883L_CRA_MS_SHIFT	(0x00u)
#define HMC5883L_CRA_MS_MASK	(0b00000011)
#define HMC5883L_CRA_DO_SHIFT	(0x02u)
#define HMC5883L_CRA_DO_MASK	(0b00011100)
#define HMC5883L_CRA_MA_SHIFT	(0x05u)
#define HMC5883L_CRA_MA_MASK	(0b01100000)
#define HMC5883L_CRB_GN_SHIFT	(0x05u)
#define HMC5883L_CRB_GN_MASK	(0b11100000)
#define HMC5883L_MR_MD_SHIFT	(0x00u)
#define HMC5883L_MR_MD_MASK		(0b00000011)
#define HMC5883L_SR_RDY_SHIFT	(0x00u)
#define HMC5883L_SR_RDY_MASK	(0b00000001)
#define HMC5883L_SR_LOCK_SHIFT	(0x01u)
#define HMC5883L_SR_LOCK_MASK	(0b00000010)

/**
 * @brief Reads the Identification registers from the HMC5883L.
 * @return Device identification code; Should be 0x00483433('\0H43')
 */
static inline uint32_t HMC5883L_Identification()
{
	uint32_t result = 0;
	uint8_t* ptr = (uint8_t*)&result;
	I2C_ReadRegisters(HMC5883L_I2CADDR, HMC5883L_REG_IRA, 3, ptr);
	
	result = I2C_ReadRegister(HMC5883L_I2CADDR, HMC5883L_REG_IRA);
	result = I2C_ReadRegister(HMC5883L_I2CADDR, HMC5883L_REG_IRB);
	result = I2C_ReadRegister(HMC5883L_I2CADDR, HMC5883L_REG_IRC);
	
	return result;
}

#endif /* HMC5883L_H_ */
