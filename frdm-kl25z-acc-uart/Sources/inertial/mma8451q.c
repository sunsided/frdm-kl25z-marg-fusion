/*
 * mma8451q.c
 *
 *  Created on: Nov 5, 2013
 *      Author: Markus
 */

#include "ARMCM0plus.h"
#include "endian.h"
#include "nice_names.h"
#include "inertial/mma8451q.h"

#define CTRL_REG1_DR_MASK 		0x38
#define CTRL_REG1_DR_SHIFT 		0x3
#define CTRL_REG1_LNOISE_MASK 	0x4
#define CTRL_REG1_LNOISE_SHIFT 	0x2

#define CTRL_REG2_MODS_MASK 	0x38
#define CTRL_REG2_MODS_SHIFT 	0x3

#define CTRL_REG3_IPOL_MASK 	0x2
#define CTRL_REG3_IPOL_SHIFT 	0x1
#define CTRL_REG3_PPOD_MASK 	0x1
#define CTRL_REG3_PPOD_SHIFT 	0x0

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

/**
 * @brief Sets the data rate and the active mode
 */
void MMA8451Q_SetDataRate(mma8451q_datarate_t datarate, mma8451q_lownoise_t lownoise)
{
	const register uint8_t value = ((datarate << CTRL_REG1_DR_SHIFT) & CTRL_REG1_DR_MASK) | ((lownoise << CTRL_REG1_LNOISE_SHIFT) & CTRL_REG1_LNOISE_MASK);
	const register uint8_t mask = ~(CTRL_REG1_DR_MASK | CTRL_REG1_LNOISE_MASK);
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG1, mask, value);
}

/**
 * @brief Reads the SYSMOD register from the MMA8451Q.
 * @return Current system mode. 
 */
uint8_t MMA8451Q_SystemMode()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_SYSMOD);
}

/**
 * @brief Reads the REG_PL_CFG register from the MMA8451Q.
 * @return Current portrait/landscape mode. 
 */
uint8_t MMA8451Q_LandscapePortraitConfig()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_PL_CFG);
}

/**
 * @brief Configures the oversampling modes
 * @param[in] oversampling The oversampling mode
 */
void MMA8451Q_SetOversampling(mma8451q_oversampling_t oversampling)
{
	const register uint8_t value = (oversampling << CTRL_REG2_MODS_SHIFT) & CTRL_REG2_MODS_MASK;
	const register uint8_t mask = ~(CTRL_REG2_MODS_MASK);
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG2, mask, value);
}

/**
 * @brief Configures the sensitivity and the high pass filter
 * @param[in] sensitivity The sensitivity
 * @param[in] highpassEnabled Set to 1 to enable the high pass filter or to 0 otherwise (default)
 */
void MMA8451Q_SetSensitivity(mma8451q_sensitivity_t sensitivity, mma8451q_hpo_t highpassEnabled)
{
	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_XZY_DATA_CFG, (sensitivity & 0x03) | ((highpassEnabled << 4) & 0x10));
}

/**
 * @brief Enables or disables interrupts
 * @param[in] mode The mode
 * @param[in] polarity The polarity
 */
void MMA8451Q_SetInterruptMode(mma8451q_intmode_t mode, mma8451q_intpol_t polarity)
{
	const uint8_t value = ((mode << CTRL_REG3_PPOD_SHIFT) & CTRL_REG3_PPOD_MASK)
								| ((polarity << CTRL_REG3_IPOL_SHIFT) & CTRL_REG3_IPOL_MASK);
	const uint8_t mask = ~(CTRL_REG3_IPOL_MASK | CTRL_REG3_PPOD_MASK);
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG3, mask, value);
}

/**
 * @brief Enables or disables specific interrupts
 * @param[in] irq The interrupt
 * @param[in] pin The pin
 */
void MMA8451Q_ConfigureInterrupt(mma8451q_interrupt_t irq, mma8451q_intpin_t pin)
{
	/* interrupt pin. Assume that the interrupt 1 should be set */
	uint8_t clearMask = I2C_MOD_NO_AND_MASK;
	uint8_t setMask = (1 << irq);
	
	/* if pin 2 should be used, revert */
	if (MMA8451Q_INTPIN_INT2 == pin) 
	{
		clearMask = ~(1 << irq);
		setMask = I2C_MOD_NO_OR_MASK;
	}
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG5, clearMask, setMask);	
	
	/* interrupt enable */
	I2C_ModifyRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG4, I2C_MOD_NO_AND_MASK, 1 << irq);
}

/**
 * @brief Clears the interrupt configuration
 */
void MMA8451Q_ClearInterruptConfiguration()
{
	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG4, 0);
	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG5, 0);	
}

/**
 * @brief Fetches the configuration into a {@see mma8451q_confreg_t} data structure
 * @param[inout] The configuration data data; Must not be null.
 */
void MMA8451Q_FetchConfiguration(mma8451q_confreg_t *const configuration)
{
	assert(configuration != 0x0);
		
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();
	
	/* start register addressing */
	I2C_SendStart();
	I2C_SendBlocking(I2C_WRITE_ADDRESS(MMA8451Q_I2CADDR));
	I2C_SendBlocking(MMA8451Q_REG_F_SETUP);

	/* start read */
	I2C_SendRepeatedStart();
	I2C_SendBlocking(I2C_READ_ADDRESS(MMA8451Q_I2CADDR));
	I2C_EnterReceiveModeWithAck();
	I2C_ReceiverModeDriveClock();
	
	/* read the registers */
	configuration->F_SETUP = I2C_ReceiveDriving();
	configuration->TRIG_CFG = I2C_ReceiveDriving();
	*((uint8_t*)&configuration->SYSMOD) = I2C_ReceiveDriving();
	
	I2C_ReceiverModeDriveClock(); /* skip 1 register */
	
	*((uint8_t*)&configuration->WHO_AM_I) = I2C_ReceiveDriving();
	configuration->XYZ_DATA_CFG = I2C_ReceiveDriving();
	configuration->HP_FILTER_CUTOFF = I2C_ReceiveDriving();
	
	I2C_ReceiverModeDriveClock(); /* skip 1 register */
	
	configuration->PL_CFG = I2C_ReceiveDriving();
	configuration->PL_COUNT = I2C_ReceiveDriving();
	configuration->PL_BF_ZCOMP = I2C_ReceiveDriving();
	configuration->P_L_THS_REG = I2C_ReceiveDriving();
	configuration->FF_MT_CFG = I2C_ReceiveDriving();
	
	I2C_ReceiverModeDriveClock(); /* skip 1 register */
	
#if 1
	/* After FF_MT_THS (0x17) and FF_MT_COUNT (0x18)
	 * the next 4 registers are undefined, so bulk-reading
	 * over them may yield in undesired behaviour
	 */
	configuration->FF_MT_THS = I2C_ReceiveDrivingWithNack();
	configuration->FF_MT_COUNT = I2C_ReceiveAndRestart();
	
	/* restart read at 0x1D */
	I2C_EnterTransmitMode();
	I2C_SendBlocking(I2C_WRITE_ADDRESS(MMA8451Q_I2CADDR));
	I2C_SendBlocking(MMA8451Q_TRANSIENT_CFG);
	
	/* re-enter read mode */
	I2C_SendRepeatedStart();
	I2C_SendBlocking(I2C_READ_ADDRESS(MMA8451Q_I2CADDR));
	I2C_EnterReceiveModeWithAck();
	I2C_ReceiverModeDriveClock();
#else
	/* bulk-read through the next registers */
	configuration->FF_MT_THS = I2C_ReceiveDriving();
	configuration->FF_MT_COUNT = I2C_ReceiveDriving();
	
	I2C_ReceiverModeDriveClock(); /* skip 4 registers */
	I2C_ReceiverModeDriveClock();
	I2C_ReceiverModeDriveClock();
	I2C_ReceiverModeDriveClock();
#endif
		
	configuration->TRANSIENT_CFG = I2C_ReceiveDriving();
	configuration->TRANSIENT_SCR = I2C_ReceiveDriving();
	configuration->TRANSIENT_THS = I2C_ReceiveDriving();
	configuration->TRANSIENT_COUNT = I2C_ReceiveDriving();
	configuration->PULSE_CFG = I2C_ReceiveDriving();
	
	I2C_ReceiverModeDriveClock(); /* skip 1 register */
	
	configuration->PULSE_THSX = I2C_ReceiveDriving();
	configuration->PULSE_THSY = I2C_ReceiveDriving();
	configuration->PULSE_THSZ = I2C_ReceiveDriving();
	configuration->PULSE_TMLT = I2C_ReceiveDriving();
	configuration->PULSE_LTCY = I2C_ReceiveDriving();
	configuration->PULSE_WIND = I2C_ReceiveDriving();
	configuration->ASPL_COUNT = I2C_ReceiveDriving();
	configuration->CTRL_REG1 = I2C_ReceiveDriving();
	configuration->CTRL_REG2 = I2C_ReceiveDriving();
	configuration->CTRL_REG3 = I2C_ReceiveDriving();
	configuration->CTRL_REG4 = I2C_ReceiveDriving();
	configuration->CTRL_REG5 = I2C_ReceiveDriving();
	configuration->OFF_X = I2C_ReceiveDriving();
	configuration->OFF_Y = I2C_ReceiveDrivingWithNack();
	configuration->OFF_Z = I2C_ReceiveAndStop();
}

/**
 * @brief Stores the configuration from a {@see mma8451q_confreg_t} data structure
 * @param[in] The configuration data data; Must not be null.
 */
void MMA8451Q_StoreConfiguration(const mma8451q_confreg_t *const configuration)
{
	assert(configuration != 0x0);
}
