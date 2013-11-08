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
#define MMA8451Q_REG_F_SETUP			(0x09)	/*< F_SETUP register */
#define MMA8451Q_REG_SYSMOD				(0x0B)	/*< SYSMOD register for system mode identification */
#define MMA8451Q_REG_PL_CFG				(0x11)	/*< PL_CFG register for portrait/landscape detection configuration */
#define MMA8451Q_REG_WHOAMI				(0x0D)	/*< WHO_AM_I register for device identification */
#define MMA8451Q_REG_XZY_DATA_CFG		(0x0E)	/*< XYZ_DATA_CFG sensitivity configuration */
#define MMA8451Q_REG_CTRL_REG1			(0x2A)	/*< CTRL_REG1 System Control 1 Register */
#define MMA8451Q_REG_CTRL_REG2			(0x2B)	/*< CTRL_REG2 System Control 2 Register */
#define MMA8451Q_REG_CTRL_REG3			(0x2C)	/*< CTRL_REG2 System Control 3 Register */
#define MMA8451Q_REG_CTRL_REG4			(0x2D)	/*< CTRL_REG2 System Control 4 Register */
#define MMA8451Q_REG_CTRL_REG5			(0x2E)	/*< CTRL_REG2 System Control 5 Register */

/**
 * @brief Sensitivity configuration
 */
typedef enum {
	MMA8451Q_SENSITIVITY_2G	= (0b00),		/*< dynamic range of +/- 1g */
	MMA8451Q_SENSITIVITY_4G	= (0b01),		/*< dynamic range of +/- 2g */
	MMA8451Q_SENSITIVITY_8G	= (0b10)		/*< dynamic range of +/- 4g */	
} mma8451q_sensitivity_t;

/**
 * @brief High-pass filtered output data
 */
typedef enum {
	MMA8451Q_HPO_DISABLED = (0b0),			/*< disables high-pass output data */
	MMA8451Q_HPO_ENABLED = (0b1)			/*< enables high-pass output data */
} mma8451q_hpo_t;

/**
 * @brief Acquisition data rate
 */
typedef enum {
	MMA8451Q_DATARATE_800Hz	= (0b000),		/*< 800Hz acquisition data rate */
	MMA8451Q_DATARATE_400Hz	= (0b001),		/*< 400Hz acquisition data rate */
	MMA8451Q_DATARATE_200Hz	= (0b010),		/*< 200Hz acquisition data rate */
	MMA8451Q_DATARATE_100Hz	= (0b011),		/*< 100Hz acquisition data rate */
	MMA8451Q_DATARATE_50Hz	= (0b100),		/*< 50Hz acquisition data rate */
	MMA8451Q_DATARATE_12p5Hz= (0b101),		/*< 12.5Hz acquisition data rate */
	MMA8451Q_DATARATE_6p2Hz	= (0b110),		/*< 6.25Hz acquisition data rate */
	MMA8451Q_DATARATE_1p5Hz	= (0b111)		/*< 1.56Hz acquisition data rate */
} mma8451q_datarate_t;

/**
 * @brief Reduced noise mode
 */
typedef enum {
	MMA8451Q_LOWNOISE_DISABLED	= (0b0),	/*< Reduced noise/reduced maximum range disabled */
	MMA8451Q_LOWNOISE_ENABLED	= (0b1)		/*< Reduced noise/reduced maximum range enabled */
} mma8451q_lownoise_t;

/**
 * @brief Oversampling mode
 */
typedef enum {
	MMA8451Q_OVERSAMPLING_NORMAL			= (0b00),		/*< Normal */
	MMA8451Q_OVERSAMPLING_LOWNOISELOWPOWER	= (0b01),		/*< Low Noise Low Power */
	MMA8451Q_OVERSAMPLING_HIGHRESOLUTION	= (0b10),		/*< High Resolution */
	MMA8451Q_OVERSAMPLING_LOWPOWER			= (0b11),		/*< Low Power */
} mma8451q_oversampling_t;

/**
 * @brief Interrupt pad mode
 */
typedef enum {
	MMA8451Q_INTMODE_PUSHPULL	= (0b0),	/*< Push/Pull mode */
	MMA8451Q_INTMODE_OPENDRAIN	= (0b1)		/*< Open Drain mode */
} mma8451q_intmode_t;

/**
 * @brief Interrupt polarity
 */
typedef enum {
	MMA8451Q_INTPOL_ACTIVELOW	= (0b0),	/*< active low interrupt */
	MMA8451Q_INTPOL_ACTIVEHIGH	= (0b1)		/*< active high interrupt */
} mma8451q_intpol_t;

/**
 * @brief Interrupt type
 */
typedef enum {
	MMA8451Q_INT_DRDY	= (0x0),	/*< Data Ready */
	MMA8451Q_INT_FFMT	= (0x2),	/*< Freefall/Motion */
	MMA8451Q_INT_PULSE	= (0x3),	/*< Pulse Detection */
	MMA8451Q_INT_LNDPRT	= (0x4),	/*< Landscape/Portrait */
	MMA8451Q_INT_TRANS	= (0x5),	/*< Transient */
	MMA8451Q_INT_FIFO	= (0x6),	/*< FIFO watermark */
	MMA8451Q_INT_ASLP	= (0x7)		/*< Auto-Sleep/Wake */
} mma8451q_interrupt_t;


/**
 * @brief Interrupt pin routing
 */
typedef enum {
	MMA8451Q_INTPIN_INT1	= (0b1),	/*< interrupt routed to INT1 */
	MMA8451Q_INTPIN_INT2	= (0b0)		/*< interrupt routed to INT2 */
} mma8451q_intpin_t;


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
 * @brief The MMA8451Q configuration registers
 */
typedef struct {
	uint8_t F_SETUP;			/*< FIFO setup, register address 0x09 */
	uint8_t TRIG_CFG;			/*< Map of FIFO data capture events, register address 0x0A */
	const uint8_t SYSMOD;		/*< Current System Mode, register address 0x0B */
	/* skipped 1 register */
	const uint8_t WHO_AM_I;		/*< Device ID (0x1A), register address 0x0D */
	uint8_t XYZ_DATA_CFG;		/*< Dynamic Range Settings, register address 0x0E */
	uint8_t HP_FILTER_CUTOFF;	/*< Cutoff frequency, register address 0x0F */
	/* skipped 1 register */
	uint8_t PL_CFG;				/*< Landscape/Portrait configuration., register address 0x11 */
	uint8_t PL_COUNT;			/*< Landscape/Portrait debounce counter, register address 0x12 */
	uint8_t PL_BF_ZCOMP;		/*< Back/Front, Z-Lock Trip threshold, register address 0x13 */
	uint8_t P_L_THS_REG;		/*< Portrait to Landscape Trip Angle, register address 0x14 */
	uint8_t FF_MT_CFG;			/*< Freefall/Motion functional block configuration, register address 0x15 */
	/* skipped 1 register */ 
	uint8_t FF_MT_THS;			/*< Freefall/Motion threshold register, register address 0x17 */
	uint8_t FF_MT_COUNT;		/*< Freefall/Motion debounce counter, register address 0x18 */
	/* skipped 4 registers */
	uint8_t TRANSIENT_CFG;		/*< Transient functional block configuration, register address 0x1D */
	uint8_t TRANSIENT_SCR;		/*< Transient event status register, register address 0x1E */
	uint8_t TRANSIENT_THS;		/*< Transient event threshold, register address 0x1F */
	uint8_t TRANSIENT_COUNT;	/*< Transient debounce counter, register address 0x20 */
	uint8_t PULSE_CFG;			/*< ELE, Double_XYZ or Single_XYZ, register address 0x21 */
	/* skipped 1 register */
	uint8_t PULSE_THSX;			/*< X pulse threshold, register address 0x23 */
	uint8_t PULSE_THSY;			/*< Y pulse threshold, register address 0x24 */
	uint8_t PULSE_THSZ;			/*< Z pulse threshold, register address 0x25 */
	uint8_t PULSE_TMLT;			/*< Time limit for pulse, register address 0x26 */
	uint8_t PULSE_LTCY;			/*< Latency time for 2nd pulse, register address 0x27 */
	uint8_t PULSE_WIND;			/*< Window time for 2nd pulse, register address 0x28 */
	uint8_t ASPL_COUNT;			/*< Counter setting for Auto-SLEEP, register address 0x29 */
	uint8_t CTRL_REG1;			/*< ODR, Active/Standby mode, register address 0x2A */
	uint8_t CTRL_REG2;			/*< Sleep Enable, OS Modes, RST, ST, register address 0x2B */
	uint8_t CTRL_REG3;			/*< Wake from Sleep, IPOL, PP_OD, register address 0x2C */
	uint8_t CTRL_REG4;			/*< Interrupt enable register, register address 0x2D */
	uint8_t CTRL_REG5;			/*< Interrupt pin (INT1/INT2) map, register address 0x2E */
	uint8_t OFF_X;				/*< X-axis offset adjust, register address 0x2F */
	uint8_t OFF_Y;				/*< Y-axis offset adjust, register address 0x30 */
	uint8_t OFF_Z;				/*< Z-axis offset adjust, register address 0x31 */
} mma8451q_confreg_t;

/**
 * @brief Fetches the configuration into a {@see mma8451q_confreg_t} data structure
 * @param[inout] The configuration data data; Must not be null.
 */
void MMA8451Q_FetchConfiguration(mma8451q_confreg_t *const configuration);

/**
 * @brief Stores the configuration from a {@see mma8451q_confreg_t} data structure
 * @param[in] The configuration data data; Must not be null.
 */
void MMA8451Q_StoreConfiguration(const mma8451q_confreg_t *const configuration);

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
uint8_t MMA8451Q_SystemMode();

/**
 * @brief Reads the REG_PL_CFG register from the MMA8451Q.
 * @return Current portrait/landscape mode. 
 */
uint8_t MMA8451Q_LandscapePortraitConfig();

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
 * @brief Sets the data rate and the active mode
 */
void MMA8451Q_SetDataRate(register mma8451q_datarate_t datarate, mma8451q_lownoise_t lownoise);
/**
 * @brief Reads the WHO_AM_I register from the MMA8451Q.
 * @return Device identification code; Should be 0b00011010. 
 */
static inline uint8_t MMA8451Q_WhoAmI()
{
	return I2C_ReadRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_WHOAMI);
}

/**
 * @brief Configures the sensitivity and the high pass filter
 * @param[in] sensitivity The sensitivity
 * @param[in] highpassEnabled Set to 1 to enable the high pass filter or to 0 otherwise (default)
 */
void MMA8451Q_SetSensitivity(mma8451q_sensitivity_t sensitivity, mma8451q_hpo_t highpassEnabled);

/**
 * @brief Enables or disables interrupts
 * @param[in] mode The mode
 * @param[in] polarity The polarity
 */
void MMA8451Q_SetInterruptMode(mma8451q_intmode_t mode, mma8451q_intpol_t polarity);

/**
 * @brief Enables or disables specific interrupts
 * @param[in] irq The interrupt
 * @param[in] pin The pin
 */
void MMA8451Q_ConfigureInterrupt(mma8451q_interrupt_t irq, mma8451q_intpin_t pin);

/**
 * @brief Clears the interrupt configuration
 */
void MMA8451Q_ClearInterruptConfiguration();

/**
 * @brief Configures the oversampling modes
 * @param[in] oversampling The oversampling mode
 */
void MMA8451Q_SetOversampling(mma8451q_oversampling_t oversampling);

#endif /* MMA8451Q_H_ */
