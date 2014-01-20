/*
 * hmc5883l.h
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "derivative.h"
#include "nice_names.h"

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
 * @brief The HMC5883L configuration registers
 */
typedef struct {
	uint8_t CRA;	/*! Configuration Register A */
	uint8_t CRB;	/*! Configuration Register B */
	uint8_t MR;		/*! Mode Register */
} hmc5883l_confreg_t;

/**
 * @brief Describes the sample averaging mode
 */
typedef enum {
	HMC5883L_MA_1 = (0b00),	/*! Averaging of 1 sample (no averaging) */
	HMC5883L_MA_2 = (0b01),	/*! Averaging of 2 samples */
	HMC5883L_MA_4 = (0b10),	/*! Averaging of 4 samples */
	HMC5883L_MA_8 = (0b11),	/*! Averaging of 8 samples */
} hmc5883l_ma_t;

/**
 * @brief Describes the output rate
 */
typedef enum {
	HMC5883L_DO_0p7Hz  	= (0b000),	/*! 0.75 Hz output rate */
	HMC5883L_DO_1p5Hz  	= (0b001),	/*! 1.5 Hz output rate */
	HMC5883L_DO_3Hz		= (0b010),	/*! 3 Hz output rate */
	HMC5883L_DO_7p5Hz	= (0b011),	/*! 7.5 Hz output rate */
	HMC5883L_DO_15Hz	= (0b100),	/*! 15 Hz output rate (default) */
	HMC5883L_DO_30Hz	= (0b101),	/*! 30 Hz output rate */
	HMC5883L_DO_75Hz	= (0b110),	/*! 75 Hz output rate */
} hmc5883l_do_t;


/**
 * @brief Describes the measurement mode
 */
typedef enum {
	HMC5883L_MS_NORMAL  = (0b00),	/*! Normal measurement configuration */
	HMC5883L_MS_POSBIAS = (0b01),	/*! Positive bias configuration */
	HMC5883L_MS_NEGBIAS = (0b10),	/*! Negative bias configuration */
} hmc5883l_ms_t;

/**
 * @brief Gain configuration
 */
typedef enum {
	HMC5883L_GN_1370_0p88Ga	= (0b000), /*! 1370 LSB/Gauss (+/- 0.88 Ga sensor field range)*/
	HMC5883L_GN_1090_1p3Ga	= (0b001), /*! 1090 LSB/Gauss (+/- 1.3 Ga sensor field range, default)*/
	HMC5883L_GN_820_1p9Ga	= (0b010), /*! 820 LSB/Gauss (+/- 1.9 Ga sensor field range)*/
	HMC5883L_GN_660_2p5Ga	= (0b011), /*! 660 LSB/Gauss (+/- 2.5 Ga sensor field range)*/
	HMC5883L_GN_440_4p0Ga	= (0b100), /*! 440 LSB/Gauss (+/- 4.0 Ga sensor field range)*/
	HMC5883L_GN_390_4p7Ga	= (0b101), /*! 390 LSB/Gauss (+/- 4.7 Ga sensor field range)*/
	HMC5883L_GN_330_5p6Ga	= (0b110), /*! 330 LSB/Gauss (+/- 5.6 Ga sensor field range)*/
	HMC5883L_GN_230_8p1Ga	= (0b111), /*! 230 LSB/Gauss (+/- 8.1 Ga sensor field range)*/
} hmc5883l_gain_t;

/**
 * @brief Operation mode
 */
typedef enum {
	HMC5883L_MD_CONT 		= (0b00), 	/*! Continuous measurement mode */
	HMC5883L_MD_SINGLE 		= (0b01),	/*! Single-measurement mode, default */
	HMC5883L_MD_IDLE 		= (0b11), 	/*! Idle mode */
} hmc5883l_mode_t;

/**
 * @brief Measurement data structure
 */
typedef struct {
	uint8_t :8;
	uint8_t status;
	union {
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
		uint16_t xyz[3];
	};
} hmc5883l_data_t;

/**
 * @brief Reads the Identification registers from the HMC5883L.
 * @return Device identification code; Should be 0x00483433 ('\0H43' sequential Memory!)
 */
uint32_t HMC5883L_Identification();

/**
 * @brief Fetches the HMC5883L configuration
 * @param[inout] configuration The configuration
 */
void HMC5883L_FetchConfiguration(hmc5883l_confreg_t *const configuration);

/**
 * @brief Stores the HMC5883L configuration
 * @param[in] configuration The configuration
 */
void HMC5883L_StoreConfiguration(const hmc5883l_confreg_t *const configuration);

/**
 * Sets the averaging mode
 * @param[inout] configuration The configuration
 * @param[in] averaging The averaging mode
 */
void HMC5883L_SetAveraging(hmc5883l_confreg_t *const configuration, register hmc5883l_ma_t averaging);

/**
 * Sets the output rate
 * @param[inout] configuration The configuration
 * @param[in] rate The output rate
 */
void HMC5883L_SetOutputRate(hmc5883l_confreg_t *const configuration, register hmc5883l_do_t rate);

/**
 * Sets the measurement mode
 * @param[inout] configuration The configuration
 * @param[in] mode The measurement mode
 */
void HMC5883L_SetMeasurementMode(hmc5883l_confreg_t *const configuration, register hmc5883l_ms_t mode);

/**
 * Sets the gain
 * @param[inout] configuration The configuration
 * @param[in] gain The sensor gain
 */
void HMC5883L_SetGain(hmc5883l_confreg_t *const configuration, register hmc5883l_gain_t gain);

/**
 * Sets the operating mode
 * @param[inout] configuration The configuration
 * @param[in] mode The operating mode
 */
void HMC5883L_SetOperatingMode(hmc5883l_confreg_t *const configuration, register hmc5883l_mode_t mode);


/**
 * @brief Fetches the data from the HMC5883L
 * @param[inout] data The sensor data
 */
void HMC5883L_ReadData(hmc5883l_data_t *const data);


/**
* @brief Prepares a data buffer by clearing its values.
* @param[inout] data The data buffer to clear.
*/
static inline void HMC5883L_InitializeData(hmc5883l_data_t *data)
{
    assert_not_null(data);
    data->status = 0;
    data->x = 0;
    data->y = 0;
    data->z = 0;
}

#endif /* HMC5883L_H_ */
