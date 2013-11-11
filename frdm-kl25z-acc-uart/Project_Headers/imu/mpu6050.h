/*
 * mpu6050.h
 *
 * MPU6050 driver according to register map description rev. 4.3
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "derivative.h"

/**
 * @brief AD0 bit of the I2C slave address of the MPU6050 IMU
 */
#define MPU6050_I2CADDR_AD0	(0b1)

/**
 * @brief I2C slave address of the MPU6050 IMU
 */
#define MPU6050_I2CADDR	(0b1101000 | MPU6050_I2CADDR_AD0)

/**
 * @brief Marker for registers not defined in MPU6000/MPU6050 Register Map document revision 4.0 and 4.3
 * 
 * A description of these registers can be found at the following places:
 * - http://www.i2cdevlib.com/devices/mpu6050#registers
 * - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.h
 *
 * Since they seem to reference register map rev. 2.0 (5/19/2011) which gut superseded
 * by revision 4.3 they are not used by this driver but are included here for completeness.
 */
#define not_in_RM_rev43

#define MPU6050_REG_AUX_VDDIO 			(0x00) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_XG_OFFS_TC 			MPU6050_REG_XG_OFFS_TC
#define MPU6050_REG_YG_OFFS_TC 			(0x01) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_ZG_OFFS_TC 			(0x02) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_X_FINE_GAIN 		(0x03) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_Y_FINE_GAIN 		(0x04) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_Z_FINE_GAIN 		(0x05) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_XA_OFFS_H 			(0x06) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_XA_OFFS_L_TC 		(0x07) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_YA_OFFS_H			(0x08) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_YA_OFFS_L_TC 		(0x09) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_ZA_OFFS_H 			(0x0A) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_ZA_OFFS_L_TC 		(0x0B) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */

#define MPU6050_REG_SELF_TEST_X			(0x0D) /* R/W, reset 0x00 */
#define MPU6050_REG_SELF_TEST_Y			(0x0E) /* R/W, reset 0x00 */
#define MPU6050_REG_SELF_TEST_Z			(0x0F) /* R/W, reset 0x00 */
#define MPU6050_REG_SELF_TEST_A			(0x10) /* R/W, reset 0x00 */

#define MPU6050_REG_XG_OFFS_USRH 		(0x13) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_XG_OFFS_USRL 		(0x14) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_YG_OFFS_USRH 		(0x15) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_YG_OFFS_USRL 		(0x16) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_ZG_OFFS_USRH 		(0x17) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_ZG_OFFS_USRL 		(0x18) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */

#define MPU6050_REG_SMPLRT_DIV 			(0x19) /* R/W, reset 0x00 */
#define MPU6050_REG_CONFIG 				(0x1A) /* R/W, reset 0x00 */
#define MPU6050_REG_GYRO_CONFIG 		(0x1B) /* R/W, reset 0x00 */
#define MPU6050_REG_ACCEL_CONFIG 		(0x1C) /* R/W, reset 0x00 */

#define MPU6050_REG_FF_THR 				(0x1D) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_FF_DUR 				(0x1E) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_MOT_THR 			(0x1F) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_MOT_DUR 			(0x20) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_ZRMOT_THR 			(0x21) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_ZRMOT_DUR 			(0x22) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */

#define MPU6050_REG_FIFO_EN 			(0x23) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_MST_CTRL 		(0x24) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV0_ADDR 		(0x25) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV0_REG 		(0x26) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV0_CTRL 		(0x27) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV1_ADDR 		(0x28) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV1_REG 		(0x29) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV1_CTRL 		(0x2A) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV2_ADDR 		(0x2B) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV2_REG 		(0x2C) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV2_CTRL 		(0x2D) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV3_ADDR 		(0x2E) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV3_REG 		(0x2F) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV3_CTRL 		(0x30) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV4_ADDR 		(0x31) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV4_REG 		(0x32) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV4_DO 		(0x33) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV4_CTRL 		(0x34) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV4_DI 		(0x35) /* R,   reset 0x00 */
#define MPU6050_REG_I2C_MST_STATUS	 	(0x36) /* R,   reset 0x00 */
#define MPU6050_REG_INT_PIN_CFG 		(0x37) /* R/W, reset 0x00 */
#define MPU6050_REG_INT_ENABLE 			(0x38) /* R/W, reset 0x00 */

#define MPU6050_REG_DMP_INT_STATUS 		(0x39) not_in_RM_rev43 /* R, according to http://www.i2cdevlib.com/devices/mpu6050#registers */

#define MPU6050_REG_INT_STATUS 			(0x3A) /* R */
#define MPU6050_REG_ACCEL_XOUT_H 		(0x3B) /* R */
#define MPU6050_REG_ACCEL_XOUT_L 		(0x3C) /* R */
#define MPU6050_REG_ACCEL_YOUT_H 		(0x3D) /* R */
#define MPU6050_REG_ACCEL_YOUT_L 		(0x3E) /* R */
#define MPU6050_REG_ACCEL_ZOUT_H 		(0x3F) /* R */
#define MPU6050_REG_ACCEL_ZOUT_L 		(0x40) /* R */
#define MPU6050_REG_TEMP_OUT_H 			(0x41) /* R */
#define MPU6050_REG_TEMP_OUT_L 			(0x42) /* R */
#define MPU6050_REG_GYRO_XOUT_H 		(0x43) /* R */
#define MPU6050_REG_GYRO_XOUT_L 		(0x44) /* R */
#define MPU6050_REG_GYRO_YOUT_H 		(0x45) /* R */
#define MPU6050_REG_GYRO_YOUT_L 		(0x46) /* R */
#define MPU6050_REG_GYRO_ZOUT_H 		(0x47) /* R */
#define MPU6050_REG_GYRO_ZOUT_L 		(0x48) /* R */
#define MPU6050_REG_EXT_SENS_DATA_00 	(0x49) /* R */
#define MPU6050_REG_EXT_SENS_DATA_01 	(0x4A) /* R */
#define MPU6050_REG_EXT_SENS_DATA_02	(0x4B) /* R */
#define MPU6050_REG_EXT_SENS_DATA_03	(0x4C) /* R */
#define MPU6050_REG_EXT_SENS_DATA_04	(0x4D) /* R */
#define MPU6050_REG_EXT_SENS_DATA_05	(0x4E) /* R */
#define MPU6050_REG_EXT_SENS_DATA_06 	(0x4F) /* R */
#define MPU6050_REG_EXT_SENS_DATA_07 	(0x50) /* R */
#define MPU6050_REG_EXT_SENS_DATA_08	(0x51) /* R */
#define MPU6050_REG_EXT_SENS_DATA_09	(0x52) /* R */
#define MPU6050_REG_EXT_SENS_DATA_10	(0x53) /* R */
#define MPU6050_REG_EXT_SENS_DATA_11 	(0x54) /* R */
#define MPU6050_REG_EXT_SENS_DATA_12	(0x55) /* R */
#define MPU6050_REG_EXT_SENS_DATA_13 	(0x56) /* R */
#define MPU6050_REG_EXT_SENS_DATA_14 	(0x57) /* R */
#define MPU6050_REG_EXT_SENS_DATA_15	(0x58) /* R */
#define MPU6050_REG_EXT_SENS_DATA_16 	(0x59) /* R */
#define MPU6050_REG_EXT_SENS_DATA_17 	(0x5A) /* R */
#define MPU6050_REG_EXT_SENS_DATA_18 	(0x5B) /* R */
#define MPU6050_REG_EXT_SENS_DATA_19 	(0x5C) /* R */
#define MPU6050_REG_EXT_SENS_DATA_20	(0x5D) /* R */
#define MPU6050_REG_EXT_SENS_DATA_21	(0x5E) /* R */
#define MPU6050_REG_EXT_SENS_DATA_22 	(0x5F) /* R */
#define MPU6050_REG_EXT_SENS_DATA_23	(0x60) /* R */

#define MPU6050_REG_MOT_DETECT_STATUS	(0x61) not_in_RM_rev43 /* R, according to http://www.i2cdevlib.com/devices/mpu6050#registers */

#define MPU6050_REG_I2C_SLV0_DO 		(0x63) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV1_DO 		(0x64) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV2_DO 		(0x65) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_SLV3_DO 		(0x66) /* R/W, reset 0x00 */
#define MPU6050_REG_I2C_MST_DELAY_CTRL 	(0x67) /* R/W, reset 0x00 */
#define MPU6050_REG_SIGNAL_PATH_RESET 	(0x68) /* R/W, reset 0x00 */
#define MPU6050_REG_MOT_DETECT_CTRL 	(0x69) /* R/W, reset 0x00 */
#define MPU6050_REG_USER_CTRL 			(0x6A) /* R/W, reset 0x00 */
#define MPU6050_REG_PWR_MGMT_1 			(0x6B) /* R/W, reset 0x40 */
#define MPU6050_REG_PWR_MGMT_2 			(0x6C) /* R/W, reset 0x00 */

#define MPU6050_REG_BANK_SEL 			(0x6D) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_MEM_START_ADDR	 	(0x6E) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_MEM_R_W 			(0x6F) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_DMP_CFG_1 			(0x70) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */
#define MPU6050_REG_DMP_CFG_2 			(0x71) not_in_RM_rev43 /* R/W, according to http://www.i2cdevlib.com/devices/mpu6050#registers */

#define MPU6050_REG_FIFO_COUNTH 		(0x72) /* R/W, reset 0x00 */
#define MPU6050_REG_FIFO_COUNTL 		(0x73) /* R/W, reset 0x00 */
#define MPU6050_REG_FIFO_R_W 			(0x74) /* R/W, reset 0x00 */
#define MPU6050_REG_WHO_AM_I 			(0x75) /* R,   reset 0x68 */

#undef not_in_RM_rev43

/**
 * MPU6050 selftest data registers
 */
#pragma pack(1)
typedef struct __attribute__ ((__packed__))
{
	const uint8_t SELF_TEST_X;		/* 0x0D */
	const uint8_t SELF_TEST_Y;		/* 0x0E */
	const int8_t SELF_TEST_Z;		/* 0x0F */
	uint8_t SELF_TEST_A;		/* 0x10 */
} mpu6050_selftest_t;

/**
 * @brief The MPU6050 configuration registers
 */
#pragma pack(1)
typedef struct __attribute__ ((__packed__))
{
	/* skipping registers 0x00 .. 0x0C */
	
	/* skipping self-test registers 0x0D .. 0x10 */
	
	/* skipping registers 0x11 .. 0x18 */
	
	uint8_t SMPLRT_DIV; 		/* 0x19 */
	uint8_t CONFIG; 			/* 0x1A */
	uint8_t GYRO_CONFIG; 		/* 0x1B */
	uint8_t ACCEL_CONFIG; 		/* 0x1C */
	
	/* skipping registers 0x1D .. 0x12 */
	
	uint8_t FIFO_EN; 			/* 0x23 */
	uint8_t I2C_MST_CTRL; 		/* 0x24 */
	uint8_t I2C_SLV0_ADDR; 		/* 0x25 */
	uint8_t I2C_SLV0_REG; 		/* 0x26 */
	uint8_t I2C_SLV0_CTRL; 		/* 0x27 */
	uint8_t I2C_SLV1_ADDR; 		/* 0x28 */
	uint8_t I2C_SLV1_REG; 		/* 0x29 */
	uint8_t I2C_SLV1_CTRL; 		/* 0x2A */
	uint8_t I2C_SLV2_ADDR; 		/* 0x2B */
	uint8_t I2C_SLV2_REG; 		/* 0x2C */
	uint8_t I2C_SLV2_CTRL; 		/* 0x2D */
	uint8_t I2C_SLV3_ADDR; 		/* 0x2E */
	uint8_t I2C_SLV3_REG; 		/* 0x2F */
	uint8_t I2C_SLV3_CTRL; 		/* 0x30 */
	uint8_t I2C_SLV4_ADDR; 		/* 0x31 */
	uint8_t I2C_SLV4_REG; 		/* 0x32 */
	uint8_t I2C_SLV4_DO; 		/* 0x33 */
	uint8_t I2C_SLV4_CTRL; 		/* 0x34 */
	const uint8_t I2C_SLV4_DI; 		/* 0x35 */
	const uint8_t I2C_MST_STATUS;	 /* 0x36 */
	uint8_t INT_PIN_CFG; 		/* 0x37 */
	uint8_t INT_ENABLE; 		/* 0x38 */

	/* skipping register 0x39 */
	
	/* skipping data status and value registers 0x3A .. 0x60 */
	
	/* skipping registers 0x61 .. 0x63 */
	
	uint8_t I2C_SLV0_DO; 		/* 0x63 */
	uint8_t I2C_SLV1_DO; 		/* 0x64 */
	uint8_t I2C_SLV2_DO; 		/* 0x65 */
	uint8_t I2C_SLV3_DO; 		/* 0x66 */
	uint8_t I2C_MST_DELAY_CTRL; /* 0x67 */
	uint8_t SIGNAL_PATH_RESET; 	/* 0x68 */
	uint8_t MOT_DETECT_CTRL; 	/* 0x69 */
	uint8_t USER_CTRL; 			/* 0x6A */
	uint8_t PWR_MGMT_1; 		/* 0x6B */
	uint8_t PWR_MGMT_2; 		/* 0x6C */

	/* skipping registers 0x6D .. 0x71 */
	
	uint8_t FIFO_COUNTH; 		/* 0x72 */
	uint8_t FIFO_COUNTL; 		/* 0x73 */
	uint8_t FIFO_R_W; 			/* 0x74 */
	const uint8_t WHO_AM_I; 	/* 0x75 */
} mpu6050_confreg_t;

/**
 * @brief MPU6050 internal and external sensor data registers
 */
#pragma pack(1)
typedef struct __attribute__ ((__packed__))
{
	uint8_t INT_STATUS; 		/* 0x3A */
	uint8_t ACCEL_XOUT_H; 		/* 0x3B */
	uint8_t ACCEL_XOUT_L; 		/* 0x3C */
	uint8_t ACCEL_YOUT_H; 		/* 0x3D */
	uint8_t ACCEL_YOUT_L; 		/* 0x3E */
	uint8_t ACCEL_ZOUT_H; 		/* 0x3F */
	uint8_t ACCEL_ZOUT_L; 		/* 0x40 */
	uint8_t TEMP_OUT_H; 		/* 0x41 */
	uint8_t TEMP_OUT_L; 		/* 0x42 */
	uint8_t GYRO_XOUT_H; 		/* 0x43 */
	uint8_t GYRO_XOUT_L; 		/* 0x44 */
	uint8_t GYRO_YOUT_H; 		/* 0x45 */
	uint8_t GYRO_YOUT_L; 		/* 0x46 */
	uint8_t GYRO_ZOUT_H; 		/* 0x47 */
	uint8_t GYRO_ZOUT_L; 		/* 0x48 */
} mpu6050_intdatareg_t;

/**
 * @brief MPU6050 internal and external sensor data registers
 */
#pragma pack(1)
typedef struct __attribute__ ((__packed__))
{
	mpu6050_intdatareg_t internalData; /* 0x3A .. 0x48 */
	uint8_t EXT_SENS_DATA_00; 	/* 0x49 */
	uint8_t EXT_SENS_DATA_01; 	/* 0x4A */
	uint8_t EXT_SENS_DATA_02;	/* 0x4B */
	uint8_t EXT_SENS_DATA_03;	/* 0x4C */
	uint8_t EXT_SENS_DATA_04;	/* 0x4D */
	uint8_t EXT_SENS_DATA_05;	/* 0x4E */
	uint8_t EXT_SENS_DATA_06; 	/* 0x4F */
	uint8_t EXT_SENS_DATA_07; 	/* 0x50 */
	uint8_t EXT_SENS_DATA_08;	/* 0x51 */
	uint8_t EXT_SENS_DATA_09;	/* 0x52 */
	uint8_t EXT_SENS_DATA_10;	/* 0x53 */
	uint8_t EXT_SENS_DATA_11; 	/* 0x54 */
	uint8_t EXT_SENS_DATA_12;	/* 0x55 */
	uint8_t EXT_SENS_DATA_13; 	/* 0x56 */
	uint8_t EXT_SENS_DATA_14; 	/* 0x57 */
	uint8_t EXT_SENS_DATA_15;	/* 0x58 */
	uint8_t EXT_SENS_DATA_16; 	/* 0x59 */
	uint8_t EXT_SENS_DATA_17; 	/* 0x5A */
	uint8_t EXT_SENS_DATA_18; 	/* 0x5B */
	uint8_t EXT_SENS_DATA_19; 	/* 0x5C */
	uint8_t EXT_SENS_DATA_20;	/* 0x5D */
	uint8_t EXT_SENS_DATA_21;	/* 0x5E */
	uint8_t EXT_SENS_DATA_22; 	/* 0x5F */
	uint8_t EXT_SENS_DATA_23;	/* 0x60 */	
} mpu6050_fulldatareg_t;

/**
 * @brief Macro for usage in configuration commands that executes configuration changes directly.
 */
#define MPU6050_CONFIGURE_DIRECT ((mpu6050_confreg_t*)0x0)

/**
 * @brief Reads the WHO_AM_I register from the MPU6050.
 * @return Device identification code; Should be 0b0110100 (0x68)
 */
uint8_t MPU6050_WhoAmI();

/**
 * @brief Fetches the configuration into a {@see mpu6050_confreg_t} data structure
 * @param[inout] The configuration data data; Must not be null.
 */
void MPU6050_FetchConfiguration(mpu6050_confreg_t *const configuration);

/**
 * @brief Stores the configuration from a {@see mpu6050_confreg_t} data structure
 * @param[in] The configuration data data; Must not be null.
 */
void MPU6050_StoreConfiguration(const mpu6050_confreg_t *const configuration);

/**
 * @brief Configures the gyro sample rate divider
 * @param[inout] configuration The configuration structure or {@see MPU6050_CONFIGURE_DIRECT} if changes should be sent directly over the wire.
 * @param[in] divider The divider in a range of 1..255. If zero is given, a value of 1 will be used.
 *
 * Sample Rate = Gyroscope Output Rate / divider
 */
void MPU6050_SetGyroscopeSampleRateDivider(mpu6050_confreg_t *const configuration, uint8_t divider);

/**
 * @brief Scale configuration for the gyroscope
 */
typedef enum {
	MPU6050_GYRO_FS_250  = (0b00),	/*! +/-  250 °/s */
	MPU6050_GYRO_FS_500  = (0b01),	/*! +/-  500 °/s */
	MPU6050_GYRO_FS_1000 = (0b10),	/*! +/- 1000 °/s */
	MPU6050_GYRO_FS_2000 = (0b11)	/*! +/- 2000 °/s */
} mpu6050_gyro_fs_t;

/**
 * @brief Configures the gyroscope full scale range
 * @param[inout] configuration The configuration structure or {@see MPU6050_CONFIGURE_DIRECT} if changes should be sent directly over the wire.
 * @param[in] fullScale The full scale
 */
void MPU6050_SetGyroscopeFullScale(mpu6050_confreg_t *const configuration, mpu6050_gyro_fs_t fullScale);

/**
 * @brief Scale configuration for the accelerometer
 */
typedef enum {
	MPU6050_ACC_FS_2  = (0b00),	/*! +/-  2g */
	MPU6050_ACC_FS_4  = (0b01),	/*! +/-  4g */
	MPU6050_ACC_FS_8  = (0b10),	/*! +/-  8g */
	MPU6050_ACC_FS_16 = (0b11)	/*! +/- 16g */
} mpu6050_acc_fs_t;

/**
 * @brief Configures the accelerometer full scale range
 * @param[inout] configuration The configuration structure or {@see MPU6050_CONFIGURE_DIRECT} if changes should be sent directly over the wire.
 * @param[in] fullScale The full scale
 */
void MPU6050_SetAccelerometerFullScale(mpu6050_confreg_t *const configuration, mpu6050_acc_fs_t fullScale);

/**
 * @brief Interrupt level configuration
 */
typedef enum {
	MPU6050_INTLEVEL_ACTIVEHIGH = (0b0),	/*! Interrupts are active high */
	MPU6050_INTLEVEL_ACTIVELOW  = (0b1),	/*! Interrupts are active low */
} mpu6050_intlevel_t;

/**
 * @brief Interrupt wire mode configuration
 */
typedef enum {
	MPU6050_INTOPEN_PUSHPULL 	= (0b0),	/*! Interrupts are push-pull */
	MPU6050_INTOPEN_OPENDRAIN  	= (0b1),	/*! Interrupts are open drain */
} mpu6050_intopen_t;

/**
 * @brief Interrupt latch configuration
 */
typedef enum {
	MPU6050_INTLATCH_PULSE 		= (0b0),	/*! Interrupt pin emits pulses of 50us duration */
	MPU6050_INTLATCH_LATCHED  	= (0b1),	/*! Interrupt pin is held high until interrupt is cleared */
} mpu6050_intlatch_t;

/**
 * @brief Interrupt clear configuration
 */
typedef enum {
	MPU6050_INTRDCLEAR_READANY  	= (0b0),	/*! Interrupts are cleared by any read */
	MPU6050_INTRDCLEAR_READSTATUS 	= (0b1),	/*! Interrupts are cleared by a read to the status register */
} mpu6050_intrdclear_t;

/**
 * @brief Configures the interrupts
 * @param[inout] configuration The configuration structure or {@see MPU6050_CONFIGURE_DIRECT} if changes should be sent directly over the wire.
 * @param[in] level The interrupt level
 * @param[in] type The interrupt type
 * @param[in] type The interrupt latch type
 * @param[in] clear The interrupt clear configuration
 */
void MPU6050_ConfigureInterrupts(mpu6050_confreg_t *const configuration, mpu6050_intlevel_t level, mpu6050_intopen_t type, mpu6050_intlatch_t latch, mpu6050_intrdclear_t clear);

/**
 * @brief Interrupt source configuration
 */
typedef enum {
	MPU6050_INT_DISABLED = (0b0),	/*! Interrupt source is disabled */
	MPU6050_INT_ENABLED = (0b1)		/*! Interrupt source is enabled */
} mpu6050_inten_t;

/**
 * @brief Configures the interrupt source
 * @param[inout] configuration The configuration structure or {@see MPU6050_CONFIGURE_DIRECT} if changes should be sent directly over the wire.
 * @param[in] fifoOverflow Interrupts from FIFO overflows
 * @param[in] i2cMaster Interrupts from I2C master activity
 * @param[in] dataReady Interrupts from data ready events
 */
void MPU6050_EnableInterrupts(mpu6050_confreg_t *const configuration, mpu6050_inten_t fifoOverflow, mpu6050_inten_t i2cMaster, mpu6050_inten_t dataReady);

/**
 * @brief Clock source configuration
 */
typedef enum {
	MPU6050_CLOCK_8MHZOSC		= (0b000), /*! Uses the internal 8MHz oscillator */ 
	MPU6050_CLOCK_XGYROPLL		= (0b001), /*! PLL with X axis gyroscope reference */
	MPU6050_CLOCK_YGYROPLL		= (0b010), /*! PLL with Y axis gyroscope reference */
	MPU6050_CLOCK_ZGYROPLL		= (0b011), /*! PLL with Z axis gyroscope reference */
	MPU6050_CLOCK_EXT32KHZPLL	= (0b100), /*! PLL with external 32.768kHz reference */
	MPU6050_CLOCK_EXT19MHZPLL	= (0b101), /*! PLL with external 19.2MHz reference */
	MPU6050_CLOCK_STOPPED		= (0b111), /*! Stops the clock and keeps the timing generator in reset */
} mpu6050_clock_t;

/**
 * @brief Configures the internal clock source
 * @param[inout] configuration The configuration structure or {@see MPU6050_CONFIGURE_DIRECT} if changes should be sent directly over the wire.
 * @param[in] source The clock source
 */
void MPU6050_SelectClockSource(mpu6050_confreg_t *const configuration, mpu6050_clock_t source);

/**
 * @brief Sleep mode configuration
 */
typedef enum {
	MPU6050_SLEEP_DISABLED		= (0b0),
	MPU6050_SLEEP_ENABLED		= (0b1),
} mpu6050_sleep_t;

/**
 * @brief Configures the internal clock source
 * @param[inout] configuration The configuration structure or {@see MPU6050_CONFIGURE_DIRECT} if changes should be sent directly over the wire.
 * @param[in] mode The sleep mode
 */
void MPU6050_SetSleepMode(mpu6050_confreg_t *const configuration, mpu6050_sleep_t mode);

/**
 * @brief Accelerometer, gyroscope and temperature data
 */
#pragma pack(1)
typedef struct __attribute__ ((__packed__))
{
	uint8_t :8; 		/*! padding byte */
	uint8_t status;		/*! the status register contents */
	union {
		struct {
			int16_t x;	/*! the x acceleration */
			int16_t y;	/*! the x acceleration */
			int16_t z;	/*! the x acceleration */
		};
		int16_t xyz[3];	/*! xyz array of accelerations */
	} accel;			/*! The accelerometer data */
	union {
		struct {
			int16_t x;	/*! the x angular velocity */
			int16_t y;	/*! the y angular velocity */
			int16_t z;	/*! the z angular velocity */
		};
		int16_t xyz[3];	/*! xyz array of angular velocities */
	} gyro;
	int16_t temperature;/*! the temperature sensor output */
} mpu6050_sensor_t;

/**
 * @brief Reads accelerometer, gyro and temperature data from the MPU6050
 * @param[inout] data The data 
 */
void MPU6050_ReadData(mpu6050_sensor_t *data);

#endif /* MPU6050_H_ */
