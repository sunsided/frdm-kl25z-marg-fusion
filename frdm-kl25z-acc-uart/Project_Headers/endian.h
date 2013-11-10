/*
 * endian.h
 *
 *  Created on: Nov 6, 2013
 *      Author: Markus
 */

#ifndef ENDIAN_H_
#define ENDIAN_H_

#include "derivative.h"

/**
 * @brief Swaps high- and low-byte of a 16bit integer
 */
#define ENDIANSWAP_16(x)		( (((x) >> 8) & 0x00FF) | (((x)& 0x00FF) << 8) )

/**
 * @brief Endian-Swaps bytes of a 32bit integer
 */
#define ENDIANSWAP_32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))

/**
 * @brief Endianness
 */
typedef enum {
	FROM_LITTLE_ENDIAN = 0,							/*< little-endian */
	FROM_BIG_ENDIAN = SCB_AIRCR_ENDIANNESS_MASK		/*< big-endian */
} endian_t;

/**
 * @brief Determines if endian correction to machine endianness is required given a source endianness
 * @param[in] sourcEndianness The endianness of the value
 * @return zero if endianness is same, nonzero otherwise 
 */
static inline uint8_t endianCorrectionRequired(const register endian_t sourceEndianness) 
{
	/* if both are equal, subtraction returns in zero (== not required) */
	return sourceEndianness != (SCB_AIRCR & SCB_AIRCR_ENDIANNESS_MASK); 
}

/**
 * @brief Converts from a given endianness to machine endianness
 * @param[in] value The value to convert
 * @param[in] sourcEndianness The endianness of the value
 */
static inline uint16_t endianCorrect16(register uint16_t value, const register endian_t sourceEndianness) 
{
	/* if in little endian mode */
	if (sourceEndianness != (SCB_AIRCR & SCB_AIRCR_ENDIANNESS_MASK)) 
	{
		return ENDIANSWAP_16(value);
	}
	
	/* finally or if in big endian mode */
	return value;
}

/**
 * @brief Converts from a given endianness to machine endianness
 * @param[in] value The value to convert
 * @param[in] sourcEndianness The endianness of the value
 */
static inline uint32_t endianCorrect32(register uint32_t value, const register endian_t sourceEndianness) 
{
	/* if in little endian mode */
	if (sourceEndianness != (SCB_AIRCR & SCB_AIRCR_ENDIANNESS_MASK)) 
	{
		return ENDIANSWAP_32(value);
	}
	
	/* finally or if in big endian mode */
	return value;
}

#endif /* ENDIAN_H_ */
