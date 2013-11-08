/*
 * p2pprotocol.h
 *
 * Naive point-to-point data protocol encoder
 *
 *  Created on: Nov 8, 2013
 *      Author: Markus
 */

#ifndef P2PPROTOCOL_H_
#define P2PPROTOCOL_H_

#include "derivative.h"
#include <stdint.h>

typedef enum {
	PREAMBLEn,
	HEADER,
	SOH,
	STX,
	DATAn,
	ETX,
	EOT,
} p2pp_encoder_state_t;

/**
 * @brief Control structure for the point-to-point protocol encoder
 */
typedef struct 
{
	void (*doneHandler)(void);
	void (*sendHandler)(uint8_t dataByte);
	const uint8_t totalBytes;
	uint8_t remainingBytes;
	const uint8_t*const preamble;
	const uint8_t preambleLength;
} p2pp_encoder_t;

/**
 * @brief Begins a P2PPE Transmission
 * @param[inout] control The control structure
 * @param[in] 	dataCount The number of data bytes
 * @param[in]	sendHandler The actual send function
 * @param[in]	doneHandle The handler that will be called when the transfer is completed. 
 */
void P2PPE_BeginTransmission(p2pp_encoder_t *const control, uint8_t dataCount, void (*sendHandler)(uint8_t dataByte), void (*doneHandler)(void));

/**
 * @brief Begins a P2PPE Transmission
 * @param[inout] control The control structure
 * @param[in]	sendHandler The actual send function
 * @param[in]	doneHandle	The handler that will be called when the transfer is completed. 
 */
void P2PPE_SendByte(p2pp_encoder_t *const control, uint8_t byte);

#endif /* P2PPROTOCOL_H_ */
