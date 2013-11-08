/*
 * p2pprotocol.c
 *
 *  Created on: Nov 8, 2013
 *      Author: Markus
 */

#include "nice_names.h"
#include "comm/p2pprotocol.h"

/**
 * @brief The length of the default preamble
 */
#define DEFAULT_PREAMBLE_LENGTH (2)

/**
 * @brief The default preamble 
 */
static const uint8_t default_preamble[DEFAULT_PREAMBLE_LENGTH] = {0xDA, 0x7A};

/**
 * @brief Begins a P2PPE Transmission
 * @param[inout] control The control structure
 * @param[in] 	dataCount The number of data bytes
 * @param[in]	sendHandler The actual send function
 * @param[in]	doneHandle The handler that will be called when the transfer is completed. 
 */
void P2PPE_BeginTransmission(p2pp_encoder_t *const control, const uint8_t dataCount, void (*sendHandler)(uint8_t dataByte), void (*doneHandler)(void))
{
	assert_not_null(control);
	
	*(uint8_t*)&control->totalBytes = dataCount;
	control->remainingBytes = dataCount;
	control->sendHandler = sendHandler;
	control->doneHandler = doneHandler;
	
	/* assign default preamble */
	register uint8_t* ptrToDefaultPr = (uint8_t*)default_preamble; 
	register uint8_t** ptrToField = (uint8_t**)&control->preamble; 
	*ptrToField = ptrToDefaultPr;

	*(uint8_t*)&control->preambleLength = DEFAULT_PREAMBLE_LENGTH;
}

/**
 * @brief Begins a P2PPE Transmission
 * @param[inout] control The control structure
 * @param[in]	sendHandler The actual send function
 * @param[in]	doneHandle	The handler that will be called when the transfer is completed. 
 */
void P2PPE_SendByte(p2pp_encoder_t *const control, uint8_t byte)
{
}
