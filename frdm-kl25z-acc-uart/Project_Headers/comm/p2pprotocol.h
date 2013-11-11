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

/**
 * @brief Begins a P2PPE Transmission
 * @param[in] data The data to send
 * @param[in] dataCount The number of data bytes
 * @param[in] sendHandler The actual send function
 */
void P2PPE_Transmission(register const uint8_t*const data, register uint8_t dataCount, register void (*sendHandler)(uint8_t dataByte));

/**
 * @brief Begins a P2PPE Transmission with a prefix
 * @param[in] data The prefix data to send
 * @param[in] dataCount The number of prefix data bytes
 * @param[in] data The data to send
 * @param[in] dataCount The number of data bytes
 * @param[in] sendHandler The actual send function
 */
void P2PPE_TransmissionPrefixed(register const uint8_t*const prefix, register uint8_t prefixCount, register const uint8_t*const data, register uint8_t dataCount, register void (*sendHandler)(uint8_t dataByte));

#endif /* P2PPROTOCOL_H_ */
