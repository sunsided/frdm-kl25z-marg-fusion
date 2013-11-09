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

#define SOH	0x01 /*< start of header */
#define STX 0x02 /*< start of text */ 
#define ETX 0x03 /*< end of text */
#define EOT 0x04 /*< end of transmission */
#define ESC 0x1B /*< escape */
#define ESC_XOR	0x42 /*< value to escape the data bytes with */

/**
 * @brief Begins a P2PPE Transmission
 * @param[in] data The data to send
 * @param[in] dataCount The number of data bytes
 * @param[in] sendHandler The actual send function

 */
void P2PPE_Transmission(register const uint8_t*const data, register uint8_t dataCount, register void (*sendHandler)(uint8_t dataByte))
{
	/* send the preamble */
	for (int i=0; i<DEFAULT_PREAMBLE_LENGTH; ++i)
	{
		sendHandler(default_preamble[i]);
	}
	
	/* send the header */
	sendHandler(SOH);
	sendHandler(dataCount);
	
	/* send the data frame */
#if 0
	sendHandler(STX);
#endif
	for (int i=0; i<dataCount; ++i)
	{
		register uint8_t byte = data[i];
		
		/* encode special characters */
		if (
#if 0
				SOH == byte || /* SOF is foudn rather often, so it's removed */
				ETX == byte ||
#endif
			   EOT == byte
			|| ESC == byte)
		{
			sendHandler(ESC);
			byte ^= ESC_XOR;
		}
		
		sendHandler(byte);
	}
	
	/* send data frame end */
#if 0
	sendHandler(ETX);
#endif
	sendHandler(EOT);
}
