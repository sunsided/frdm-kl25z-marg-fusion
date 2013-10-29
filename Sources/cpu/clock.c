/*
 * clock.c
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#include <stdint.h>
#include "mcg/mcg.h"
#include "cpu/clock.h"

/**
 * @brief Initialises the core clock
 * @return none.
 *
 * \par Will initialise the KL25Z128 clock to 48 MHz (assuming default
 * XTAL of 8 MHz and 1/2*24 prescaler) PEE mode.
 */
void InitClock()
{
	// KUDOS: https://www.youtube.com/watch?v=uiSTB4jkxhw
	
	// NOTE:
	// - core and platform clock are required to have 48 MHz or less
	// - for PBE (PLL engaged external) mode, xtal/divider must be in range 2..4 MHz
	
	const int xtal = XTAL_FREQ;						/* FRDM-KL25Z has an on-board 8 MHz xtal */
	const int8_t divider = XTAL_PEE_DIVIDE;			/* divide by 4 (8 MHz --> 2 MHz) */
	const int8_t multiplier = XTAL_PEE_UPSCALE;		/* scale up by 24 (2 MHz --> 48 MHz) */

	pll_init(xtal, LOW_POWER, CRYSTAL, divider, multiplier, MCGOUT);
	// TODO: assert frequency is correct
}
