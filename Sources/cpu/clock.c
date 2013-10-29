/*
 * clock.c
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#include <stdint.h>
#include "mcg/mcg.h"
#include "cpu/clock.h"

void InitClock()
{
	// KUDOS: https://www.youtube.com/watch?v=uiSTB4jkxhw
	
	// NOTE:
	// - core and platform clock are required to have 48 MHz or less
	// - for PBE (PLL engaged external) mode, xtal/divider must be in range 2..4 MHz
	
	const int xtal = 8000000;			// FRDM-KL25Z has an on-board 8 MHz xtal
	const int8_t divider = 4;			// divide by 4 (8 MHz --> 2 MHz)
	const int8_t multiplier = 24;		// scale up by 24 (2 MHz --> 48 MHz)

	pll_init(xtal, LOW_POWER, CRYSTAL, divider, multiplier, MCGOUT);
	// TODO: assert frequency is correct
}
