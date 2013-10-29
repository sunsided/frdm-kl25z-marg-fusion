/*
 * clock.h
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#ifndef CLOCK_H_
#define CLOCK_H_

/**
* @brief Macros defining the XTAL frequency and core clock
*/

#define XTAL_FREQ			(8000000u) 	/* Hz, FRDM-KL25Z has an on-board 8 MHz xtal */
#define XTAL_PEE_DIVIDE		(4u)		/* divide by 4 (8 MHz --> 2 MHz) */
#define XTAL_PEE_UPSCALE	(24u)		/* scale up by 24 (2 MHz --> 48 MHz) */

#define CORE_CLOCK			(XTAL_FREQ/XTAL_PEE_DIVIDE*XTAL_PEE_UPSCALE) /* Hz */

/**
* @brief Function to initialize the core clock 
*/
void InitClock();

#endif /* CLOCK_H_ */
