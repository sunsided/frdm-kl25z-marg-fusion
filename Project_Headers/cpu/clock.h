/*
 * clock.h
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#define XTAL_FREQ			(8000000u) /* Hz */
#define XTAL_PEE_DIVIDE		(4u)
#define XTAL_PEE_UPSCALE	(24u)

#define CORE_CLOCK			(XTAL_FREQ/XTAL_PEE_DIVIDE*XTAL_PEE_UPSCALE) /* Hz */

void InitClock();

#endif /* CLOCK_H_ */
