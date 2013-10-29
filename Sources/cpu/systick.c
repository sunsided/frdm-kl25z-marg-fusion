/*
 * systick.c
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#include "derivative.h"
#include "cpu/clock.h"
#include "cpu/systick.h"

void InitSysTick()
{
	/* see Cortex-M0+ Devices Generic User Guide, Section 4.4 */
	SYST_RVR = CORE_CLOCK/SYSTICK_FREQUENCY;	/* set the reload value */
	SYST_CSR = 	SysTick_CSR_ENABLE_MASK			/* enable the systick timer */ 
				| SysTick_CSR_TICKINT_MASK 		/* enable interrupt if timer reaches zero */
				| SysTick_CSR_CLKSOURCE_MASK;	/* use processor clock instead of external clock */
}

void SysTick_Handler() 
{
	__asm("bkpt");
}
