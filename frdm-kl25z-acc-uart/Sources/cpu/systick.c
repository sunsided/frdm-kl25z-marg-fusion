/*
 * systick.c
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#include "derivative.h"

#include "cpu/clock.h"
#include "cpu/systick.h"

/**
 * @brief Initializes the SysTick interrupt
 * @return none.
 *
 * \par Will initialise the system tick interrupt using interrupt-on-zero
 * utilizing the core clock.
 */
void InitSysTick()
{
	/* see Cortex-M0+ Devices Generic User Guide, Section 4.4 */
	SysTick_BASE_PTR->RVR = CORE_CLOCK/SYSTICK_FREQUENCY - 1;	/* set the reload value */
	SysTick_BASE_PTR->CSR = SysTick_CSR_ENABLE_MASK				/* enable the systick timer */ 
							| SysTick_CSR_TICKINT_MASK 			/* enable interrupt if timer reaches zero */
							| SysTick_CSR_CLKSOURCE_MASK;		/* use processor clock instead of external clock */
}

/**
 * @brief The system tick counter
 */
volatile uint32_t SystemMilliseconds = 0;

/**
 * @brief The SysTick interrupt handler
 * @return none.
 */
void SysTick_Handler() 
{
	++SystemMilliseconds;
}
