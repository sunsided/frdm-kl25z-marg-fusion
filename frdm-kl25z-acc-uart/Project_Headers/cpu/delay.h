/*
 * delay.h
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#ifndef DELAY_H_
#define DELAY_H_

/**
 * @brief The system tick counter
 */
extern const volatile uint32_t SystemMilliseconds;

/**
 * @brief The system tick counter
 * @param[in] ms The delay time in milliseconds
 * @return none.
 */
void delay_ms(const uint16_t ms) 
{
	const uint32_t start_ticks = SystemMilliseconds;
	do {
		__WFI();
	} while((SystemMilliseconds - start_ticks) < ms);
}

#endif /* DELAY_H_ */
