/*
 * systick.h
 *
 *  Created on: Oct 29, 2013
 *      Author: Markus
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

/**
* @brief Defines for the system tick behaviour
*/

#define SYSTICK_FREQUENCY		(1000u) /* Hz */

/**
* @brief Function to initialize the SysTick interrupt 
*/
void InitSysTick();

#endif /* SYSTICK_H_ */
