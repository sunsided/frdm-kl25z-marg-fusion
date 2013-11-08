/*
 * led.h
 *
 *  Created on: Nov 8, 2013
 *      Author: Markus
 */

#ifndef LED_H_
#define LED_H_

#include "ARMCM0plus.h"
#include "derivative.h"
#include "nice_names.h"

/**
 * @brief Sets up the GPIOs for LED driving
 */
void LED_Init();

/**
 * @brief Enables the red LED
 */
#define LED_RedOn() 		GPIOB->PCOR  = 1<<18

/**
 * @brief Disables the red LED
 */
#define LED_RedOff() 		GPIOB->PSOR  = 1<<18

/**
 * @brief Enables the green LED
 */
#define LED_GreenOn() 		GPIOB->PCOR  = 1<<19

/**
 * @brief Disables the green LED
 */
#define LED_GreenOff() 		GPIOB->PSOR  = 1<<19

/**
 * @brief Enables the blue LED
 */
#define LED_BlueOn() 		GPIOD->PCOR  = 1<<1

/**
 * @brief Disables the blue LED
 */
#define LED_BlueOff() 		GPIOD->PSOR  = 1<<1

/**
 * @brief Lights the red LED
 */
static inline void LED_Red()
{
	LED_RedOn();
	LED_GreenOff();
	LED_BlueOff();
}

/**
 * @brief Lights the green LED
 */
static inline void LED_Green()
{
	LED_RedOff();
	LED_GreenOn();
	LED_BlueOff();
}

/**
 * @brief Lights the blue LED
 */
static inline void LED_Blue()
{
	LED_RedOff();
	LED_GreenOn();
	LED_BlueOff();
}

/**
 * @brief Lights the red and green LED
 */
static inline void LED_Yellow()
{
	LED_RedOn();
	LED_GreenOn();
	LED_BlueOff();
}

/**
 * @brief Lights the red and blue LED
 */
static inline void LED_Magenta()
{
	LED_RedOn();
	LED_GreenOff();
	LED_BlueOn();
}

/**
 * @brief Lights the blue and green LED
 */
static inline void LED_Cyan()
{
	LED_RedOff();
	LED_GreenOn();
	LED_BlueOn();
}

/**
 * @brief Lights all LEDs
 */
static inline void LED_White()
{
	LED_RedOn();
	LED_GreenOn();
	LED_BlueOn();
}

/**
 * @brief Disables all LEDs
 */
static inline void LED_Off()
{
	LED_RedOff();
	LED_GreenOff();
	LED_BlueOff();
}

#endif /* LED_H_ */
