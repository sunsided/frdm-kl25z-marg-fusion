/*
 * led.c
 *
 *  Created on: Nov 8, 2013
 *      Author: Markus
 */

#include "derivative.h"
#include "nice_names.h"

#include "cpu/delay.h"
#include "led/led.h"

/**
 * @brief Sets up the GPIOs for LED driving
 */
void LED_Init()
{
	/* Set system clock gating to enable gate to port B */
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	
	/* Set Port B, pin 18 and 19 to GPIO mode */
	PORTB->PCR[18] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; /* not using |= assignment here due to some of the flags being undefined at reset */
	PORTB->PCR[19] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	
	/* Set Port d, pin 1 GPIO mode */
	PORTD->PCR[1] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
		
	/* Data direction for port B, pin 18 and 19 and port D, pin 1 set to output */ 
	GPIOB->PDDR |= GPIO_PDDR_PDD(1<<18) | GPIO_PDDR_PDD(1<<19);
	GPIOD->PDDR |= GPIO_PDDR_PDD(1<<1);
	
	/* disable all leds */
	LED_Off();
}

/**
 * @brief LED Traffic Light!
 */
void TrafficLight()
{
	LED_Red();
	delay_ms(1000);
	LED_Yellow();
	delay_ms(1000);
	LED_Green();
	delay_ms(1000);
}

/**
 * @brief LED Double Flash!
 */
void DoubleFlash()
{
	LED_White();
	delay_ms(50);
	LED_Off();
	delay_ms(50);
	LED_White();
	delay_ms(50);
	LED_Off();	
}
