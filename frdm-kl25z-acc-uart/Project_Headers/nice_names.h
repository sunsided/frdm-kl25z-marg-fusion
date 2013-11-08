/*
 * nice_names.h
 *
 *  Created on: Oct 30, 2013
 *      Author: Markus
 */

#ifndef NICE_NAMES_H_
#define NICE_NAMES_H_

/**
 * @brief Helper macro for {@see TOKENPASTE(a,b)} to merge token
 */
#define TOKENPASTE_HELPER(x, y) x ## y

/**
 * @brief Merges two tokens, expanding any macros 
 */
#define TOKENPASTE(x, y) TOKENPASTE_HELPER(x, y)

/**
 * @brief Macro to mark variables that are unused by intention
 */
#define INTENTIONALLY_UNUSED(type) type __attribute__((unused)) TOKENPASTE(unused, __COUNTER__)

/**
 * @brief NULL address
 */
#ifndef NULL
#define NULL	(0x0)
#endif

/**
 * @brief Assertion
 */
#ifndef assert
#define assert(condition) if (!(condition)) { while(1) {} }
#define assert_not_null(pointer) if ((void*)0 == (void*)(pointer)) { while(1) {} }
#endif

#define SIM 	SIM_BASE_PTR
#define PORTA	PORTA_BASE_PTR
#define PORTB	PORTB_BASE_PTR
#define PORTC	PORTC_BASE_PTR
#define PORTD	PORTD_BASE_PTR
#define PORTE	PORTE_BASE_PTR
#define GPIOA	FPTA_BASE_PTR /* fast GPIO using core IOPORT mapped registers */
#define GPIOB	FPTB_BASE_PTR /* fast GPIO using core IOPORT mapped registers */
#define GPIOD	FPTD_BASE_PTR /* fast GPIO using core IOPORT mapped registers */

#define UART0	UART0_BASE_PTR
#define I2C0	I2C0_BASE_PTR

#endif /* NICE_NAMES_H_ */
