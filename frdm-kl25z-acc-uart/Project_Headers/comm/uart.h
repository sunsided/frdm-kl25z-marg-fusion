/*
 * uart.h
 *
 *  Created on: Oct 30, 2013
 *      Author: Markus
 */

#ifndef UART_H_
#define UART_H_

/**
 * @brief Enables or disables decorated storage support using the
 * Bit Manipulation Engine.
 */
#define USE_BME 1

#if USE_BME
#include "bme.h"
#endif

#include "nice_names.h"
#include "buffer.h"

/*
 * @brief The IRQ number (not exception number!) for UART0 interrupt
 */
#define UART0_IRQ		(12)

/*
 * @brief Sets up the UART0 for 115.2 kbaud on PTA1/RX, PTA2/TX using PLL/2 clocking.
 */
void InitUart0();

/**
 * @brief Initializes the interrupt for UART0
 */
void Uart0_InitializeIrq(buffer_t *restrict const readFifo, buffer_t *restrict const writeFifo);

/**
 * @brief Enables the UART0 RX interrupt
 */
static inline void Uart0_EnableReceiveIrq()
{
#if !USE_BME
	UART0->C2 |= UART0_C2_RIE_MASK;
#else
	BME_OR_B(&UART0->C2, (1 << UART0_C2_RIE_SHIFT) & UART0_C2_RIE_MASK);
#endif
}

/**
 * @brief Disables the UART0 RX interrupt
 */
static inline void Uart0_DisableReceiveIrq()
{
#if !USE_BME
	UART0->C2 &= ~UART0_C2_RIE_MASK;
#else
	BME_AND_B(&UART0->C2, (uint8_t)~((1 << UART0_C2_RIE_SHIFT) & UART0_C2_RIE_MASK));
#endif
}

/**
 * @brief Enables the UART0 TX interrupt
 */
static inline void Uart0_EnableTransmitIrq()
{
#if !USE_BME
	UART0->C2 |= UART0_C2_RIE_MASK;
#else
	BME_OR_B(&UART0->C2, (1 << UART0_C2_TIE_SHIFT) & UART0_C2_TIE_MASK);
#endif
}

/**
 * @brief Disables the UART0 TX interrupt
 */
static inline void Uart0_DisableTransmitIrq()
{
#if !USE_BME
	UART0->C2 &= ~UART0_C2_TIE_MASK;
#else
	BME_AND_B(&UART0->C2, (uint8_t)~((1 << UART0_C2_TIE_SHIFT) & UART0_C2_TIE_MASK));
#endif
}

#endif /* UART_H_ */
