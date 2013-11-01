/*
 * uart.h
 *
 *  Created on: Oct 30, 2013
 *      Author: Markus
 */

#ifndef UART_H_
#define UART_H_

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
void Uart0_InitializeIrq(buffer_t* readFifo, buffer_t* writeFifo);

/**
 * @brief Enables the UART0 RX interrupt
 */
static inline void Uart0_EnableReceiveIrq()
{
	UART0_C2 |= UART0_C2_RIE_MASK;
}

/**
 * @brief Disables the UART0 RX interrupt
 */
static inline void Uart0_DisableReceiveIrq()
{
	UART0_C2 &= ~UART0_C2_RIE_MASK;
}

/**
 * @brief Enables the UART0 TX interrupt
 */
static inline void Uart0_EnableTransmitIrq()
{
	UART0_C2 |= UART0_C2_TIE_MASK;
}

/**
 * @brief Disables the UART0 TX interrupt
 */
static inline void Uart0_DisableTransmitIrq()
{
	UART0_C2 &= ~UART0_C2_TIE_MASK;
}

#endif /* UART_H_ */
