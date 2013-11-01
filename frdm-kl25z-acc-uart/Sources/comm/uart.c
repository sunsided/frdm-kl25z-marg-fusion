/*
 * uart.c
 *
 *  Created on: Oct 30, 2013
 *      Author: Markus
 */

#ifndef UART_C_
#define UART_C_

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "bme.h"
#include "nice_names.h"

#include "buffer.h"
#include "uart.h"

buffer_t* uartReadFifo = 0; /*< the read buffer, initialized by Uart0_InitializeIrq() */
buffer_t* uartWriteFifo = 0; /*< the write buffer, initialized by Uart0_InitializeIrq() */

/*
 * @brief Sets up the UART0 for 115.2 kbaud on PTA1/RX, PTA2/TX using PLL/2 clocking.
 */
void InitUart0()
{
	/*
	static const uint32_t uartclk_hz = CORE_CLOCK/2;
	static const uint32_t baud_rate = 115200U;
	*/
	static const uint32_t osr = 15U;
	static const uint16_t sbr = 13U;
	/*
	uint32_t calculated_baud = uartclk_hz / ((osr+1)*sbr);
	int32_t difference = (calculated_baud - baud_rate);
	if (calculated_baud < baud_rate)
	{
		difference = (baud_rate - calculated_baud);
	}
	const int8_t valid = difference < (baud_rate/100*3);
	*/
	
	/* enable clock gating to uart0 module */
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
	
	/* disable rx and tx */
	UART0->C2 &= ~UART0_C2_TE_MASK & ~UART0_C2_RE_MASK;
	
	/* set uart clock to oscillator clock */
	SIM->SOPT2 &= ~(SIM_SOPT2_UART0SRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK); 
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(0b01U) | SIM_SOPT2_PLLFLLSEL_MASK;
	
	/* enable clock gating to port A */
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; /* enable clock to port A (PTA1=rx, PTA2=tx) */
	
	/* set pins to uart0 rx/tx */
	/* not using |= assignment here due to some of the flags being undefined at reset */
	PORTA->PCR[1] = PORT_PCR_ISF_MASK | PORT_PCR_MUX(2);	/* alternative 2: RX */
	PORTA->PCR[2] = PORT_PCR_ISF_MASK | PORT_PCR_MUX(2);	/* alternative 2: TX */
	
	/* configure the uart */
	UART0->BDH =  (0 << UART_BDH_LBKDIE_SHIFT) /* disable line break detect interrupt */
				| (0 << UART_BDH_RXEDGIE_SHIFT) /* disable RX input active edge interrupt */
				| (0 << UART_BDH_SBNS_SHIFT) /* use one stop bit */
				| UART_BDH_SBR((sbr & 0x1F00) >> 8); /* set high bits of scaler */
	UART0->BDL = UART_BDL_SBR(sbr & 0x00FF) ; /* set low bits of scaler */
	
	/* set oversampling ratio */
	UART0->C4 &= ~UART0_C4_OSR_MASK;
	UART0->C4 |= UART0_C4_OSR(osr);
	
	/* set oversampling ratio since oversampling is between 4 and 7 
	 * and is optional for higher oversampling ratios */
	UART0->C5 = 0; // UART0_C5_BOTHEDGE_MASK;
	
	/* keep default settings for parity and loopback */
	UART0->C1 = 0;
	
	UART0->C3 |= 0;
	
	/* clear flags to avoid unexpected behaviour */
	UART0->MA1 = 0;
	UART0->MA2 = 0;
	UART0->S1 |= UART0_S1_IDLE_MASK
            | UART0_S1_OR_MASK  
            | UART0_S1_NF_MASK
            | UART0_S1_FE_MASK 
            | UART0_S1_PF_MASK;
	UART0->S2 = (UART0_S2_LBKDIF_MASK | UART0_S2_RXEDGIF_MASK);
	
	/*UART0->C1 |= UART0_C1_LOOPS_MASK;*/
	
	/* enable rx and tx */
	UART0->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;
}

/**
 * @brief Initializes the interrupt for UART0
 */
void Uart0_InitializeIrq(buffer_t* receiveFifo, buffer_t* transmitFifo)
{
	/* initialize the structures */
	uartReadFifo = receiveFifo;
	uartWriteFifo = transmitFifo;
	
	/* disable specific interrupts */
	Uart0_DisableReceiveIrq();
	Uart0_DisableTransmitIrq();
	
	/* prepare interrupts for UART0 */
	NVIC_ICPR |= 1 << UART0_IRQ;	/* clear pending flag */
	NVIC_ISER |= 1 << UART0_IRQ;	/* enable interrupt */
}

/**
 * @brief Handles the receiver part of the UART0 IRQ handler
 */
static inline void HandleReceiveInterrupt()
{
	uint8_t data = UART0->D;
	RingBuffer_Write(uartReadFifo, data);
}

/**
 * @brief Handles the receiver part of the UART0 IRQ handler
 */
static inline void HandleTransmitInterrupt()
{
	/* if the buffer is not empty, fetch a byte and send it */
	if (!RingBuffer_Empty(uartWriteFifo))
	{
		const uint8_t data = RingBuffer_Read(uartWriteFifo);
		UART0->D = data;
	}
	else
	{
		/* since the buffer was empty, disable the TDRE IRQ */
		Uart0_DisableTransmitIrq();
	}
}

/**
 * @brief IRQ handler for UART0
 */
void UART0_IRQHandler()
{
	const uint8_t config = UART0->C2;
	const uint8_t status = UART0->S1;
	
	/* handle the receiver full IRQ */
	if ((config & UART0_C2_RIE_MASK) & (status & UART_S1_RDRF_MASK))
	{
		HandleReceiveInterrupt();
	}
	
	/* handle the transmitter empty IRQ */
	if ((config & UART0_C2_TIE_MASK) & (status & UART_S1_TDRE_MASK))
	{
		HandleTransmitInterrupt();
	}
}

#endif /* UART_C_ */
