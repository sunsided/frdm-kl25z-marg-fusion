/*
 * uart.c
 *
 *  Created on: Oct 30, 2013
 *      Author: Markus
 */

#ifndef UART_C_
#define UART_C_

#define UART_115200 115200 /*! UART in 115.2 kbaud mode */
#define UART_230400 230400 /*! UART in 230.4 kbaud mode */
#define UART_DEV 0x0815 

/**
 * @brief Configures the UART speed
 */
#define UART_SPEED_MODE UART_115200

/**
 * @brief Default UART speed mode selection
 */
#ifndef UART_SPEED_MODE
#define UART_SPEED_MODE UART_115200
#endif

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "bme.h"

#include "comm/buffer.h"
#include "comm/uart.h"

#include "nice_names.h"

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
	
#if UART_SPEED_MODE == UART_115200
#pragma message "Configuring UART0 in 115.200 baud mode."
	
	/* 115200 baud: sbr 13, osr 15 is known to work */
	static const uint16_t sbr = 13U;
	static const uint8_t osr = 15U;
#elif UART_SPEED_MODE == UART_230400
#pragma message "Configuring UART0 in 230.400 baud mode."
	
	/* 230400 baud: sbr 7, osr 15 */
	static const uint16_t sbr = 7U;
	static const uint8_t osr  = 15U;
#elif UART_SPEED_MODE == UART_DEV
	static const uint16_t sbr = 15U;
	static const uint8_t osr  = 3U;
#else
#error No UART speed configured
#endif
	
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
	
	/* set uart clock to PLL/2 clock */
#if UART_SPEED_MODE == UART_115200
	SIM->SOPT2 &= ~(SIM_SOPT2_UART0SRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK); 
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(0b01U) | SIM_SOPT2_PLLFLLSEL_MASK;
#elif UART_SPEED_MODE == UART_230400
	SIM->SOPT2 &= ~(SIM_SOPT2_UART0SRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK); 
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(0b01U) | SIM_SOPT2_PLLFLLSEL_MASK;
#elif UART_SPEED_MODE == UART_DEV
	SIM->SOPT2 &= ~(SIM_SOPT2_UART0SRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK); 
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(0b01U) | SIM_SOPT2_PLLFLLSEL_MASK;
#else
#error No UART speed configured
#endif
	
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
	
	/* set oversampling ratio when oversampling is between 4 and 7 
	 * (it is optional for higher oversampling ratios) */
#if UART_SPEED_MODE == UART_115200
	UART0->C5 = 0;
#elif UART_SPEED_MODE == UART_230400
	UART0->C5 = 0;
#elif UART_SPEED_MODE == UART_DEV
	UART0->C5 = 0;
#else
	if (osr >= 3 && osr <= 7)
	{
		UART0->C5 = UART0_C5_BOTHEDGE_MASK;
	}
	else {
		UART0->C5 = 0;	
	}
#endif
	
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
void Uart0_InitializeIrq(buffer_t *restrict const receiveFifo, buffer_t *restrict const transmitFifo)
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
void UART0_Handler()
{
	const uint8_t config = UART0->C2;
	const uint8_t status = UART0->S1;

    /* handle the receiver full IRQ */
    if ((config & UART0_C2_RIE_MASK) && (status & UART0_S1_RDRF_MASK))
    {
        HandleReceiveInterrupt();

        // clear flags
        // TODO: use BME
        UART0->S1 |= UART0_S1_OR_MASK;  // overrun
    }

	/* handle the transmitter empty IRQ */
	if ((config & UART0_C2_TIE_MASK) && (status & UART0_S1_TDRE_MASK))
	{
		HandleTransmitInterrupt();
	}
}

#endif /* UART_C_ */
