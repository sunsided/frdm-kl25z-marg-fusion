/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "bme.h"

//#include "uart/uart.h"

#include "cpu/clock.h"
#include "cpu/systick.h"
#include "cpu/delay.h"

#define SIM 	SIM_BASE_PTR
#define PORTA	PORTA_BASE_PTR
#define PORTB	PORTB_BASE_PTR
#define PORTD	PORTD_BASE_PTR
#define GPIOB	PTB_BASE_PTR
#define GPIOD	PTD_BASE_PTR

#define UART0	UART0_BASE_PTR

void setup_gpios_for_led()
{
	// Set system clock gating to enable gate to port B
	BME_OR_W(&SIM->SCGC5, SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK);
	
	// Set Port B, pin 18 and 19 to GPIO mode
	BME_OR_W(&PORTB->PCR[18], PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);
	BME_OR_W(&PORTB->PCR[19], PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);
	
	// Set Port d, pin 1 GPIO mode
	BME_OR_W(&PORTD->PCR[1], PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);
		
	// Data direction for port B, pin 18 and 19 and port D, pin 1 set to output 
	BME_OR_W(&GPIOB->PDDR, GPIO_PDDR_PDD(1<<18) | GPIO_PDDR_PDD(1<<19));
	BME_OR_W(&GPIOD->PDDR, GPIO_PDDR_PDD(1<<1));
	
	// LEDs are low active
	GPIOB->PCOR  = 1<<18; // clear output to light red LED
	GPIOB->PCOR  = 1<<19; // clear output to light green LED
	GPIOD->PSOR  = 1<<1;  // set output to clear blue LED
}

#define UART_USE_XTAL_CLOCK	(0b10u)

void setup_uart0()
{
	/* enable clock gating to uart0 module */
	BME_OR_W(&SIM->SCGC4, SIM_SCGC4_UART0_MASK);
	
	/* set uart clock to oscillator clock */
	BME_OR_W(&SIM->SOPT2, SIM_SOPT2_UART0SRC(UART_USE_XTAL_CLOCK));
	
	/* enable clock gating to port A */
	BME_OR_W(&SIM->SCGC5, SIM_SCGC5_PORTA_MASK); 		// enable clock to port A (PTA1=rx, PTA2=tx)
	
	/* set pins to uart0 rx/tx */
	BME_OR_W(&PORTA->PCR[1], PORT_PCR_MUX(2));	// alternative 2: RX
	BME_OR_W(&PORTA->PCR[2], PORT_PCR_MUX(2));	// alternative 2: TX

//	uart0_init(UART0_BASE_PTR, 8000, 19200);

	/* target baud rate: 19200 @ 8 MHz clock
	 * baud rate B will be 
	 * B = clock / ((OSR+1)*SBR)
	 * 
	 * OSR: oversampling ratio, UART0_C4[3:0]
	 * SBR: baud date modulo divisor, UART0_BDH[4:0], UART0_BDL[7:0]
	 */
	
	static const uint32_t uartclk_hz = XTAL_FREQ;
	static const uint32_t baud_rate = 19200;
	static const uint32_t osr = 3;
	uint16_t sbr = uartclk_hz / (baud_rate * (osr+1));
	uint16_t calculated_baud = uartclk_hz / ((osr+1)*sbr);
	
	int32_t difference = (calculated_baud - baud_rate);
	if (calculated_baud < baud_rate)
	{
		difference = (baud_rate - calculated_baud);
	}
	const int8_t valid = difference < (baud_rate/100*3);
	
	// with SBR (modulo divisor): CLK / ((OSR+1)*SBR) = 125000
	
	/* configure the uart */
	UART0->BDH &= ~(UART0_BDH_SBR_MASK); 
	UART0->BDL &= ~(UART0_BDL_SBR_MASK);
	
	UART0->BDH = 0 << UART_BDH_LBKDIE_SHIFT /* disable line break detect interrupt */
				| 0 << UART_BDH_RXEDGIE_SHIFT /* disable RX input active edge interrupt */
				| 0 << UART_BDH_SBNS_SHIFT /* use one stop bit */
				| UART_BDH_SBR((sbr & 0x1F00) >> 8); /* set high bits of scaler */
	UART0->BDL = UART_BDL_SBR(sbr & 0x00FF) ; /* set low bits of scaler */
	
	/* set oversampling ratio */
	UART0->C4 |= UART0_C4_OSR(osr);
	
	/* set oversampling ratio since oversampling is between 4 and 7 
	 * and is optional for higher oversampling ratios */
	if (osr >= 4)
	{
		UART0->C5 |= UART0_C5_BOTHEDGE_MASK;
	}
	
	/* keep default settings for parity and loopback */
	UART0->C1 = 0;
			
	/* enable rx and tx */
	UART0->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;
	
	// blue on
	GPIOB->PSOR = 1<<18;
	GPIOB->PSOR = 1<<19;
	GPIOD->PCOR = 1<<1;
	
	// wait for TX register to become empty
	for (;;) {
		int tx_register_full = (UART0_S1 & UART0_S1_TDRE_MASK) == 0;
		if (!tx_register_full) break;
	}
	
	// send data
	UART0->D = 0xA;
	__NOP();
	UART0->D = 0xA;
	
	// wait for TX register to become empty
	for (;;) {
		int tx_register_full = (UART0_S1 & UART0_S1_TDRE_MASK) == 0;
		if (!tx_register_full) break;
	}
	
	// blue off
	GPIOB->PSOR = 1<<18;
	GPIOB->PSOR = 1<<19;
	GPIOD->PSOR = 1<<1;
}

int main(void)
{
	InitClock();
	InitSysTick();
	
	setup_gpios_for_led();
	
	setup_uart0();
	
	// LEDs are low active
	GPIOB->PCOR  = 1<<18; // clear output to light red LED
	GPIOB->PSOR  = 1<<19; // set output to clear green LED
	GPIOD->PSOR  = 1<<1; // set output to clear blue LED

	// Toggle the LEDs
	for(;;) 
	{
		// red
		GPIOB->PCOR = 1<<18;
		GPIOB->PSOR = 1<<19;
		GPIOD->PSOR = 1<<1;
		delay_ms(1500);
		
		// yellow
		GPIOB->PCOR = 1<<18;
		GPIOB->PCOR = 1<<19;
		GPIOD->PSOR = 1<<1;
		delay_ms(1500);
		
		// green
		GPIOB->PSOR = 1<<18;
		GPIOB->PCOR = 1<<19;
		GPIOD->PSOR = 1<<1;
		delay_ms(1500);
		
		// all
		GPIOB->PSOR = 1<<18;
		GPIOB->PSOR = 1<<19;
		GPIOD->PSOR = 1<<1;
		delay_ms(10000);
		
		// blue
		GPIOB->PSOR = 1<<18;
		GPIOB->PSOR = 1<<19;
		GPIOD->PCOR = 1<<1;
		delay_ms(2000);
	}
	
	return 0;
}
