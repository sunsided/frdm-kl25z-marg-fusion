/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */

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
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	
	// Set Port B, pin 18 and 19 to GPIO mode
	PORTB->PCR[18] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTB->PCR[19] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	
	// Set Port d, pin 1 GPIO mode
	PORTD->PCR[1] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;;
		
	// Data direction for port B, pin 18 and 19 and port D, pin 1 set to output 
	GPIOB->PDDR |= GPIO_PDDR_PDD(1<<18) | GPIO_PDDR_PDD(1<<19);
	GPIOD->PDDR |= GPIO_PDDR_PDD(1<<1);
	
	// LEDs are low active
	GPIOB->PCOR  = 1<<18; // clear output to light red LED
	GPIOB->PCOR  = 1<<19; // clear output to light green LED
	GPIOD->PSOR  = 1<<1;  // set output to clear blue LED
}

int main(void)
{
	InitClock();
	InitSysTick();
	
	setup_gpios_for_led();
	
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(0b10);	// set UART0 clock to oscillator
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;		// enable clock to UART0 module
	
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; 		// enable clock to port A (PTA1=rx, PTA2=tx)
	PORTA->PCR[1] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;	// alternative 2: RX
	PORTA->PCR[2] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;	// alternative 2: TX
		
	// configure the UART0
	UART0->BDH = 0b00000000 | UART_BDH_SBR(0); // polling, polling, 1 stop bit, default baud
	UART0->BDL = UART_BDL_SBR(0b00000100) ; // default baud
	
	UART0->C1 = 0b00000000; // all defaults
	UART0->C4 |= UART0_C4_OSR(15); // oversampling ratio of 16
	
	// with SBR (modulo divisor): CLK / ((OSR+1)*SBR) = 125000
	UART0->C2 = 0b00001100; // all defaults, but TX and RX enabled
	
	/*
	// wait for TX register to become empty
	for (;;) {
		int tx_register_full = (UART0_S1 & 0b10000000) == 0;
		if (!tx_register_full) break;
	}
	
	// send data
	UART0_D = 0xA;
	
	// wait for TX register to become empty
	for (;;) {
		int tx_register_full = (UART0_S1 & 0b10000000) == 0;
		if (!tx_register_full) break;
	}
	*/
	
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
