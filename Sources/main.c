/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "mcg/mcg.h"

void stupid_delay();

void InitClock()
{
	// KUDOS: https://www.youtube.com/watch?v=uiSTB4jkxhw
	
	// NOTE:
	// - core and platform clock are required to have 48 MHz or less
	// - for PBE (PLL engaged external) mode, xtal/divider must be in range 2..4 MHz
	
	const int xtal = 8000000;			// FRDM-KL25Z has an on-board 8 MHz xtal
	const int8_t divider = 4;			// divide by 4 (8 MHz --> 2 MHz)
	const int8_t multiplier = 24;		// scale up by 24 (2 MHz --> 48 MHz)

	pll_init(xtal, LOW_POWER, CRYSTAL, divider, multiplier, MCGOUT);
	// TODO: assert frequency is correct
}

void setup_gpios_for_led()
{
	// Set system clock gating to enable gate to port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	
	// Set Port B, pin 18 and 19 to GPIO mode
	PORTB_PCR18 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTB_PCR19 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	
	// Set Port d, pin 1 GPIO mode
	PORTD_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;;
		
	// Data direction for port B, pin 18 and 19 and port D, pin 1 set to output 
	GPIOB_PDDR |= GPIO_PDDR_PDD(1<<18)
				| GPIO_PDDR_PDD(1<<19);
	GPIOD_PDDR |= GPIO_PDDR_PDD(1<<1);
	
	// LEDs are low active
	GPIOB_PCOR  = 1<<18; // clear output to light red LED
	GPIOB_PCOR  = 1<<19; // clear output to light green LED
	GPIOD_PSOR  = 1<<1;  // set output to clear blue LED
}

int main(void)
{
	InitClock();
	
	setup_gpios_for_led();
	
	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(0b10);	// set UART0 clock to oscillator
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;		// enable clock to UART0 module
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; 		// enable clock to port A (PTA1=rx, PTA2=tx)
	PORTA_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;	// alternative 2: RX
	PORTA_PCR2 = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;	// alternative 2: TX
		
	// configure the UART0
	UART0_BDH = 0b00000000 | UART_BDH_SBR(0); // polling, polling, 1 stop bit, default baud
	UART0_BDL = UART_BDL_SBR(0b00000100) ; // default baud
	
	UART0_C1 = 0b00000000; // all defaults
	UART0_C4 |= UART0_C4_OSR(15); // oversampling ratio of 16
	
	// with SBR (modulo divisor): CLK / ((OSR+1)*SBR) = 125000
	
	UART0_C2 = 0b00001100; // all defaults, but TX and RX enabled
	
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
	GPIOB_PCOR  = 1<<18; // clear output to light red LED
	GPIOB_PSOR  = 1<<19; // set output to clear green LED
	GPIOD_PSOR  = 1<<1; // set output to clear blue LED

	// Toggle the LEDs
	for(;;) 
	{	      	
	   	GPIOB_PSOR = 1<<18;
	   	GPIOB_PCOR = 1<<19;
	   	GPIOD_PSOR = 1<<1;
	   	stupid_delay();
	   	
	   	GPIOB_PSOR = 1<<18;
		GPIOB_PSOR = 1<<19;
		GPIOD_PCOR = 1<<1;
		stupid_delay();
		
		GPIOB_PCOR = 1<<18;
		GPIOB_PSOR = 1<<19;
		GPIOD_PSOR = 1<<1;
		stupid_delay();
	}
	
	return 0;
}


void stupid_delay()
{
	uint16_t i, j;
	for (i=0; i<65534U; ++i) 
	{
		for (j=0; j<50; ++j)
		{
			asm("nop");
		}
	}
}
