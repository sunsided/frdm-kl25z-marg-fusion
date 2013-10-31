/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */

#include "cpu/clock.h"
#include "cpu/systick.h"
#include "cpu/delay.h"
#include "comm/uart.h"
#include "nice_names.h"

void setup_gpios_for_led()
{
	// Set system clock gating to enable gate to port B
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	
	// Set Port B, pin 18 and 19 to GPIO mode
	PORTB->PCR[18] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; /* not using |= assignment here due to some of the flags being undefined at reset */
	PORTB->PCR[19] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	
	// Set Port d, pin 1 GPIO mode
	PORTD->PCR[1] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
		
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
	
	InitUart0();

	// blue on
	GPIOB->PSOR = 1<<18;
	GPIOB->PSOR = 1<<19;
	GPIOD->PCOR = 1<<1;
	
	/*
	uint8_t c = 'a';
	while (1)
	{	
		while(!(UART0_S1&UART_S1_TDRE_MASK) && !(UART0_S1&UART_S1_TC_MASK));
		UART0->D = c;
		
		while(!(UART0_S1&UART_S1_RDRF_MASK));
		c = UART0->D;
	}
	*/

	// blue off
	GPIOB->PSOR = 1<<18;
	GPIOB->PSOR = 1<<19;
	GPIOD->PSOR = 1<<1;
	
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
