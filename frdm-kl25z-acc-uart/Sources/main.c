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
#include "comm/buffer.h"
#include "comm/io.h"

#include "i2c/i2c.h"
#include "inertial/mma8451q.h"

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

#define UART_RX_BUFFER_SIZE	(32)
#define UART_TX_BUFFER_SIZE	(32)
uint8_t uartInputData[UART_RX_BUFFER_SIZE], 
		uartOutputData[UART_TX_BUFFER_SIZE];
buffer_t uartInputFifo, 
		uartOutputFifo;

int main(void)
{
	InitClock();
	InitSysTick();
	
	setup_gpios_for_led();
	
	InitI2C();
	InitUart0();

	/* initialize UART fifos */
	RingBuffer_Init(&uartInputFifo, &uartInputData, UART_RX_BUFFER_SIZE);
	RingBuffer_Init(&uartOutputFifo, &uartOutputData, UART_TX_BUFFER_SIZE);
	
	/* initialize UART0 interrupts */
	Uart0_InitializeIrq(&uartInputFifo, &uartOutputFifo);
	Uart0_EnableReceiveIrq();
		
	/* turn off LEDs; they are low active, so clearing is enabling */
	GPIOB->PSOR  = 1<<18;
	GPIOB->PSOR  = 1<<19;
	GPIOD->PCOR  = 1<<1;

	IO_SendZString("Accelerometer id:\0");
		
	uint8_t accelerometer = MMA8451Q_WhoAmI();
	IO_SendByte(accelerometer);
	
	IO_SendZString("\r\n\0");
	
	for(;;) 
	{
		/* lights off */
		GPIOB->PSOR = 1<<18;
		GPIOB->PSOR = 1<<19;
		GPIOD->PSOR = 1<<1;
		delay_ms(10); /* TODO: should be __WFI() really, no need to loop */
		
		/* as long as there is data in the buffer */
		while(!RingBuffer_Empty(&uartInputFifo))
		{
			// green
			GPIOB->PSOR = 1<<18;
			GPIOB->PCOR = 1<<19;
			GPIOD->PSOR = 1<<1;
			
			/* fetch byte */
			uint8_t data = IO_ReadByte();
			
			/* echo to output */
			IO_SendByte(data);
		}
	}
	
	return 0;
}
