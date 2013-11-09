/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "bme.h"

#include "cpu/clock.h"
#include "cpu/systick.h"
#include "cpu/delay.h"
#include "comm/uart.h"
#include "comm/buffer.h"
#include "comm/io.h"
#include "comm/p2pprotocol.h"

#include "i2c/i2c.h"
#include "inertial/mma8451q.h"
#include "led/led.h"

#include "nice_names.h"


#define UART_RX_BUFFER_SIZE	(32)				/*< Size of the UART RX buffer in byte*/
#define UART_TX_BUFFER_SIZE	(128)				/*< Size of the UART TX buffer in byte */
uint8_t uartInputData[UART_RX_BUFFER_SIZE], 	/*< The UART RX buffer */
		uartOutputData[UART_TX_BUFFER_SIZE];	/*< The UART TX buffer */
buffer_t uartInputFifo, 						/*< The UART RX buffer driver */
		uartOutputFifo;							/*< The UART TX buffer driver */

#define MMA8451Q_INT_PORT	PORTA				/*< Port at which the MMA8451Q INT1 and INT2 pins are attached */
#define MMA8451Q_INT1_PIN	14					/*< Pin at which the MMA8451Q INT1 is attached */
#define MMA8451Q_INT2_PIN	15					/*< Pin at which the MMA8451Q INT1 is attached */

/**
 * @brief Indicates that polling the MMA8451Q is required
 */
static volatile uint8_t poll_mma8451q = 1;

/**
 * @brief Handler for interrupts on port A
 */
void PORTA_IRQHandler()
{
	register uint32_t fromMMA8451Q = (MMA8451Q_INT_PORT->ISFR & ((1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN)));
	if (fromMMA8451Q)
	{
		poll_mma8451q = 1;
		LED_RedOn();
		GPIOC->PSOR  = 1<<1;
		
		/* clear interrupts using BME decorated logical OR store 
		 * PORTA->ISFR |= (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN); 
		 */
		BME_OR_W(&MMA8451Q_INT_PORT->ISFR, (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN));
	}
}

/**
 * @brief Sets up the MMA8451Q communication
 */
void InitMMA8451Q()
{
	mma8451q_confreg_t configuration;
	
	/* configure interrupts for accelerometer */
	/* INT1_ACCEL is on PTA14, INT2_ACCEL is on PTA15 */
	SIM->SCGC5 |= (1 << SIM_SCGC5_PORTA_SHIFT) & SIM_SCGC5_PORTA_SHIFT; /* power to the masses */
	MMA8451Q_INT_PORT->PCR[MMA8451Q_INT1_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
	MMA8451Q_INT_PORT->PCR[MMA8451Q_INT2_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
	GPIOA->PDDR &= ~(GPIO_PDDR_PDD(1<<14) | GPIO_PDDR_PDD(1<<15));
	
	/* prepare interrupts for pin change / PORTA */
	NVIC_ICPR |= 1 << 30;	/* clear pending flag */
	NVIC_ISER |= 1 << 30;	/* enable interrupt */
	
	/* configure accelerometer */
	MMA8451Q_EnterPassiveMode();
	MMA8451Q_Reset();
	delay_ms(20);
	MMA8451Q_FetchConfiguration(&configuration);
	
	MMA8451Q_SetSensitivity(&configuration, MMA8451Q_SENSITIVITY_2G, MMA8451Q_HPO_DISABLED);
	MMA8451Q_SetDataRate(&configuration, MMA8451Q_DATARATE_800Hz, MMA8451Q_LOWNOISE_ENABLED);
	MMA8451Q_SetOversampling(&configuration, MMA8451Q_OVERSAMPLING_HIGHRESOLUTION);
	MMA8451Q_ClearInterruptConfiguration(&configuration);
	MMA8451Q_SetInterruptMode(&configuration, MMA8451Q_INTMODE_OPENDRAIN, MMA8451Q_INTPOL_ACTIVELOW);
	MMA8451Q_ConfigureInterrupt(&configuration, MMA8451Q_INT_DRDY, MMA8451Q_INTPIN_INT2);
	
	MMA8451Q_StoreConfiguration(&configuration);
	MMA8451Q_EnterActiveMode();
}

/**
 * @brief LED Traffic Light!
 */
void TrafficLight()
{
	LED_Red();
	delay_ms(1000);
	LED_Yellow();
	delay_ms(1000);
	LED_Green();
	delay_ms(1000);
}

/**
 * @brief LED Double Flash!
 */
void DoubleFlash()
{
	LED_White();
	delay_ms(50);
	LED_Off();
	delay_ms(50);
	LED_White();
	delay_ms(50);
	LED_Off();	
}

int main(void)
{
	/* initialize the core clock and the systick timer */
	InitClock();
	InitSysTick();
	
	/* initialize the RGB led */
	LED_Init();
	/* fun fun fun */
	TrafficLight();
	DoubleFlash();
	
	/* initlialize the I2C bus */
	I2C_Init();
	I2C_ResetBus();
	
	/* Initialize UART0 */
	InitUart0();
	
	/* initialize UART fifos */
	RingBuffer_Init(&uartInputFifo, &uartInputData, UART_RX_BUFFER_SIZE);
	RingBuffer_Init(&uartOutputFifo, &uartOutputData, UART_TX_BUFFER_SIZE);
	
	/* initialize UART0 interrupts */
	Uart0_InitializeIrq(&uartInputFifo, &uartOutputFifo);
	Uart0_EnableReceiveIrq();

	/* setting PTC8/9 to I2C0 for wire sniffing */
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; /* clock to gate C */
	PORTC->PCR[8] = PORT_PCR_MUX(2) | ((1 << PORT_PCR_PE_SHIFT) | PORT_PCR_PE_MASK); /* SCL: alternative 2 with pull-up enabled */
	PORTC->PCR[9] = PORT_PCR_MUX(2) | ((1 << PORT_PCR_PE_SHIFT) | PORT_PCR_PE_MASK); /* SDA_ alternative 2 with pull-up enabled */
	
	/* setting PTC1 to debug MMA8451Q irq */
	PORTC->PCR[1] = PORT_PCR_MUX(1);
	PORTC->PCR[2] = PORT_PCR_MUX(1);
	PORTB->PCR[3] = PORT_PCR_MUX(1);
	GPIOC->PDDR |= GPIO_PDDR_PDD(1<<1) | GPIO_PDDR_PDD(1<<2);
	GPIOB->PDDR |= GPIO_PDDR_PDD(1<<3);
	
	/* initialize the MMA8451Q accelerometer */
	IO_SendZString("MMA8451Q init ...\r\n");
	InitMMA8451Q();
	IO_SendZString("done\r\n");
	
	/* initialize the MMA8451Q data structure for accelerometer data fetching */
	mma8451q_acc_t acc;
	MMA8451Q_InitializeData(&acc);

	for(;;) 
	{
		GPIOB->PSOR  = 1<<3;
		__WFI();
		
		/* as long as there is data in the buffer */
		while(!RingBuffer_Empty(&uartInputFifo))
		{
			/* light one led */
			LED_Blue();
			
			/* fetch byte */
			uint8_t data = IO_ReadByte();
			
			/* echo to output */
			IO_SendByte(data);
		}
		GPIOB->PCOR  = 1<<3;
		
		/* read accelerometer */
		if (poll_mma8451q)
		{
			GPIOC->PCOR  = 1<<1;
			GPIOC->PSOR  = 1<<2;
			
			/*
			 * An interesting observation about this approach is that as soon
			 * as the accelerometer is a position where Z points straight up or
			 * down, the UART TX buffer will run into an "full" block. When this
			 * happens, the system ends up in a state where the poll_mma8451q 
			 * flag is not set although the IRQ handler should have set it.
			 * Since the flag is not set, the MMA8451Q is not getting polled.
			 * Because it is not polled, it's interrupt bit is not getting
			 * cleared and as a result the poll flag will never be set again.
			 * 
			 * Even with 800 Hz the buffer should not run full:
			 * 800 Hz * (6 Data Byte + 5 Byte protocol) = 70.400 bit,
			 * 
			 * In the end, the problem was caused by the _WFI instruction in this
			 * loop when it went to sleep, shortly after the IRQ handler signaled
			 * poll_mma8451q. Speeding up the system tick timer by 4 (0.25ms)
			 * helped reduce the problem.
			 */
			
			poll_mma8451q = 0;
			LED_RedOff();
			MMA8451Q_ReadAcceleration14bitNoFifo(&acc);
			if (acc.status != 0)
			{
				P2PPE_Transmission((uint8_t*)acc.xyz, 3*sizeof(acc.xyz[0]), IO_SendByte);
			}
			GPIOC->PCOR  = 1<<2;
		}
	}
	
	return 0;
}
