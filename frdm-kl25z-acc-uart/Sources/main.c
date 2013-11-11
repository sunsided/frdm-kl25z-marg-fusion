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
#include "i2c/i2carbiter.h"
#include "imu/mma8451q.h"
#include "imu/mpu6050.h"
#include "imu/hmc5883l.h"
#include "led/led.h"

#include "nice_names.h"


#define UART_RX_BUFFER_SIZE	(32)				/*< Size of the UART RX buffer in byte*/
#define UART_TX_BUFFER_SIZE	(128)				/*< Size of the UART TX buffer in byte */
uint8_t uartInputData[UART_RX_BUFFER_SIZE], 	/*< The UART RX buffer */
		uartOutputData[UART_TX_BUFFER_SIZE];	/*< The UART TX buffer */
buffer_t uartInputFifo, 						/*< The UART RX buffer driver */
		uartOutputFifo;							/*< The UART TX buffer driver */

#define MMA8451Q_INT_PORT	PORTA				/*< Port at which the MMA8451Q INT1 and INT2 pins are attached */
#define MMA8451Q_INT_GPIO	GPIOA				/*< Port at which the MMA8451Q INT1 and INT2 pins are attached */
#define MMA8451Q_INT1_PIN	14					/*< Pin at which the MMA8451Q INT1 is attached */
#define MMA8451Q_INT2_PIN	15					/*< Pin at which the MMA8451Q INT2 is attached */

#define MPU6050_INT_PORT	PORTA				/*< Port at which the MPU6050 INT pin is attached */
#define MPU6050_INT_GPIO	GPIOA				/*< Port at which the MPU6050 INT pin is attached */
#define MPU6050_INT_PIN		13					/*< Pin at which the MPU6050 INT is attached */

#define I2CARBITER_COUNT 	(3)					/*< Number of I2C devices we're talking to */
i2carbiter_entry_t i2carbiter_entries[I2CARBITER_COUNT]; /*< Structure for the pin enabling/disabling manager */

/**
 * @brief Indicates that polling the MMA8451Q is required
 */
static volatile uint8_t poll_mma8451q = 1;

/**
 * @brief Indicates that polling the MPU6050 is required
 */
static volatile uint8_t poll_mpu6050 = 1;

/**
 * @brief Handler for interrupts on port A
 */
void PORTA_IRQHandler()
{
	register uint32_t isfr = MMA8451Q_INT_PORT->ISFR;
	register uint32_t fromMMA8451Q 	= (isfr & ((1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN)));
	register uint32_t fromMPU6050	= (isfr & (1 << MPU6050_INT_PIN));
	
	/* check MMA8451Q */
	if (fromMMA8451Q || fromMPU6050)
	{
		poll_mma8451q = 1;
		LED_RedOn();
		
		/* clear interrupts using BME decorated logical OR store 
		 * PORTA->ISFR |= (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN); 
		 */
		BME_OR_W(&MMA8451Q_INT_PORT->ISFR, (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN));
	}
	
	/* check MPU6050 */
	if (fromMPU6050)
	{
		poll_mpu6050 = 1;
		LED_BlueOn();
		
		/* clear interrupts using BME decorated logical OR store 
		 * PORTA->ISFR |= (1 << MPU6050_INT_PIN); 
		 */
		BME_OR_W(&MMA8451Q_INT_PORT->ISFR, (1 << MPU6050_INT_PIN));
	}
}

/**
 * @brief Sets up the MMA8451Q communication
 */
void InitMMA8451Q()
{
	mma8451q_confreg_t configuration;

	IO_SendZString("MMA8451Q: initializing ...\r\n");
	
	/* configure interrupts for accelerometer */
	/* INT1_ACCEL is on PTA14, INT2_ACCEL is on PTA15 */
	SIM->SCGC5 |= (1 << SIM_SCGC5_PORTC_SHIFT) & SIM_SCGC5_PORTC_MASK; /* power to the masses */
	MMA8451Q_INT_PORT->PCR[MMA8451Q_INT1_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
	MMA8451Q_INT_PORT->PCR[MMA8451Q_INT2_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
	MMA8451Q_INT_GPIO->PDDR &= ~(GPIO_PDDR_PDD(1<<MMA8451Q_INT1_PIN) | GPIO_PDDR_PDD(1<<MMA8451Q_INT2_PIN));
	
	/* prepare interrupts for pin change / PORTA */
	NVIC_ICPR |= 1 << 30;	/* clear pending flag */
	NVIC_ISER |= 1 << 30;	/* enable interrupt */
	
	/* switch to the correct port */
	I2CArbiter_Select(MMA8451Q_I2CADDR);
	
	/* perform identity check */
	uint8_t id = MMA8451Q_WhoAmI();
	assert(id = 0x1A);
	IO_SendZString("MMA8451Q: device found.\r\n");
	
	/* configure accelerometer */
	MMA8451Q_EnterPassiveMode();
	MMA8451Q_Reset();
	delay_ms(20);
	
	/* TODO: Initiate self-test */
	
	/* configure interrupts for */
	/* INT is on PTA1 */
	SIM->SCGC5 |= (1 << SIM_SCGC5_PORTA_SHIFT) & SIM_SCGC5_PORTA_MASK; /* power to the masses */
	MPU6050_INT_PORT->PCR[MPU6050_INT_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
	MPU6050_INT_GPIO->PDDR &= ~(GPIO_PDDR_PDD(1<<MPU6050_INT_PIN));
	
	/* read configuration and modify */
	MMA8451Q_FetchConfiguration(&configuration);
	
	MMA8451Q_SetSensitivity(&configuration, MMA8451Q_SENSITIVITY_2G, MMA8451Q_HPO_DISABLED);
	MMA8451Q_SetDataRate(&configuration, MMA8451Q_DATARATE_200Hz, MMA8451Q_LOWNOISE_ENABLED);
	MMA8451Q_SetOversampling(&configuration, MMA8451Q_OVERSAMPLING_HIGHRESOLUTION);
	MMA8451Q_ClearInterruptConfiguration(&configuration);
	MMA8451Q_SetInterruptMode(&configuration, MMA8451Q_INTMODE_OPENDRAIN, MMA8451Q_INTPOL_ACTIVELOW);
	MMA8451Q_ConfigureInterrupt(&configuration, MMA8451Q_INT_DRDY, MMA8451Q_INTPIN_INT2);
	
	MMA8451Q_StoreConfiguration(&configuration);
	MMA8451Q_EnterActiveMode();
	
	IO_SendZString("MMA8451Q: configuration done.\r\n");
}

/**
 * @brief Sets up the MPU6050 communication
 */
void InitMPU6050()
{
	mpu6050_confreg_t configuration;
	IO_SendZString("MPU6050: initializing ...\r\n");
	
	/* switch to the correct port */
	I2CArbiter_Select(MPU6050_I2CADDR);
	
	/* perform identity check */
	uint8_t value = MPU6050_WhoAmI();
	assert(value == 0x68);
	IO_SendZString("MPU6050: device found.\r\n");
	
	/* read configuration and modify */
	MPU6050_FetchConfiguration(&configuration);
	MPU6050_SetGyroscopeSampleRateDivider(&configuration, 40); /* the gyro samples at 8kHz, so divide by 40 to get to 200Hz */
	MPU6050_SetGyroscopeFullScale(&configuration, MPU6050_GYRO_FS_250);
	MPU6050_SetAccelerometerFullScale(&configuration, MPU6050_ACC_FS_4);
	MPU6050_ConfigureInterrupts(&configuration, 
			MPU6050_INTLEVEL_ACTIVELOW, 
			MPU6050_INTOPEN_OPENDRAIN, 
			MPU6050_INTLATCH_LATCHED, /* if configured to PULSE the line goes postal */ 
			MPU6050_INTRDCLEAR_READSTATUS);
	MPU6050_EnableInterrupts(&configuration, 
			MPU6050_INT_DISABLED, 
			MPU6050_INT_DISABLED, 
			MPU6050_INT_ENABLED); /* enable data ready interrupt */
	MPU6050_SelectClockSource(&configuration, MPU6050_CLOCK_XGYROPLL);
	MPU6050_SetSleepMode(&configuration, MPU6050_SLEEP_DISABLED);
	MPU6050_StoreConfiguration(&configuration);
	
	IO_SendZString("MPU6050: configuration done.\r\n");
}

/**
 * @brief Sets up the HMC5883L communication
 */
void InitHMC5883L()
{
	IO_SendZString("HMC5883L: initializing ...\r\n");
	
	I2CArbiter_Select(HMC5883L_I2CADDR);
	uint32_t ident = HMC5883L_Identification();
	assert(ident == 0x00483433);
	IO_SendZString("HMC5883L: device found.\r\n");
	
	/* TODO: Further configuration */
	
	IO_SendZString("HMC5883L: configuration done.\r\n");
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
	
	/* prior to configuring the I2C arbiter, enable the clocks required for
	 * the used pins
	 */
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;
	
	/* configure I2C arbiter 
	 * The arbiter takes care of pin selection 
	 */
	I2CArbiter_PrepareEntry(&i2carbiter_entries[0], MMA8451Q_I2CADDR, PORTE, 24, 5, 25, 5);
	I2CArbiter_PrepareEntry(&i2carbiter_entries[1],  MPU6050_I2CADDR, PORTB,  0, 2,  1, 2);
	I2CArbiter_PrepareEntry(&i2carbiter_entries[2], HMC5883L_I2CADDR, PORTB,  0, 2,  1, 2);
	I2CArbiter_Configure(i2carbiter_entries, I2CARBITER_COUNT);
	
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
	
	/* initialize the IMUs */
	InitMMA8451Q();
	InitMPU6050();
	InitHMC5883L();
	
	/* Wait for the config messages to get flushed */
	RingBuffer_BlockWhileNotEmpty(&uartOutputFifo);

	/* initialize the MMA8451Q data structure for accelerometer data fetching */
	mma8451q_acc_t acc;
	MMA8451Q_InitializeData(&acc);

	/* initialize the MPU6050 data structure */
	mpu6050_sensor_t accgyrotemp;
	MPU6050_InitializeData(&accgyrotemp);
	
	/* configure pins for debugging */
	SIM->SCGC5 |= (1 << SIM_SCGC5_PORTC_SHIFT) & SIM_SCGC5_PORTC_MASK; /* power to the masses */
	PORTC->PCR[7] = PORT_PCR_MUX(0x1);
	GPIOC->PDDR |= GPIO_PDDR_PDD(1<<7);
	
	for(;;) 
	{
		GPIOC->PCOR = 1<<7;
		int eventsProcessed = 0;
		int readMMA, readMPU;
		
		/* atomic detection of fresh data */
		__disable_irq();
		readMMA = poll_mma8451q;
		readMPU = poll_mpu6050;
		poll_mma8451q = 0;
		poll_mpu6050 = 0; 
		__enable_irq();
				
		/* read accelerometer */
		if (readMMA)
		{
			LED_RedOff();
			
			I2CArbiter_Select(MMA8451Q_I2CADDR);
			MMA8451Q_ReadAcceleration14bitNoFifo(&acc);
			
			/* mark event as detected */
			eventsProcessed = 1;
		}

		/* read accelerometer/gyro */
		if (readMPU)
		{
			LED_BlueOff();
			
			I2CArbiter_Select(MPU6050_I2CADDR);
			MPU6050_ReadData(&accgyrotemp);
			if (accgyrotemp.accel.z > 16000)
			{
				GPIOC->PSOR = 1<<7;
			}
			
			/* mark event as detected */
			eventsProcessed = 1;
		}
		
		/* data availability + sanity check */
		if (readMMA && acc.status != 0) 
		{
			uint8_t type = 0x01;
			P2PPE_TransmissionPrefixed(&type, 1, (uint8_t*)acc.xyz, sizeof(acc.xyz), IO_SendByte);
		}
		
		/* data availability + sanity check 
		 * This sent me on a long bug hunt: Sometimes the interrupt would be raised
		 * even if not all data registers were written. This always resulted in a
		 * z data register not being fully written which, in turn, resulted in
		 * extremely jumpy measurements. 
		 */
		if (readMPU && accgyrotemp.status != 0)
		{
			/* write data */
			uint8_t type = 0x02;
			P2PPE_TransmissionPrefixed(&type, 1, (uint8_t*)accgyrotemp.data, sizeof(accgyrotemp.data), IO_SendByte);
		}

		GPIOC->PCOR = 1<<7;
		
#if 0
		/* as long as there is data in the buffer */
		while(!RingBuffer_Empty(&uartInputFifo))
		{
			/* light one led */
			LED_Blue();
			
			/* fetch byte */
			uint8_t data = IO_ReadByte();
			
			/* echo to output */
			IO_SendByte(data);
			
			/* mark event as detected */
			eventsProcessed = 1;
		}
#endif
		
		/* in case of no events, allow a sleep */
		if (!eventsProcessed)
		{
			/*
			 * Care must be taken with this instruction here, as it can lead
			 * to a condition where after being woken up (e.g. by the SysTick)
			 * and looping through, immediately before entering WFI again
			 * an interrupt would yield a true condition for the branches below.
			 * In this case this loop would be blocked until the next IRQ,
			 * which, in case of a 1ms SysTick timer, could be too late.
			 * 
			 * To counter this behaviour, SysTick has been speed up by factor
			 * four (0.25ms).
			 */
#if 0
			__WFI();
#endif
		}
	}
	
	return 0;
}
