/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#if 1

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

#include "init_sensors.h"
#include "nice_names.h"

#define UART_RX_BUFFER_SIZE	(32)				/*! Size of the UART RX buffer in byte*/
#define UART_TX_BUFFER_SIZE	(128)				/*! Size of the UART TX buffer in byte */
uint8_t uartInputData[UART_RX_BUFFER_SIZE], 	/*! The UART RX buffer */
		uartOutputData[UART_TX_BUFFER_SIZE];	/*! The UART TX buffer */
buffer_t uartInputFifo, 						/*! The UART RX buffer driver */
		uartOutputFifo;							/*! The UART TX buffer driver */

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
void PORTA_Handler()
{
#if ENABLE_MMA8451Q	
    register uint32_t isfr_mma = MMA8451Q_INT_PORT->ISFR;

	/* check MMA8451Q */
    register uint32_t fromMMA8451Q 	= (isfr_mma & ((1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN)));
	if (fromMMA8451Q || fromMPU6050)
	{
		poll_mma8451q = 1;
		LED_RedOn();
		
		/* clear interrupts using BME decorated logical OR store 
		 * PORTA->ISFR |= (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN); 
		 */
		BME_OR_W(&MMA8451Q_INT_PORT->ISFR, (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN));
	}
#endif
	
	/* check MPU6050 */
    register uint32_t isfr_mpu = MPU6050_INT_PORT->ISFR;
    register uint32_t fromMPU6050 = (isfr_mpu & (1 << MPU6050_INT_PIN));
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

void InitI2CArbiter()
{
    /* prior to configuring the I2C arbiter, enable the clocks required for
    * the used pins
    */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;

    /* configure I2C arbiter
    * The arbiter takes care of pin selection
    */
    I2CArbiter_PrepareEntry(&i2carbiter_entries[0], MMA8451Q_I2CADDR, PORTE, 24, 5, 25, 5);
    I2CArbiter_PrepareEntry(&i2carbiter_entries[1], MPU6050_I2CADDR, PORTB, 0, 2, 1, 2);
    I2CArbiter_PrepareEntry(&i2carbiter_entries[2], HMC5883L_I2CADDR, PORTB, 0, 2, 1, 2);
    I2CArbiter_Configure(i2carbiter_entries, I2CARBITER_COUNT);
}

int main(void)
{
    /* initialize the core clock and the systick timer */
    InitClock();
    InitSysTick();

    /* initialize the RGB led */
    LED_Init();

    /* Initialize UART0 */
    InitUart0();
    for (uint8_t i = 0; i < 2; ++i) {
        TrafficLight();
    }

    /* double rainbow all across the sky */
    DoubleFlash();

    /* initialize the I2C bus */
    I2C_Init();

    /* initialize UART fifos */
    RingBuffer_Init(&uartInputFifo, &uartInputData, UART_RX_BUFFER_SIZE);
    RingBuffer_Init(&uartOutputFifo, &uartOutputData, UART_TX_BUFFER_SIZE);

    /* initialize UART0 interrupts */
    Uart0_InitializeIrq(&uartInputFifo, &uartOutputFifo);
    Uart0_EnableReceiveIrq();

    /* initialize I2C arbiter */
    InitI2CArbiter();
		
	/* initialize the IMUs */
    InitHMC5883L();
	InitMPU6050();
//    InitMPU6050();

#if ENABLE_MMA8451Q
	InitMMA8451Q();
#endif

	/* Wait for the config messages to get flushed */
    //TrafficLight();
    DoubleFlash();
	RingBuffer_BlockWhileNotEmpty(&uartOutputFifo);

#if ENABLE_MMA8451Q
	/* initialize the MMA8451Q data structure for accelerometer data fetching */
	mma8451q_acc_t acc;
	MMA8451Q_InitializeData(&acc);
#endif

	/* initialize the MPU6050 data structure */
	mpu6050_sensor_t accgyrotemp;
	MPU6050_InitializeData(&accgyrotemp);
	
	/* initialize the HMC5883L data structure */
	uint32_t lastHMCRead = 0;
	const uint32_t readHMCEvery = 1000/75; /* at 75Hz, data come every (1000/75Hz) ms. */
	hmc5883l_data_t compass;
	
	/**
	 * BUG:
	 * There seems to be the problem that after power-up the buffers run full immediately
	 * because too much data is fetched from the sensors.
	 * After reset, however, everything works fine.
	 * WORKAROUND:
	 * Power up, wait for some seconds, then reset. 
	 */
	
	for(;;) 
	{
		int eventsProcessed = 0;
        int readMPU, readHMC;
#if ENABLE_MMA8451Q
        int readMMA;
#endif
		
		/* atomic detection of fresh data */
		__disable_irq();
#if ENABLE_MMA8451Q
		readMMA = poll_mma8451q;
#endif
		readMPU = poll_mpu6050;
		poll_mma8451q = 0;
		poll_mpu6050 = 0;
		__enable_irq();
		
		/* detection of HMC read */
		/*
		 * TODO: read synchronized with MPU
		 */
		readHMC = 0;
		uint32_t time = systemTime(); 
		if ((time - lastHMCRead) >= readHMCEvery)
		{
			readHMC = 1;
			lastHMCRead = time;
		}

		/* read accelerometer/gyro */
		if (readMPU)
		{
			LED_BlueOff();
			
			I2CArbiter_Select(MPU6050_I2CADDR);
			MPU6050_ReadData(&accgyrotemp);
			
			/* mark event as detected */
			eventsProcessed = 1;
		}
		
		/* read compass data */
		if (readHMC)
		{
			I2CArbiter_Select(HMC5883L_I2CADDR);
			HMC5883L_ReadData(&compass);
			
			/* mark event as detected */
			eventsProcessed = 1;
		}
		
#if ENABLE_MMA8451Q		
		/* read accelerometer */
		if (readMMA)
		{
			LED_RedOff();
			
			I2CArbiter_Select(MMA8451Q_I2CADDR);
			MMA8451Q_ReadAcceleration14bitNoFifo(&acc);
			
			/* mark event as detected */
			eventsProcessed = 1;
		}
#endif
		
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
		
		/* data availability + sanity check */
		if (readHMC && (compass.status & HMC5883L_SR_RDY_MASK) != 0) /* TODO: check if not in lock state */
		{
			uint8_t type = 0x03;
			P2PPE_TransmissionPrefixed(&type, 1, (uint8_t*)compass.xyz, sizeof(compass.xyz), IO_SendByte);
		}
		
#if ENABLE_MMA8451Q
		/* data availability + sanity check */
		if (readMMA && acc.status != 0) 
		{
			uint8_t type = 0x01;
			P2PPE_TransmissionPrefixed(&type, 1, (uint8_t*)acc.xyz, sizeof(acc.xyz), IO_SendByte);
		}
#endif
		
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

#endif // 0