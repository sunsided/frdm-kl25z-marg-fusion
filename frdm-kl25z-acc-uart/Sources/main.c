/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

/*!
* \def DATA_FUSE_MODE Set to the <code>1</code> to enable raw sensor data transmission or disable with <code>0</code> to enable data fusion
*/
#define DATA_FETCH_MODE 0

/*!
* \def DATA_FUSE_MODE Set to the opposite of {\ref DATA_FETCH_MODE} and enables sensor fusion
*/
#define DATA_FUSE_MODE (!DATA_FETCH_MODE)

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

#include "fusion/sensor_prepare.h"
#include "fusion/sensor_fusion.h"

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

/************************************************************************/
/* Interrupt handlers                                                   */
/************************************************************************/

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

/************************************************************************/
/* I2C arbiter configuration                                            */
/************************************************************************/

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

/************************************************************************/
/* Signaling of fusion process                                          */
/************************************************************************/

#if DATA_FUSE_MODE

/**
* @brief Sets up the GPIOs for fusion signaling
*/
void FusionSignal_Init()
{
    /* Set system clock gating to enable gate to port B */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    /* Set Port B, pin 8 and 9 to GPIO mode */
    PORTB->PCR[8] = PORT_PCR_MUX(1); /* not using |= assignment here due to some of the flags being undefined at reset */
    PORTB->PCR[9] = PORT_PCR_MUX(1);

    /* Data direction for port B, pin 8 and 9  to output */
    GPIOB->PDDR |= GPIO_PDDR_PDD(1 << 8) | GPIO_PDDR_PDD(1 << 9);
}

/**
* @brief Sets the fusion predict signal
*/
STATIC_INLINE void FusionSignal_Predict()
{
    GPIOB->PSOR = 1 << 8;
    GPIOB->PCOR = 1 << 9;
}

/**
* @brief Clears the fusion update signal
*/
STATIC_INLINE void FusionSignal_Update()
{
    GPIOB->PCOR = 1 << 8;
    GPIOB->PSOR = 1 << 9;
}

/**
* @brief Clears the fusion signal
*/
STATIC_INLINE void FusionSignal_Clear()
{
    GPIOB->PCOR = (1 << 8) | (1 << 9);
}

#endif // #if DATA_FUSE_MODE

/************************************************************************/
/* Main program                                                         */
/************************************************************************/

int main(void)
{
    /* initialize the core clock and the systick timer */
    InitClock();
    InitSysTick();

    /* initialize the RGB led */
    LED_Init();

    /* Initialize UART0 */
    InitUart0();

    /* double rainbow all across the sky */
    DoubleFlash();

    /* initialize the I2C bus */
    I2C_Init();

#if DATA_FUSE_MODE

    /* signaling for fusion */
    FusionSignal_Init();

#endif // DATA_FUSE_MODE

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
    mpu6050_sensor_t accgyrotemp, previous_accgyrotemp;
	MPU6050_InitializeData(&accgyrotemp);
    MPU6050_InitializeData(&previous_accgyrotemp);
	
	/* initialize the HMC5883L data structure */
	hmc5883l_data_t compass, previous_compass;
    HMC5883L_InitializeData(&compass);
    HMC5883L_InitializeData(&previous_compass);

    /* initialize HMC5883L reading */
    uint32_t lastHMCRead = 0;
    const uint32_t readHMCEvery = 1000 / 75; /* at 75Hz, data come every (1000/75Hz) ms. */
    	
    /************************************************************************/
    /* Fetch scaler values                                                  */
    /************************************************************************/

#if DATA_FUSE_MODE

    const fix16_t mpu6050_accelerometer_scaler = mpu6050_accelerometer_get_scaler();
    const fix16_t mpu6050_gyroscope_scaler = mpu6050_gyroscope_get_scaler();
    const fix16_t hmc5883l_magnetometer_scaler = hmc5883l_magnetometer_get_scaler();

#endif // DATA_FUSE_MODE

    /************************************************************************/
    /* Prepare data fusion                                                  */
    /************************************************************************/

#if DATA_FUSE_MODE

    fusion_initialize();

#endif // DATA_FUSE_MODE

    /************************************************************************/
    /* Main loop                                                            */
    /************************************************************************/

	for(;;) 
	{
        /* helper variables to track data freshness */
        uint_fast8_t have_gyro_data = 0;
        uint_fast8_t have_acc_data = 0;
        uint_fast8_t have_mag_data = 0;

        /************************************************************************/
        /* Determine if sensor data fetching is required                        */
        /************************************************************************/

        /* helper variables for event processing */
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

        /************************************************************************/
        /* Fetching MPU6050 sensor data if required                             */
        /************************************************************************/

		/* read accelerometer/gyro */
		if (readMPU)
		{
			LED_BlueOff();
			
			I2CArbiter_Select(MPU6050_I2CADDR);
			MPU6050_ReadData(&accgyrotemp);
			
			/* mark event as detected */
			eventsProcessed = 1;

            /* check for data freshness */
            have_acc_data = (accgyrotemp.accel.x != previous_accgyrotemp.accel.x)
                || (accgyrotemp.accel.y != previous_accgyrotemp.accel.y)
                || (accgyrotemp.accel.z != previous_accgyrotemp.accel.z);

            have_gyro_data = (accgyrotemp.gyro.x != previous_accgyrotemp.gyro.x)
                || (accgyrotemp.gyro.y != previous_accgyrotemp.gyro.y)
                || (accgyrotemp.gyro.z != previous_accgyrotemp.gyro.z);

            /* loop current data --> previous data */
            previous_accgyrotemp = accgyrotemp;
		}
		
        /************************************************************************/
        /* Fetching HMC5883L sensor data if required                            */
        /************************************************************************/

		/* read compass data */
		if (readHMC)
		{
			I2CArbiter_Select(HMC5883L_I2CADDR);
			HMC5883L_ReadData(&compass);
			
			/* mark event as detected */
			eventsProcessed = 1;

            /* check for data freshness */
            have_mag_data = (compass.x != previous_compass.x)
                || (compass.y != previous_compass.y)
                || (compass.z != previous_compass.z);

            /* loop current data --> previous data */
            previous_compass = compass;
		}
		
        /************************************************************************/
        /* Fetching MMA8451Q sensor data if required                            */
        /************************************************************************/

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
		
        /************************************************************************/
        /* Raw sensor data output over serial                                   */
        /************************************************************************/

#if DATA_FETCH_MODE

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
		
#endif // DATA_FETCH_MODE

        /************************************************************************/
        /* Sensor data fusion                                                   */
        /************************************************************************/

#if DATA_FUSE_MODE

        // if there were sensor data ...
        if (eventsProcessed)
        {
            // convert, calibrate and store gyroscope data
            if (have_gyro_data)
            {
                v3d gyro;
                sensor_prepare_mpu6050_gyroscope_data(&gyro, accgyrotemp.gyro.x, accgyrotemp.gyro.y, accgyrotemp.gyro.z, mpu6050_gyroscope_scaler);
                fusion_set_gyroscope_v3d(&gyro);
            }

            // convert, calibrate and store accelerometer data
            if (have_acc_data)
            {
                v3d acc;
                sensor_prepare_mpu6050_accelerometer_data(&acc, accgyrotemp.accel.x, accgyrotemp.accel.y, accgyrotemp.accel.z, mpu6050_accelerometer_scaler);
                fusion_set_accelerometer_v3d(&acc);
            }

            // convert, calibrate and store magnetometer data
            if (have_mag_data)
            {
                v3d mag;
                sensor_prepare_hmc5883l_data(&mag, compass.x, compass.y, compass.z, hmc5883l_magnetometer_scaler);
                fusion_set_magnetometer_v3d(&mag);
            }

            // get the time differential
            const fix16_t deltaT = F16(1); // TODO: get correct value!

            FusionSignal_Predict();

            // predict the current measurements
            fusion_predict(deltaT);

            FusionSignal_Update();

            // correct the measurements
            fusion_update(deltaT);

            // sanitize state data
            fusion_sanitize_state();

            FusionSignal_Clear();
        }

#endif // DATA_FUSE_MODE

        /************************************************************************/
        /* Read user data input                                                 */
        /************************************************************************/

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
		
        /************************************************************************/
        /* Save energy if you like to                                           */
        /************************************************************************/

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