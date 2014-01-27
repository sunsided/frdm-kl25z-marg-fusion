#include "comm/io.h"
#include "i2c/i2c.h"
#include "i2c/i2carbiter.h"
#include "imu/mma8451q.h"
#include "imu/mpu6050.h"
#include "imu/hmc5883l.h"

#include "init_sensors.h"

/**
* @brief Static buffer to save memory
*
* This buffer is not needed after initialization, so one should probably find a better way to do this.
*/
static union {
#if ENABLE_MMA8451Q
    mma8451q_confreg_t mma8451q_configuration;
#endif
    mpu6050_confreg_t mpu6050_configuration;
    hmc5883l_confreg_t hmc5883l_configuration;
} config_buffer;


/**
* @brief Gets the scaling value for the MPU6050 accelerometer
*/
static fix16_t mpu6050_accelerometer_scaler = 0;

/**
* @brief Gets the scaling value for the MPU6050 gyroscope
*/
static fix16_t mpu6050_gyroscope_scaler = 0;

/**
* @brief Gets the scaling value for the HMC5883L magnetometer
*/
static fix16_t hmc5883l_magnetometer_scaler = 0;

/**
* @brief Sets up the MMA8451Q communication
*/
void InitMMA8451Q()
{
#if ENABLE_MMA8451Q
    mpu6050_confreg_t *configuration = &config_buffer.mma8451q_configuration;

    IO_SendZString("MMA8451Q: initializing ...\r\n");

    /* configure interrupts for accelerometer */
    /* INT1_ACCEL is on PTA14, INT2_ACCEL is on PTA15 */
    SIM->SCGC5 |= (1 << SIM_SCGC5_PORTC_SHIFT) & SIM_SCGC5_PORTC_MASK; /* power to the masses */
    MMA8451Q_INT_PORT->PCR[MMA8451Q_INT1_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
    MMA8451Q_INT_PORT->PCR[MMA8451Q_INT2_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
    MMA8451Q_INT_GPIO->PDDR &= ~(GPIO_PDDR_PDD(1 << MMA8451Q_INT1_PIN) | GPIO_PDDR_PDD(1 << MMA8451Q_INT2_PIN));

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

    /* read configuration and modify */
    MMA8451Q_FetchConfiguration(configuration);

    MMA8451Q_SetSensitivity(configuration, MMA8451Q_SENSITIVITY_2G, MMA8451Q_HPO_DISABLED);
    MMA8451Q_SetDataRate(configuration, MMA8451Q_DATARATE_100Hz, MMA8451Q_LOWNOISE_ENABLED);
    MMA8451Q_SetOversampling(configuration, MMA8451Q_OVERSAMPLING_HIGHRESOLUTION);
    MMA8451Q_ClearInterruptConfiguration(configuration);
    MMA8451Q_SetInterruptMode(configuration, MMA8451Q_INTMODE_OPENDRAIN, MMA8451Q_INTPOL_ACTIVELOW);
    MMA8451Q_ConfigureInterrupt(configuration, MMA8451Q_INT_DRDY, MMA8451Q_INTPIN_INT2);

    MMA8451Q_StoreConfiguration(configuration);
    MMA8451Q_EnterActiveMode();

    IO_SendZString("MMA8451Q: configuration done.\r\n");
#endif
}

/**
* @brief Sets up the MPU6050 communication
*/
void InitMPU6050()
{
    mpu6050_confreg_t *configuration = &config_buffer.mpu6050_configuration;

    IO_SendZString("MPU6050: initializing ...\r\n");

    /**
    * BUG: see also note in main()
    * After power-up the interrupt line toggles
    * WORKAROUND:
    * Power up, wait for some seconds, then reset.
    */

    /* switch to the correct port */
    I2CArbiter_Select(MPU6050_I2CADDR);

    /* perform identity check */
    uint8_t value = MPU6050_WhoAmI();
    assert(value == 0x68);
    IO_SendZString("MPU6050: device found.\r\n");

    /* disable interrupts */
    MPU6050_SelectClockSource(MPU6050_CONFIGURE_DIRECT, MPU6050_CLOCK_8MHZOSC);
    MPU6050_EnableInterrupts(MPU6050_CONFIGURE_DIRECT,
        MPU6050_INT_DISABLED,
        MPU6050_INT_DISABLED,
        MPU6050_INT_DISABLED); /* disable data ready interrupt */

    /* read configuration and modify */
    MPU6050_FetchConfiguration(configuration);
#if DEBUG
    MPU6050_SetGyroscopeSampleRateDivider(configuration, 80); /* the gyro samples at 8kHz, so division by 40 --> 200Hz */
#else
    MPU6050_SetGyroscopeSampleRateDivider(configuration, 40); /* the gyro samples at 8kHz, so division by 40 --> 200Hz */
#endif
    
    MPU6050_SetGyroscopeFullScale(configuration, MPU6050_GYRO_FS_1000);
    mpu6050_gyroscope_scaler = fix16_from_float(MPU6050_GYRO_LSB(MPU6050_GYRO_FS_1000));

    MPU6050_SetAccelerometerFullScale(configuration, MPU6050_ACC_FS_4);
    mpu6050_accelerometer_scaler = fix16_from_int(8192);    /* scaler value for ACC_FS_4*/

    MPU6050_ConfigureInterrupts(configuration,
        MPU6050_INTLEVEL_ACTIVELOW,
        MPU6050_INTOPEN_OPENDRAIN,
        MPU6050_INTLATCH_LATCHED, /* if configured to PULSE the line goes postal */
        MPU6050_INTRDCLEAR_READSTATUS);
    MPU6050_EnableInterrupts(configuration,
        MPU6050_INT_DISABLED,
        MPU6050_INT_DISABLED,
        MPU6050_INT_ENABLED); /* enable data ready interrupt */
    MPU6050_SelectClockSource(configuration, MPU6050_CLOCK_XGYROPLL);
    MPU6050_SetSleepMode(configuration, MPU6050_SLEEP_DISABLED);
    MPU6050_StoreConfiguration(configuration);

    /* configure interrupts for MPU6050 */
    /* INT is on PTA13 */
    SIM->SCGC5 |= (1 << SIM_SCGC5_PORTA_SHIFT) & SIM_SCGC5_PORTA_MASK; /* power to the masses */
    MPU6050_INT_PORT->PCR[MPU6050_INT_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
    MPU6050_INT_GPIO->PDDR &= ~(GPIO_PDDR_PDD(1 << MPU6050_INT_PIN));

    /* prepare interrupts for pin change / PORTA */
    NVIC_ICPR |= 1 << 30;	/* clear pending flag */
    NVIC_ISER |= 1 << 30;	/* enable interrupt */

    IO_SendZString("MPU6050: configuration done.\r\n");
}

/**
* @brief Sets up the HMC5883L communication
*/
void InitHMC5883L()
{
    hmc5883l_confreg_t *configuration = &config_buffer.hmc5883l_configuration;
    IO_SendZString("HMC5883L: initializing ...\r\n");

    I2CArbiter_Select(HMC5883L_I2CADDR);
    uint32_t ident = HMC5883L_Identification();
    assert(ident == 0x00483433);
    IO_SendZString("HMC5883L: device found.\r\n");

    /* read configuration and modify */
    HMC5883L_FetchConfiguration(configuration);
    HMC5883L_SetAveraging(configuration, HMC5883L_MA_1);
    HMC5883L_SetOutputRate(configuration, HMC5883L_DO_75Hz);
    HMC5883L_SetMeasurementMode(configuration, HMC5883L_MS_NORMAL);
    
    HMC5883L_SetGain(configuration, HMC5883L_GN_1090_1p3Ga);
    hmc5883l_magnetometer_scaler = fix16_from_int(1090);         /* scaler value for GN_1090_1p3Ga*/

    HMC5883L_SetOperatingMode(configuration, HMC5883L_MD_CONT);
    HMC5883L_StoreConfiguration(configuration);

    IO_SendZString("HMC5883L: configuration done.\r\n");
}

/**
* @brief Gets the scaling value for the MPU6050 accelerometer
*/
fix16_t mpu6050_accelerometer_get_scaler()
{
    return mpu6050_accelerometer_scaler;
}

/**
* @brief Gets the scaling value for the MPU6050 gyroscope
*/
fix16_t mpu6050_gyroscope_get_scaler()
{
    return mpu6050_gyroscope_scaler;
}

/**
* @brief Gets the scaling value for the HMC5883L magnetometer
*/
fix16_t hmc5883l_magnetometer_get_scaler()
{
    return hmc5883l_magnetometer_scaler;
}