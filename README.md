frdm-kl25z-marg-fusion
======================

Evaluation of a regular Kalman filter based Direction Cosine Matrix estimation through MARG (IMU) sensor fusion, implemented on bare metal Freescale FRDM-KL25Z.

## Implemented so far ##

### Inertial Sensing ###

- (on-board) MMA8451Q accelerometer driver, sensor data dumping in 14-bit non-fifo mode
- MPU6050 accelerometer, gyroscope and temperature sensor driver
- HMC5883L magnetometer sensor driver
- Full configuration fetch and store over I2C
- Automatic endian conversion based on system settings
- Interrupt-based update notification and polling
- MATLAB interfacing at up 800 Hz over serial port
  - serial protocol decoder
  - visual representation of the measured accelerations at 60 FPS with virtual horizon. For an older version, see [https://www.youtube.com/watch?v=RunpaR2PdHQ](https://www.youtube.com/watch?v=RunpaR2PdHQ).

### Sensor Fusion ###

- Standard Kalman filter in 32bit Q16 fixed point using [libfixkalman](https://github.com/sunsided/libfixkalman)
- Direct estimation of DCM axes based on *A DCM Based Orientation Estimation Algorithm with an Inertial Measurement Unit and a Magnetic Compass* (Nguyen Ho Quoc Phuong et al., [J.UCS 15.4](http://www.jucs.org/jucs_15_4/a_dcm_based_orientation)), but using TRIAD approach instead of magnetometer tilt-compensation.
- Quaternion conversion for transfer
- C# demonstration program using OpenTK available in branch [orientation-opengl](https://github.com/sunsided/frdm-kl25z-acc-uart/tree/orientation-opengl) 

### System ###

- Clock in PLL engaged mode (PEE) with 48 MHz core, 24 MHz bus
- SysTick timer running at 0.25ms
- delay_ms() function with low power wait support (WFI)

### Communication ###

- UART0 on pins PTA1/PTA2 with 115.2 kbaud
- interrupt-decoupled UART0 using custom ring buffers
- simple IO layer for data output over UART
  - binary single and bulk transfer
  - integer-to-string, basically itoa() without arrays
  - Q2.12 fixed point to string (nice for the MMA8451Q 14bit mode)
- simple escaping protocol encoder with preamble and length header
- I2C driver for I2C0 (currently hardcoded, but easy to change)
  - arbiter that sets pin configurations on demand per requested device (e.g. when I2C0 is used from different ports)

### LED ###

- RGB LED GPIO access using fast GPIO (Core IOPORT; very trivial API, basically crap)
