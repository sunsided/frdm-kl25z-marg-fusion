frdm-kl25z-acc-uart
===================

Freescale FRDM-KL25Z bare metal foo - an attempt to bring the Kinetis KL25Z Freedom board
to life without using Processor Expert, so just bare metal here. No CMSIS though at the time being.

## Implemented so far ##

### System ###

- Clock in PLL engaged mode (PEE) with 48 MHz core, 24 MHz bus
- SysTick timer running at 1ms
- delay_ms() function with low power wait support (WFI)

### Communication ###

- UART0 on pins PTA1/PTA2 with 115.2 kbaud
- interrupt-decoupled UART0 using custom ring buffers
- simple escaping protocol encoder with preamble and length header
- I2C driver for I2C0 (currently hardcoded, but easy to change)

### LED ###

- RGB LED GPIO access using fast GPIO (Core IOPORT; very trivial API, basically crap)

### Accelerometer ###

- MMA8451Q accelerometer sensor data dumping in 14-bit non-fifo mode
- Full configuration fetch and store over I2C
- Automatic endian conversion based on system settings
- Interrupt-based update notification and polling
- MATLAB interfacing at 800 Hz over serial port
  - serial protocol decoder
  - visual representation of the measured accelerations at 60 FPS