frdm-kl25z-acc-uart
===================

Freescale FRDM-KL25Z bare metal foo - an attempt to bring the Kinetis KL25Z Freedom board
to life without using Processor Expert, so just bare metal here. No CMSIS though at the time being.

Implemented so far:
  - Clock in PLL engaged mode (PEE) with 48 MHz core, 24 MHz bus
  - UART0 on pins PTA1/PTA2 with 115.2 kbaud
  - interrupt-decoupled UART0 using custom ring buffers
  - RGB LED GPIO access using fast GPIO (Core IOPORT; No API though, just crap)
