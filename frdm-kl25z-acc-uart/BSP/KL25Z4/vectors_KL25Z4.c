/*
	This file contains the definitions of the interrupt handlers for KL25Z4 MCU family.
	The file is provided by Sysprogs under the BSD license.
*/


extern void *_estack;
#define NULL ((void *)0)

void Reset_Handler();
void Default_Handler();

void NMI_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void Hard_Fault_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void SVCall_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void PendableSrvReq_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void SysTick_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA3_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void FTFA_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void LVD_LVW_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void LLW_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void I2C0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void SPI0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void SPI1_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void UART0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void UART1_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void UART2_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void ADC0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void CMP0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void TPM0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void TPM1_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void TPM2_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void RTC_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void RTC_Seconds_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void PIT_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void USB0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void DAC0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void TSI0_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void MCG_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void LPTimer_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void PORTA_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void PORTD_Handler() __attribute__ ((weak, alias ("Default_Handler")));

void * __vect_table[0x30] __attribute__ ((section (".vectortable"))) = 
{
	&_estack,
	&Reset_Handler,
	&NMI_Handler,
	&Hard_Fault_Handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	&SVCall_Handler,
	NULL,
	NULL,
	&PendableSrvReq_Handler,
	&SysTick_Handler,
	&DMA0_Handler,
	&DMA1_Handler,
	&DMA2_Handler,
	&DMA3_Handler,
	NULL,
	&FTFA_Handler,
	&LVD_LVW_Handler,
	&LLW_Handler,
	&I2C0_Handler,
	&I2C1_Handler,
	&SPI0_Handler,
	&SPI1_Handler,
	&UART0_Handler,
	&UART1_Handler,
	&UART2_Handler,
	&ADC0_Handler,
	&CMP0_Handler,
	&TPM0_Handler,
	&TPM1_Handler,
	&TPM2_Handler,
	&RTC_Handler,
	&RTC_Seconds_Handler,
	&PIT_Handler,
	NULL,
	&USB0_Handler,
	&DAC0_Handler,
	&TSI0_Handler,
	&MCG_Handler,
	&LPTimer_Handler,
	NULL,
	&PORTA_Handler,
	&PORTD_Handler
};

void Default_Handler()
{
	__asm("BKPT 255");
}
