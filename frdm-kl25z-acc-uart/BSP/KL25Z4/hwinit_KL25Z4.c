#include "mkl25z4.h"

extern void *__vect_table[]; //Defined in vectors_KL25Z4.c

void __attribute__ ((weak)) __init_hardware(void)
{
	//This function provides the MINIMAL hardware initialization required to run your firmware.
	//It DOES NOT setup the clock generator!
	//Please use Freescale Processor Expert to generate a complete __init_hardware() function for
	//your hardware and put it into one of your source files inside your project.
	//The new function will override this 'weak' version.
	
	SCB_VTOR = (uint32_t)(&__vect_table);   //Initialize vector table pointer
	SIM_COPC = SIM_COPC_COPT(0x00);   //Disable watchdog module
}
