/*
	This file contains the entry point (Reset_Handler) of your firmware project.
	The reset handled initializes the RAM and calls system library initializers as well as
	the platform-specific initializer and the main() function.
*/

extern void *_sidata, *_sdata, *_edata;
extern void *_sbss, *_ebss;

void __init_hardware(void);
void __libc_init_array();
int main();

void __attribute__((naked, noreturn)) Reset_Handler()
{
	__init_hardware();

	//Normally the CPU should will setup the based on the value from the first entry in the vector table.
	//If you encounter problems with accessing stack variables during initialization, ensure 
	//asm ("ldr sp, =_estack");

	void **pSource, **pDest;
	for (pSource = &_sidata, pDest = &_sdata; pDest != &_edata; pSource++, pDest++)
		*pDest = *pSource;

	for (pDest = &_sbss; pDest != &_ebss; pDest++)
		*pDest = 0;

	__libc_init_array();
	main();
	for (;;) ;
}

/*
	The structure below defines the FLASH security configuration values placed at offset 0x400 in FLASH memory.
	Note that specifying wrong values can make your board unusable (all debugger access including mass erase will be disabled).
	If you want to change those values, define KINETIS_CUSTOM_FLASH_SECURITY macro in your Project Settings and provide your own
	definition of the FLASH security configuration values.
*/
#ifndef KINETIS_CUSTOM_FLASH_SECURITY
  __attribute__ ((section (".cfmconfig"))) const unsigned char _cfm[0x10] = {
   /* NV_BACKKEY3: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY2: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY1: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY0: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY7: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY6: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY5: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY4: KEY=0xFF */
    0xFFU,
   /* NV_FPROT3: PROT=0xFF */
    0xFFU,
   /* NV_FPROT2: PROT=0xFF */
    0xFFU,
   /* NV_FPROT1: PROT=0xFF */
    0xFFU,
   /* NV_FPROT0: PROT=0xFF */
    0xFFU,
   /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */
    0x7EU,
   /* NV_FOPT: ??=1,??=1,FAST_INIT=1,LPBOOT1=1,RESET_PIN_CFG=1,NMI_DIS=1,??=1,LPBOOT0=1 */
    0xFFU,
    0xFFU,
    0xFFU
  };
#endif