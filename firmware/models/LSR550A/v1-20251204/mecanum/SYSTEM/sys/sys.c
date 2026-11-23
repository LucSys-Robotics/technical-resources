#include "sys.h"  


//THUMB directive does not support assembly inline
//Use the following method to execute the WFI assembly instruction
__asm void WFI_SET(void)
{
	WFI;		  
}
//Disable all interrupts (excluding fault and NMI interrupts)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//Enable all interrupts
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//Set the top of the stack address
//addr: Stack top address
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
















