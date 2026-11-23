#include "sys.h"  

//System variables for debugging
DEBUG_t debug;

//Input version parameters; function returns version information
const uint8_t* getSW_Ver(SOFTWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_00: return "V1.00";
		//Pending for subsequent version.
	}
	
	return "unknown";
}

const uint8_t* getHW_Ver(HARDWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_0: return "V1.0";
		case V1_1: return "V1.1";
		//Pending for subsequent version.
	}
	
	return "unknown";
}


//System-level variable initialization
//Set software and hardware version information. Hardware version can only be determined after hardware initialization.
void SYS_VAL_t_Init(SYS_VAL_t* p)
{
	//Current software version
	p->SoftWare_Ver = V1_00;
	
	p->HardWare_Ver = HW_NONE;//To be determined after hardware initialization
	
	p->Time_count = 0;
	p->HardWare_charger = 0;
	p->SecurityLevel = 1; //System safety level, default settings: 1. No command loss protection
	p->LED_delay = 1; //System status indicator light blinking interval, unit: ms
}




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
















