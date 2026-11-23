#include "delay.h"
#include "sys.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//If using OS, just include the following header files
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS use
#include "task.h"
#endif


static u8  fac_us=0;							//us delay multiplier
static u16 fac_ms=0;							//ms delay multiplier, which in OS represents the number of ms per beat

static u32 sysTickCnt=0;
extern void xPortSysTickHandler(void);
 
//systick interrupt service function, used when using OS
void SysTick_Handler(void)
{	
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//The system is running
    {
        xPortSysTickHandler();	
    }
}
	/********************************************************
*getSysTickCnt()
*Before scheduling starts, sysTickCnt is returned
*Before scheduling is enabled, return xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*The system is running*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}		   

//Initialize the delay function
//The SYSTICK clock is fixed to AHB clocks, while in the basic example, the SYSTICK clock frequency is AHB/8
//For FreeRTOS compatibility, set the SYSTICK clock frequency to the AHB frequency here.
//SYSCLK: System clock frequency
void delay_init(u8 SYSCLK)
{
	u32 reload;
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 
	fac_us=SYSCLK;							//Whether or not you use the OS, fac_us needs to use it
	reload=SYSCLK;							//Counting times per second The unit is M
	reload*=1000000/configTICK_RATE_HZ;		//Set the overflow time based on the configTICK_RATE_HZ
											//reload is a 24-bit register, maximum value of 16777216, at 168M, approximately 0.0998s
	fac_ms=1000/configTICK_RATE_HZ;			//represents the smallest unit the OS can delay
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//Enable SYSTICK interrupts
	SysTick->LOAD=reload; 					//Break every 1/configTICK_RATE_HZ
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //Enable SYSTICK
}								    

//Delayed NUS
//nus: The number of us to delay.
//nus:0~204522252 (maximum value is 2^32/fac_us@fac_us=168)
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD value
	ticks=nus*fac_us; 						//Required tick count
	told=SysTick->VAL;        				//Counter value at entry
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//Note that SYSTICK is a down-counter.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//Exits if the time exceeds or equals the delayed time.
		}  
	};										    
}  
//Delayed nms
//nms: the number of ms to delay
//nms:0~65535
void delay_ms(u32 nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//The system is running
	{		
		if(nms>=fac_ms)						//The delay time exceeds the minimum OS time period
		{ 
   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS delay
		}
		nms%=fac_ms;						//OS can no longer provide such a small delay, using regular delay instead
	}
	delay_us((u32)(nms*1000));				//Standard delay
}

//Delayed NMS does not cause task scheduling
//nms: the number of ms to delay
void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

			 



































