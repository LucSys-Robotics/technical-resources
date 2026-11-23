#include "bsp_dwt.h"

//Under FreeRTOS, SysTick will be taken over and configured by RTOS, where a DWT timer is used to satisfy the delay function
//The DWT counter frequency is the system clock frequency, with a total of 32 bits, sequential counting.
void DWT_Init(void)
{
	//1. DGB system tracing must be enabled before using DWT (Cortex-M3 Authoritative Guide).
	DEM_CR |= 1<<24;

	//2. Counter clears to 0
	DWT_CYCCNT = (uint32_t)0u;

	//3. Enable timing
	DWT_CTRL |= 1<<0;
}

//Return the 32-bit CYCCNT counter value; counting frequency equals the system clock frequency
static uint32_t DWT_GetTick(void)
{
	return (uint32_t)DWT_CYCCNT;
}

void delay_us(uint32_t us)
{
	//Exceeding the maximum delay 25565281 that can be satisfied
	if( us > DWT_CNT_MAX/(SystemCoreClock/1000000) ) us = DWT_CNT_MAX/(SystemCoreClock/1000000);
	
	us = us*(SystemCoreClock/1000000); //Unit conversion
	
	//Used to save elapsed time
	uint32_t runningtime = 0;
	
	//Get the count value for the current moment
	uint32_t InserTick = DWT_GetTick();
	
	//Used to refresh real-time
	uint32_t tick = 0;
	
	uint8_t countflag = 0;
	//Wait for delay
	while(1)
	{
		tick = DWT_GetTick();//Refresh the current moment count value
		
		if( tick < InserTick ) countflag = 1;//If overflow polling occurs, switch to the timing calculation method
		
		if( countflag ) runningtime = tick + DWT_CNT_MAX - InserTick;
		else runningtime = tick - InserTick;
		
		if( runningtime>=us ) break;
	}
}

void delay_ms(uint32_t ms)
{
	//Exceeding the maximum delay that can be met
	if( ms > DWT_CNT_MAX/(SystemCoreClock/1000) ) ms = DWT_CNT_MAX/(SystemCoreClock/1000);
	delay_us(ms*1000);
}

