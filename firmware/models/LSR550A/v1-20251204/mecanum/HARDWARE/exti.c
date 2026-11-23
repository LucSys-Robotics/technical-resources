#include "exti.h"
  /**************************************************************************
By LucSys Robotics
**************************************************************************/
/**************************************************************************
Function: Initialize external interrupts
Entry parameters: None
Return value: none
**************************************************************************/
void EXTI1_Init(void)
{
//	RCC->APB2ENR|=1<<3; //Enable PORTB clock
//	GPIOB->CRH&=0X0FFFFFFF; 
//	GPIOB->CRH|=0X80000000; PB5 pulls up input
//  GPIOB->ODR|=1<<15; PB5 pulls up
//	Ex_NVIC_Config(GPIO_B,15,FTIR); // Trigger on falling edge
//	MY_NVIC_Init(2,1,EXTI15_10_IRQn,2); Preemption 2, Sub-priority 1, Group 2
}










