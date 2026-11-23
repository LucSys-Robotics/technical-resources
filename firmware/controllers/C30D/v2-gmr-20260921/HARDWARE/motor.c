#include "motor.h"


/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
Function: Initialize the enable switch pin
Entry parameters: None
Return value: none
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//Enable the GPIOB clock
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //Key corresponding pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//Normal input mode
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOD, &GPIO_InitStructure);//Initialize GPIOB14
} 

void TIM1_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	  //TIM8 clock enabled
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//Enable the PORTC clock
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //Reuse function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//Speed: 100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Push-pull reuse output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //Pull up
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //Initialize PC port
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//Set the value of the automatic reload register cycle for the next update event to load the activity
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//Set the predivision value used as the divisor of the TIMx clock frequency
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//Set clock split: TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	//Up counting mode 
	//Count Up mode
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//Initialize TIMx time base units based on the parameters specified in the TIM_TimeBaseInitStruct
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //Select timer mode: TIM pulse-width modulation mode 1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//Compare output enablement
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  //Output polarity :TIM output polarity is higher	
  //Output polarity: TIM output compare active high
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //Initialize the peripheral TIMx according to the parameters specified in the TIM_OCInitStruct
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); 
	
	// Advanced timer output must be enabled
	//This statement is required to enable advanced-timer output
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	//CH1 is pre-loaded and enabled
	//CH1 preloaded enable
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  

  // Enable the TIMX preloaded register on the ARR
  //Enable TIMx preload registers on ARR
	TIM_ARRPreloadConfig(TIM1, ENABLE); 
	
	//Enable TIM8
	//Enable TIM8
	TIM_Cmd(TIM1, ENABLE);  
}

void TIM9_PWM_Init(u16 arr,u16 psc)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);/*Enable the timer 11 clock*/
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);/*Enable the GPIOF clock*/

GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);/*Reuse*/
GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);/*Reuse*/

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;           //GPIOF9
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //Reuse function
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//Speed: 100MHz
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Push-pull reuse output
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //Pull up
GPIO_Init(GPIOE,&GPIO_InitStructure);              //Initialize PF9

TIM_TimeBaseInitStructure.TIM_Period = arr;/*Automatic reloading*/
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*Prescaler*/
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*Clock frequency division*/
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*Count up*/
TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitStructure);/*Initialize*/

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWM mode*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*Output*/
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*Relatively high polarity*/
TIM_OC1Init(TIM9,&TIM_OCInitStructure);
TIM_OC2Init(TIM9,&TIM_OCInitStructure);

TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);/*Enable output compare preload*/
TIM_OC2PreloadConfig(TIM9,TIM_OCPreload_Enable);/*Enable output compare preload*/

TIM_ARRPreloadConfig(TIM9,ENABLE);/*Automatic reload preload enable*/

TIM_Cmd(TIM9,ENABLE);/*Count enable*/

}


void TIM10_PWM_Init(u16 arr,u16 psc)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);/*Enable the GPIOF clock*/
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);/*Enable the timer 11 clock*/

GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM10);/*Reuse*/

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*Reuse*/
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*Push-pull output*/
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;/*PF7*/
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*Pull up*/
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
GPIO_Init(GPIOB,&GPIO_InitStructure);/*Initialize IO*/

TIM_TimeBaseInitStructure.TIM_Period = arr;/*Automatic reloading*/
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*Prescaler*/
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*Clock frequency division*/
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*Count up*/
TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);/*Initialize*/

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWM mode*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*Output*/
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*Relatively high polarity*/
TIM_OC1Init(TIM10,&TIM_OCInitStructure);

TIM_OC1PreloadConfig(TIM10,TIM_OCPreload_Enable);/*Enable output compare preload*/

//TIM_CtrlPWMOutputs(TIM10,ENABLE);

TIM_ARRPreloadConfig(TIM10,ENABLE);/*Automatic reload preload enable*/

TIM_Cmd(TIM10,ENABLE);/*Count enable*/

}

void TIM11_PWM_Init(u16 arr,u16 psc)
{

TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);/*Enable the GPIOF clock*/
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);/*Enable the timer 11 clock*/

GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM11);/*Reuse*/

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*Reuse*/
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*Push-pull output*/
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;/*PF7*/
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*Pull up*/
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
GPIO_Init(GPIOB,&GPIO_InitStructure);/*Initialize IO*/

TIM_TimeBaseInitStructure.TIM_Period = arr;/*Automatic reloading*/
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*Prescaler*/
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*Clock frequency division*/
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*Count up*/
TIM_TimeBaseInit(TIM11,&TIM_TimeBaseInitStructure);/*Initialize*/

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWM mode*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*Output*/
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*Relatively high polarity*/
TIM_OC1Init(TIM11,&TIM_OCInitStructure);

TIM_OC1PreloadConfig(TIM11,TIM_OCPreload_Enable);/*Enable output compare preload*/

TIM_CtrlPWMOutputs(TIM11,ENABLE);

TIM_ARRPreloadConfig(TIM11,ENABLE);/*Automatic reload preload enable*/

TIM_Cmd(TIM11,ENABLE);/*Count enable*/

}








