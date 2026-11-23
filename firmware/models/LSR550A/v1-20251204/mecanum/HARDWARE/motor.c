#include "motor.h"

/**************************************************************************
Function: Motor orientation pin initialization
Input   : none
Output  : none
Function: Initialize motor orientation pins
Entry parameters: None
Return value: none
**************************************************************************/
void LucSys_Motor_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);//Enable the GPIOB C D clock

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;//Corresponding to IO ports
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize GPIO
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12|GPIO_Pin_13 |GPIO_Pin_14;//| GPIO_Pin_15; 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
	GPIO_Init(GPIOB, &GPIO_InitStructure); 	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_12;//Corresponding to IO ports
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOC, &GPIO_InitStructure);//Initialize GPIO
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10|GPIO_Pin_12;//Corresponding to IO ports
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//50MHz
//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; Pull up GPIO_PuPd_NOPULL
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOD, &GPIO_InitStructure);//Initialize GPIO
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13 |GPIO_Pin_14);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_12 );
	GPIO_ResetBits(GPIOD,GPIO_Pin_10|GPIO_Pin_12);

}

/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
Function: Initialization of motor PWM pins
Entry parameters: arr: auto-reset value, psc: clock predivision count
Return value: none
**************************************************************************/
void LucSys_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	  //TIM8 clock enabled
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//Enable the PORTC clock
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //Reuse function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//Speed: 100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Push-pull reuse output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //Pull up
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //Initialize PC port
	
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
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

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
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); 
	
	// Advanced timer output must be enabled
	//This statement is required to enable advanced-timer output
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	
	//CH1 is pre-loaded and enabled
	//CH1 preloaded enable
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  

  // Enable the TIMX preloaded register on the ARR
  //Enable TIMx preload registers on ARR
	TIM_ARRPreloadConfig(TIM8, ENABLE); 
	
		 TIM8->CCR1=0;	
	 TIM8->CCR2=0;	
	 TIM8->CCR3=0;	
	 TIM8->CCR4=0;
	
	
	//Enable TIM8
	//Enable TIM8
	TIM_Cmd(TIM8, ENABLE);  
} 

/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
Function: Initialization of motor PWM pins
Entry parameters: arr: auto-reset value, psc: clock predivision count
Return value: none
**************************************************************************/
void TIM9_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	  //TIM9 clock enabled
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//Enable the PORTA clock
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM9); 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //Reuse function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//Speed: 100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Push-pull reuse output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //Pull up
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //Initialize PC port
	
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
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure); 

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
	TIM_OC2Init(TIM9, &TIM_OCInitStructure); 
	
	
	//CH is pre-loaded and enabled
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  

  // Enable the TIMX preloaded register on the ARR
  //Enable TIMx preload registers on ARR
	TIM_ARRPreloadConfig(TIM9, ENABLE); 
		
    TIM9->CCR2=1500;	
	
	
	//Enable TIM9
	//Enable TIM9
	TIM_Cmd(TIM9, ENABLE);  
} 

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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//Enable the GPIOB clock
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //Key corresponding pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//Normal input mode
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOE, &GPIO_InitStructure);//Initialize GPIOB14
} 

/**************************************************************************
Function: Model aircraft three channel remote control pin and steering gear PWM pin initialization
Input   : ARR: Automatic reload value  PSC: clock preset frequency
Output  : none
Function: Initialization of model aircraft three-channel remote control pins and servo PWM pins
Entry parameters: arr: auto-reset value, psc: clock predivision count
Return value: none
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc)	
{ 
	GPIO_InitTypeDef GPIO_InitStructure;           //IO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //Timer
	NVIC_InitTypeDef NVIC_InitStructure;           //Interrupt priority
	TIM_ICInitTypeDef TIM_ICInitStructure;         //Input capture
	TIM_OCInitTypeDef  TIM_OCInitStructure;        //PWM output
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1 clock enabled
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//Enable the PORTE clock
		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);

	/*** Initialize timer 1*/
	//Set the counter to automatically reload // Set the counter's automatic reload value
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider // Pre-divider
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim // Set the clock split: TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode // TIM up count mode
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//Initialize TIMx time base units based on the parameters specified in the TIM_TimeBaseInitStruct
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

	/*** Initialize TIM1 input capture parameters, channel 1*/
	//Select input //Select input
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  //Rising edge capture // Rising edge capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency // Configure input frequency division, no division
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
  //IC1F=0000 Configure input filter // Configure input filter
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** Initialize TIM1 input capture parameters, channel 2*/
	//CC1S=01 Select input // Select the input end
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	//Rising edge capture // Rising edge capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency // Configure input frequency division, no division
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 
	//IC1F=0000 Configure input filter // Configure input filter
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** Initialize TIM1 input capture parameters, channel 3*/
	//Select input //Select input
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;   
	//Rising edge capture // Rising edge capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency // Configure input frequency division, no division
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	//IC1F=0000 Configure input filter // Configure input filter
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	
	//----------- servo initialization -----------//
	//Select Timer mode :TIM Pulse Width Modulation mode 1
  //Select timer mode: TIM pulse-width modulation mode 1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//Compare output enablement
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//Set the pulse value of the capture comparison register to be loaded
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //Output polarity :TIM output polarity is higher	
  //Output polarity: TIM output compare active high
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   
	
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //Initialize the peripheral TIMx according to the parameters specified in the TIM_OCInitStruct
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); 
	//Channel preload enable
	//Enable channel preload
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	//----------- servo initialization -----------//

  /*** interrupt packet initialization || Interrupt group initialization ***/
  //TIM1 interrupts // TIM1 interrupts
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; 
  //Preempt priority 0 //Preempt priority level 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	//Level 0 from priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	//IRQ channels are enabled // IRQ channels are enabled
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
	//Initialize the peripheral NVIC register according to the parameters specified in the NVIC_InitStruct
	NVIC_Init(&NVIC_InitStructure);   

  //Enable the TIMX preloaded register on the ARR
  //Enable TIMx preload registers on ARR
	//TIM_ARRPreloadConfig(TIM1, ENABLE); 
  //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts, not allowed update_interrupts
  //Interrupts are not allowed to update; CC1IE, CC2IE, CC3IE, and CC4IE are allowed to capture interrupts
	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3,	ENABLE);   
	//Advanced timer output must be enabled // Advanced timer output must be enabled
	TIM_CtrlPWMOutputs(TIM1,ENABLE); 	
	//Enable timer
	TIM_Cmd(TIM1, ENABLE); 		 

  //The channel value is initialized to 1500, corresponding to the steering gear zero
	//Initialize channel value to 1500, corresponding to the servo zero position
	TIM1->CCR4=1500;
}
