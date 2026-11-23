#include "timer.h"

//Input the capture flag for channel 1, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Channel 1 input-capture flags: upper two bits for capture flags, lower 6 bits for overflow flags
u8 TIM1CH1_CAPTURE_STA = 0;	
u16 TIM1CH1_CAPTURE_UPVAL;
u16 TIM1CH1_CAPTURE_DOWNVAL;

//Input the capture flag for channel 2, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Channel 2 input-capture flags: upper two bits for capture flags, lower 6 bits for overflow flags
u8 TIM1CH2_CAPTURE_STA = 0;		
u16 TIM1CH2_CAPTURE_UPVAL;
u16 TIM1CH2_CAPTURE_DOWNVAL;

//Input the capture flag for channel 3, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Channel 3 input-capture flags: upper two bits for capture flags, lower 6 bits for overflow flags
u8 TIM1CH3_CAPTURE_STA = 0;		
u16 TIM1CH3_CAPTURE_UPVAL;
u16 TIM1CH3_CAPTURE_DOWNVAL;

//Input the capture flag for channel 4, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Channel 4 input-capture flags: upper two bits for capture flags, lower 6 bits for overflow flags
u8 TIM1CH4_CAPTURE_STA = 0;			
u16 TIM1CH4_CAPTURE_UPVAL;
u16 TIM1CH4_CAPTURE_DOWNVAL;

u32 TIM1_T1;
u32 TIM1_T2;
u32 TIM1_T3;
u32 TIM1_T4;

//Variables related to remote control acquisition of model aircraft
//Variables related to RC transmitter acquisition
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//Model aircraft remote control receiver variable
//RC transmitter receiving variables
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;  

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
Function: Aircraft model remote control initialization function, timer 1 inputs capture initialization
Input parameters: arr: auto-reinstall value, psc: clock predivision count
Return value: none
**************************************************************************/ 
void TIM1_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1 clock enabled
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//Enable the PORTE clock
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//Reuse function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//Speed: 100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Push-pull reuse output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //Pull down
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
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** Initialize TIM1 input capture parameters, channel 2*/
	//CC1S=01 Select input // Select the input end
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	//Rising edge capture // Rising edge capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency // Configure input frequency division, no division
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 Configure input filter
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** Initialize TIM1 input capture parameters, channel 3*/
	//Select input //Select input
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;   
	//Rising edge capture // Rising edge capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency // Configure input frequency division, no division
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	//IC1F=0000 Configure input filter // Configure input filter, no filtering
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** Initialize TIM1 input capture parameters, channel 4*/
	//Select input //Select input
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
	//Rising edge capture // Rising edge capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency // Configure input frequency division, no division
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	   
	//IC1F=0000 Configure input filter // Configure input filter, no filtering
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

  /*** interrupt packet initialization || Interrupt group initialization ***/
  //TIM1 interrupts // TIM1 interrupts
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; 
  //Preempt priority 0 //Preempt priority level 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	//Level 0 from priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	//IRQ channels are enabled // IRQ channels are enabled
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
	//Initialize the peripheral NVIC register according to the parameters specified in the NVIC_InitStruct
	NVIC_Init(&NVIC_InitStructure);   
	
  //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts, not allowed update_interrupts
  //Interrupts are not allowed to update; CC1IE, CC2IE, CC3IE, and CC4IE are allowed to capture interrupts
	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE);   
	//Advanced timer output must be enabled // Advanced timer output must be enabled
	TIM_CtrlPWMOutputs(TIM1,ENABLE); 	
	//Enable timer
	TIM_Cmd(TIM1, ENABLE); 		
}
/**************************************************************************
Function: Model aircraft remote control receiving interrupt, namely timer 1 input capture interrupt
Input   : none
Output  : none
Function: RC transmitter receives interrupts, i.e., timer 1 captures interrupts at input
Entry parameters: None
Return value: none
**************************************************************************/ 
void TIM1_CC_IRQHandler(void)
{
	static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;
	
	//After connecting the RC transmitter, push the forward stick before RC control of the chassis is enabled
	//After connecting the remote controller of the model aircraft, 
	//you need to push down the forward lever to officially control the car of the model aircraft
  if(Remoter_Ch2>1600&&Remote_ON_Flag==0&&Deviation_Count>=CONTROL_DELAY)
  {
		//Model aircraft remote control mark position 1, other marks position 0
		//Set RC-control flag to 1 and all other flags to 0
		Remote_ON_Flag=1;
	  APP_ON_Flag=0;
		PS2_ON_Flag=0;
		CAN_ON_Flag=0;
		Usart_ON_Flag=0; 
	}
	 
	//Channel 1 // Channel 1
	if ((TIM1CH1_CAPTURE_STA & 0X80) == 0) 			
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) //A capture event occurred on channel 1 // Capture event occurred on channel 1
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //Clear the interrupt flag bit // Clear the interrupt flag bit
			if (TIM1CH1_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
			{
				TIM1CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM1); //Record the timer value at this point // Record the timer value at this moment
				if (TIM1CH1_CAPTURE_DOWNVAL < TIM1CH1_CAPTURE_UPVAL)
				{
					TIM1_T1 = 9999;
				}
				else
					TIM1_T1 = 0;
				Remoter_Ch1 = TIM1CH1_CAPTURE_DOWNVAL - TIM1CH1_CAPTURE_UPVAL + TIM1_T1;	//Time to get the total high level // Time to get the total high level
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch1_filter_times++;
					if( ch1_filter_times<=5 ) Remoter_Ch1=L_Remoter_Ch1; //Filter //Filter
					else ch1_filter_times=0;
				}
				else
				{
					ch1_filter_times=0;
				}
				
				L_Remoter_Ch1=Remoter_Ch1;
				
				TIM1CH1_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture // Set to rising edge capture
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
				TIM1CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM1); //Obtain rising edge data
				TIM1CH1_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture // Set to falling edge capture
			}
		}
	}
  //Channel 2 // Channel 2
	if ((TIM1CH2_CAPTURE_STA & 0X80) == 0)		
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)	//A capture event occurred on channel 2 // Capture event occurred on channel 2
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //Clear the interrupt flag bit // Clear the interrupt flag bit
			if (TIM1CH2_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
			{
				TIM1CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM1); //Record the timer value at this point // Record the timer value at this moment
				if (TIM1CH2_CAPTURE_DOWNVAL < TIM1CH2_CAPTURE_UPVAL)
				{
					TIM1_T2 = 9999;
				}
				else
					TIM1_T2 = 0;
				Remoter_Ch2 = TIM1CH2_CAPTURE_DOWNVAL - TIM1CH2_CAPTURE_UPVAL + TIM1_T2; //Time to get the total high level // Time to get the total high level
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch2_filter_times++;
					if( ch2_filter_times<=5 ) Remoter_Ch2=L_Remoter_Ch2; //Filter //Filter
					else ch2_filter_times=0;
				}
				else
				{
					ch2_filter_times=0;
				}
				
				L_Remoter_Ch2=Remoter_Ch2;
				
				TIM1CH2_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture // Set to rising edge capture
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
				TIM1CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM1); //Obtain rising edge data
				TIM1CH2_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture // Set to falling edge capture
			}
		}
	}
  //Channel 3 // Channel 3
	if ((TIM1CH3_CAPTURE_STA & 0X80) == 0)			
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)	//A capture event occurred on channel 3 // Capture event occurred on channel 3
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //Clear the interrupt flag bit // Clear the interrupt flag bit
			if (TIM1CH3_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
			{
				TIM1CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM1); //Record the timer value at this point // Record the timer value at this moment
				if (TIM1CH3_CAPTURE_DOWNVAL < TIM1CH3_CAPTURE_UPVAL)
				{
					TIM1_T3 = 9999;
				}
				else
					TIM1_T3 = 0;
				Remoter_Ch3 = TIM1CH3_CAPTURE_DOWNVAL - TIM1CH3_CAPTURE_UPVAL + TIM1_T3; //Time to get the total high level // Time to get the total high level
				if(abs(Remoter_Ch3-L_Remoter_Ch3)>500)
				{
					ch3_filter_times++;
					if( ch3_filter_times<=5 ) Remoter_Ch3=L_Remoter_Ch3; //Filter //Filter
					else ch3_filter_times=0;
				}
				else
				{
					ch3_filter_times=0;
				}
				L_Remoter_Ch3=Remoter_Ch3;	
				TIM1CH3_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture // Set to rising edge capture
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
				TIM1CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM1); //Obtain rising edge data
				TIM1CH3_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture // Set to falling edge capture
			}
		}
	}

		//Channel 4 // Channel 4
		if ((TIM1CH4_CAPTURE_STA & 0X80) == 0)		
		{
			if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)	//A capture event occurred on channel 4 // A capture event occurred on channel 4
			{
				TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //Clear the interrupt flag bit // Clear the interrupt flag bit
				if (TIM1CH4_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
				{
					TIM1CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM1); //Record the timer value at this point // Record the timer value at this moment
					if (TIM1CH4_CAPTURE_DOWNVAL < TIM1CH4_CAPTURE_UPVAL)
					{
						TIM1_T4 = 9999;
					}
					else
						TIM1_T4 = 0;
					Remoter_Ch4 = TIM1CH4_CAPTURE_DOWNVAL - TIM1CH4_CAPTURE_UPVAL + TIM1_T4; //Time to get the total high level // Time to get the total high level
					if(abs(Remoter_Ch4-L_Remoter_Ch4)>500)
					{
						ch4_filter_times++;
						if( ch4_filter_times<=5 ) Remoter_Ch4=L_Remoter_Ch4; //Filter //Filter
						else ch4_filter_times=0;
					}
					else
					{
						ch4_filter_times=0;
					}
					
					L_Remoter_Ch4=Remoter_Ch4;				
					TIM1CH4_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
					TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture // Set to rising edge capture
				}
				else 
				{
					//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				  //Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
					TIM1CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM1); //Obtain rising edge data
					TIM1CH4_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
					TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture // Set to falling edge capture
				}
			}
		}
}
/**************************************************************************
Function: TIM1 Update Interrupt
Input   : none
Output  : none
Function: Timer 1 updates interrupt
Entry parameters: None
Return value: none
**************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void) 
{ 
	//Clear the interrupt flag bit
	//Clear interrupt flags
  TIM1->SR&=~(1<<0);	    
}

