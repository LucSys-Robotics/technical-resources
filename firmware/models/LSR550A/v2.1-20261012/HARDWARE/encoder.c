#include "encoder.h"

//Generic encoder initialization: configure the given timer and corresponding pins in encoder mode 3
static void Encoder_TI12_ModeInit(GPIO_TypeDef* GPIOx_1,uint16_t GPIO_PIN_1,GPIO_TypeDef* GPIOx_2,uint16_t GPIO_PIN_2,TIM_TypeDef* TIMx)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
   
	//Pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx_1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx_2, &GPIO_InitStructure);
	
	//Confirm the timer for reuse
	//All timers except TIM6 and TIM7 have channels; use their corresponding alternate functions
	uint8_t GPIO_AF;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                      GPIO_AF = 0x01;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_AF = 0x02;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_AF = 0x03;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_AF = 0x09;
	
	//Confirm the pin number of pin 1 that is multiplexed
	uint8_t PinSource=0;
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (GPIO_PIN_1>>PinSource)&0x01 )==1 ) break;
	}
	//Reuse configuration
	GPIO_PinAFConfig(GPIOx_1,PinSource,GPIO_AF);
	
	//Confirm the pin number of pin 2 that is multiplexed
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (GPIO_PIN_2>>PinSource)&0x01 )==1 ) break;
	}
	//Reuse configuration
	GPIO_PinAFConfig(GPIOx_2,PinSource,GPIO_AF);
	
	
	//Timer configuration
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 				    //No frequency division
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;      //Set the counter's auto-reset value
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //Select clock division: no division
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM counts up
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);    //Initialize the timer

	//Use encoder mode 3
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	//Filter coefficient set to 0
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
	
	//Clear the update flag of TIM
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	//Clear count values
    TIM_SetCounter(TIMx,0);
	//Start the timer
    TIM_Cmd(TIMx, ENABLE);
}

/**************************************************************************
Function: Encoder interface A initialization
Input   : none
Output  : none
Function: Initialize encoder interface A
Entry parameters: None
Return value: none
**************************************************************************/
void EncoderA_Init(void)
{
    ENABLE_ENCODER_A_TIM_CLOCK;   //Enable timers
	ENBALE_ENCODER_A1_PORT_CLOCK; //Enable the port corresponding to pin 1
	ENBALE_ENCODER_A2_PORT_CLOCK; //Enable the port corresponding to pin 2
	
	//Configure encoder A in encoder mode 3
	Encoder_TI12_ModeInit(ENCODER_A1_PORT,ENCODER_A1_PIN,ENCODER_A2_PORT,ENCODER_A2_PIN,ENCODER_A_TIM);
}


void EncoderB_Init(void)
{
    ENABLE_ENCODER_B_TIM_CLOCK;   //Enable timers
	ENBALE_ENCODER_B1_PORT_CLOCK; //Enable the port corresponding to pin 1
	ENBALE_ENCODER_B2_PORT_CLOCK; //Enable the port corresponding to pin 2
	
	//Configure encoder B in encoder mode 3
	Encoder_TI12_ModeInit(ENCODER_B1_PORT,ENCODER_B1_PIN,ENCODER_B2_PORT,ENCODER_B2_PIN,ENCODER_B_TIM);
}


void EncoderC_Init(void)
{
    ENABLE_ENCODER_C_TIM_CLOCK;   //Enable timers
	ENBALE_ENCODER_C1_PORT_CLOCK; //Enable the port corresponding to pin 1
	ENBALE_ENCODER_C2_PORT_CLOCK; //Enable the port corresponding to pin 2
   
	//Configure encoder C in encoder mode 3
	Encoder_TI12_ModeInit(ENCODER_C1_PORT,ENCODER_C1_PIN,ENCODER_C2_PORT,ENCODER_C2_PIN,ENCODER_C_TIM);
}


void EncoderD_Init(void)
{
    ENABLE_ENCODER_D_TIM_CLOCK;   //Enable timers
	ENBALE_ENCODER_D1_PORT_CLOCK; //Enable the port corresponding to pin 1
	ENBALE_ENCODER_D2_PORT_CLOCK; //Enable the port corresponding to pin 2
   
	//Configure encoder D in encoder mode 3
	Encoder_TI12_ModeInit(ENCODER_D1_PORT,ENCODER_D1_PIN,ENCODER_D2_PORT,ENCODER_D2_PIN,ENCODER_D_TIM);
}


/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
Function: Read encoder counts
Entry parameters: Timer
Return value: encoder value (representing speed)
**************************************************************************/
short Read_Encoder(ENCODER_t e)
{
    short Encoder_TIM;
    switch(e)
    {
		case Encoder_A:
			Encoder_TIM = (short)ENCODER_A_TIM -> CNT;
			ENCODER_A_TIM -> CNT=0;
			break;
		case Encoder_B:
			Encoder_TIM = (short)ENCODER_B_TIM -> CNT;
			ENCODER_B_TIM -> CNT=0;
			break;
		case Encoder_C:
			Encoder_TIM = (short)ENCODER_C_TIM -> CNT;
			ENCODER_C_TIM -> CNT=0;
			break;
		case Encoder_D:
			Encoder_TIM = (short)ENCODER_D_TIM -> CNT;
			ENCODER_D_TIM -> CNT=0;
			break;
		default:
			Encoder_TIM=0;
    }
    return Encoder_TIM;
}

