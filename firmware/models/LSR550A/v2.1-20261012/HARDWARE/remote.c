#include "remote.h"

REMOTER_t remoter;

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
Function: Aircraft model remote control initialization function, timer 1 inputs capture initialization
Input parameters: arr: auto-reinstall value, psc: clock predivision count
Return value: none
**************************************************************************/
//General pin configuration: Set the incoming pin to multiplex push-pull output
static void GPIO_AFPP_Init(GPIO_TypeDef* GPIOx,uint16_t PINx)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   =  PINx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

//Pin multiplexing configuration
static void GPIOx_PIN_AFConfig(GPIO_TypeDef* GPIOx,uint16_t PINx,TIM_TypeDef* TIMx)
{
	//Confirm the multiplexed pin number
	uint8_t PinSource=0;
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (PINx>>PinSource)&0x01 )==1 ) break;
	}
	
	//Confirm the timer for reuse
	//All timers except TIM6 and TIM7 have channels; use their corresponding alternate functions
	uint8_t GPIO_AF;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                       GPIO_AF = 0x01;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_AF = 0x02;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_AF = 0x03;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_AF = 0x09;
	
	//Reuse configuration
	GPIO_PinAFConfig(GPIOx,PinSource,GPIO_AF);
}

void Remoter_Param_Init(REMOTER_t *p)
{
	p->ch1 = 1500;
	p->ch2 = 1500;
	p->ch3 = 1000;
	p->ch4 = 1500;
	p->check_count = 0;
}

void Remoter_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
	
	//Pin and timer clock on
	ENABLE_REMOTE_TIM_CLOCK;
	ENABLE_REMOTE_CH1_PORT_CLOCK;
	ENABLE_REMOTE_CH2_PORT_CLOCK;
	ENABLE_REMOTE_CH3_PORT_CLOCK;
	ENABLE_REMOTE_CH4_PORT_CLOCK;
	
	//Set the corresponding pin to multiplex output
	GPIO_AFPP_Init(REMOTE_CH1_PORT,REMOTE_CH1_PIN);
	GPIO_AFPP_Init(REMOTE_CH2_PORT,REMOTE_CH2_PIN);
	GPIO_AFPP_Init(REMOTE_CH3_PORT,REMOTE_CH3_PIN);
	GPIO_AFPP_Init(REMOTE_CH4_PORT,REMOTE_CH4_PIN);
	
	//Pin multiplexing configuration
	GPIOx_PIN_AFConfig(REMOTE_CH1_PORT,REMOTE_CH1_PIN,REMOTE_TIM);
	GPIOx_PIN_AFConfig(REMOTE_CH2_PORT,REMOTE_CH2_PIN,REMOTE_TIM);
	GPIOx_PIN_AFConfig(REMOTE_CH3_PORT,REMOTE_CH3_PIN,REMOTE_TIM);
	GPIOx_PIN_AFConfig(REMOTE_CH4_PORT,REMOTE_CH4_PIN,REMOTE_TIM);
	

    /*** Initialize timer 1*/
    //Set the counter to automatically reload // Set the counter's automatic reload value
    TIM_TimeBaseStructure.TIM_Period = 9999;     //Reload value; reload when the count reaches 10000
    //Pre-divider // Pre-divider
    TIM_TimeBaseStructure.TIM_Prescaler = 167;   //Prescaler 168; advanced timer: 168M/168 = 1M ==> one count per 1us
    //Set the clock split: TDTS = Tck_tim // Set the clock split: TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //TIM up count mode // TIM up count mode
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
    //Initialize TIMx time base units based on the parameters specified in the TIM_TimeBaseInitStruct
    TIM_TimeBaseInit(REMOTE_TIM, &TIM_TimeBaseStructure);

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
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

    /*** Initialize TIM1 input capture parameters, channel 2*/
    //CC1S=01 Select input // Select the input end
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    //Rising edge capture // Rising edge capture
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    //Configure input frequency division, regardless of frequency // Configure input frequency division, no division
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 Configure input filter
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

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
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

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
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

    /*** interrupt packet initialization || Interrupt group initialization ***/
    //TIM1 interrupts // TIM1 interrupts
    NVIC_InitStructure.NVIC_IRQChannel = REMOTE_TIM_IRQn ;
    //Preempt priority 0 //Preempt priority level 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    //Level 0 from priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQ channels are enabled // IRQ channels are enabled
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
    //Initialize the peripheral NVIC register according to the parameters specified in the NVIC_InitStruct
    NVIC_Init(&NVIC_InitStructure);

    //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts, not allowed update_interrupts
    //Interrupts are not allowed to update; CC1IE, CC2IE, CC3IE, and CC4IE are allowed to capture interrupts
    TIM_ITConfig(REMOTE_TIM, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);

    //Enable timer
    TIM_Cmd(REMOTE_TIM, ENABLE);
}
