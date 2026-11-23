#include "motor.h"

/**************************************************************************
Function: Motor orientation pin initialization
Input   : none
Output  : none
Function: Initialize motor orientation pins
Entry parameters: None
Return value: none
**************************************************************************/
//General pin configuration: Set the incoming pin to push-pull output mode
static void GPIO_Output_Init(GPIO_TypeDef* GPIOx,uint16_t PINx)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   =  PINx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

//Timer General Configuration: Set the input timer and corresponding pins to PWM1 output mode
void TIMx_PWM1_Mode_Init(GPIO_TypeDef* GPIOx,uint16_t PINx,TIM_TypeDef* TIMx,uint8_t TIM_Channel,u16 arr,u16 psc,u16 init_val)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
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
	
	//Configure corresponding pins for alternate function
    GPIO_InitStructure.GPIO_Pin = PINx;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
    GPIO_Init(GPIOx,&GPIO_InitStructure);           
	
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
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

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
	if( TIM_Channel==1 )
	{
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
		TIM_SetCompare1(TIMx,init_val);
	}
	else if( TIM_Channel==2 )
	{
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
		TIM_SetCompare2(TIMx,init_val);
	}
	else if( TIM_Channel==3 )
	{
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
		TIM_SetCompare3(TIMx,init_val);
	}
	else if( TIM_Channel==4 )
	{
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
		TIM_SetCompare4(TIMx,init_val);
	}
	
	
	if( TIMx==TIM1 || TIMx==TIM8  || TIMx==TIM9)
	{
		// Advanced timer output must be enabled
		//This statement is required to enable advanced-timer output
		TIM_CtrlPWMOutputs(TIMx,ENABLE);
	}

    // Enable the TIMX preloaded register on the ARR
    //Enable TIMx preload registers on ARR
    TIM_ARRPreloadConfig(TIMx, ENABLE);
	
    //Enable TIM
    //Enable timers
    TIM_Cmd(TIMx, ENABLE);
}

static void LucSys_Motor_Init(void)
{
	//Turn on the corresponding pin clock
	ENABLE_AIN1_CLOCK;
	ENABLE_AIN2_CLOCK;
	ENABLE_BIN1_CLOCK;
	ENABLE_BIN2_CLOCK;
	ENABLE_CIN1_CLOCK;
	ENABLE_CIN2_CLOCK;
	ENABLE_DIN1_CLOCK;
	ENABLE_DIN2_CLOCK;
	
	//Set the corresponding pin push-pull output
	GPIO_Output_Init(AIN1_PORT,AIN1_PIN);
	GPIO_Output_Init(AIN2_PORT,AIN2_PIN);
	
	GPIO_Output_Init(BIN1_PORT,BIN1_PIN);
	GPIO_Output_Init(BIN2_PORT,BIN2_PIN);
	
	GPIO_Output_Init(CIN1_PORT,CIN1_PIN);
	GPIO_Output_Init(CIN2_PORT,CIN2_PIN);
	
	GPIO_Output_Init(DIN1_PORT,DIN1_PIN);
	GPIO_Output_Init(DIN2_PORT,DIN2_PIN);
	
	AIN1 = 0; AIN2 = 0;
	BIN1 = 0; BIN2 = 0;
	CIN1 = 0; CIN2 = 0;
	DIN1 = 0; DIN2 = 0;
}

static void V1_0_LucSys_Motor_Init(void)
{
	//Turn on the corresponding pin clock
	ENABLE_AIN1_CLOCK;
	ENABLE_AIN2_CLOCK;
	V1_0_ENABLE_BIN1_CLOCK;
	V1_0_ENABLE_BIN2_CLOCK;
	ENABLE_CIN1_CLOCK;
	ENABLE_CIN2_CLOCK;
	ENABLE_DIN1_CLOCK;
	ENABLE_DIN2_CLOCK;
	
	//Set the corresponding pin push-pull output
	GPIO_Output_Init(AIN1_PORT,AIN1_PIN);
	GPIO_Output_Init(AIN2_PORT,AIN2_PIN);
	
	GPIO_Output_Init(V1_0_BIN1_PORT,V1_0_BIN1_PIN);
	GPIO_Output_Init(V1_0_BIN2_PORT,V1_0_BIN2_PIN);
	
	GPIO_Output_Init(CIN1_PORT,CIN1_PIN);
	GPIO_Output_Init(CIN2_PORT,CIN2_PIN);
	
	GPIO_Output_Init(DIN1_PORT,DIN1_PIN);
	GPIO_Output_Init(DIN2_PORT,DIN2_PIN);
	
	AIN1 = 0; AIN2 = 0;
	V1_0_BIN1 = 0; V1_0_BIN2 = 0;
	CIN1 = 0; CIN2 = 0;
	DIN1 = 0; DIN2 = 0;
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
	LucSys_Motor_Init();//Initialize the motor direction pin
	
	//Enable PWM-corresponding timers and pin port clocks
	ENABLE_PWMA_TIM_CLOCK;
	ENABLE_PWMA_PORT_CLOCK;
	
	ENABLE_PWMB_TIM_CLOCK;
	ENABLE_PWMB_PORT_CLOCK;
	
	ENABLE_PWMC_TIM_CLOCK;
	ENABLE_PWMC_PORT_CLOCK;
	
	ENABLE_PWMD_TIM_CLOCK;
	ENABLE_PWMD_PORT_CLOCK;
	
	//Configure the corresponding timer and pin to PWM1 mode
	TIMx_PWM1_Mode_Init(PWMA_PORT,PWMA_PIN,PWMA_TIM,PWMA_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMB_PORT,PWMB_PIN,PWMB_TIM,PWMB_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMC_PORT,PWMC_PIN,PWMC_TIM,PWMC_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMD_PORT,PWMD_PIN,PWMD_TIM,PWMD_Channel,arr,psc,0);
	
}

void V1_0_LucSys_PWM_Init(u16 arr,u16 psc)
{
	V1_0_LucSys_Motor_Init();//Initialize the motor direction pin
	
	//Enable PWM-corresponding timers and pin port clocks
	ENABLE_PWMA_TIM_CLOCK;
	ENABLE_PWMA_PORT_CLOCK;
	
	ENABLE_PWMB_TIM_CLOCK;
	ENABLE_PWMB_PORT_CLOCK;
	
	ENABLE_PWMC_TIM_CLOCK;
	ENABLE_PWMC_PORT_CLOCK;
	
	ENABLE_PWMD_TIM_CLOCK;
	ENABLE_PWMD_PORT_CLOCK;
	
	//Configure the corresponding timer and pin to PWM1 mode
	TIMx_PWM1_Mode_Init(PWMA_PORT,PWMA_PIN,PWMA_TIM,PWMA_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMB_PORT,PWMB_PIN,PWMB_TIM,PWMB_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMC_PORT,PWMC_PIN,PWMC_TIM,PWMC_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMD_PORT,PWMD_PIN,PWMD_TIM,PWMD_Channel,arr,psc,0);
}

/**************************************************************************
Function: Senior Ackerman model servo initialization
Input   : ARR: Automatic reload value  PSC: clock preset frequency
Output  : none
Function: servo initialization for high-end Ackermann models
Entry parameters: arr: auto-reset value, psc: clock predivision count
Return value: none
**************************************************************************/
void Servo_Senior_Init(u16 arr,u16 psc,int servomid)
{
	ENABLE_SERVO_SENIOR_TIM_CLOCK;
	ENABLE_SERVO_SENIOR_PORT_CLOCK;
	TIMx_PWM1_Mode_Init(SERVO_SENIOR_PORT,SERVO_SENIOR_PIN,SERVO_SENIOR_TIM,SERVO_SENIOR_Channel,arr,psc,servomid);
}

void Servo_Top_Init(u16 arr,u16 psc,int servomid)
{
	ENABLE_SERVO_TOP_TIM_CLOCK;
	ENABLE_SERVO_TOP_PORT_CLOCK;
	TIMx_PWM1_Mode_Init(SERVO_TOP_PORT,SERVO_TOP_PIN,SERVO_TOP_TIM,SERVO_TOP_Channel,arr,psc,servomid);
}
