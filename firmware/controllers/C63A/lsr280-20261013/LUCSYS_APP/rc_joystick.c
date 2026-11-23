#include "rc_joystick.h"

typedef struct{
	uint8_t sm;
	uint32_t upvalue;
	uint32_t downvalue;
	TIM_HandleTypeDef* tim;
	uint64_t ch;
	uint16_t val;
}RCJoystickChannelType_t;

enum {
	WaitUp=0,
	WaitDown,
};

RC_REMOTE_t rc_remote = {
	.LX = 1500,
	.LY = 1500,
	.RX = 1500,
	.RY = 1000
};

//Hardware mount
static RCJoystickChannelType_t JoyRX = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_1
};

static RCJoystickChannelType_t JoyLY = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_2
};

static RCJoystickChannelType_t JoyRY = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_3
};

static RCJoystickChannelType_t JoyLX = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_4
};

#include "bsp_RTOSdebug.h"
static pRTOSDebugInterface_t debug = &RTOSDebugTimer;
static RTOSDebugPrivateVar var = { 0 };

//Processes content captured by the channel
static void RCJoystickHandle_Result(RCJoystickChannelType_t* hal,BaseType_t* wokenTask)
{
	switch (hal->sm)
	{
		case WaitUp:
		{
			//Read out the count value
			hal->upvalue = HAL_TIM_ReadCapturedValue(hal->tim,hal->ch);
			
			//Set down edge capture
			TIM_RESET_CAPTUREPOLARITY(hal->tim,hal->ch);
			TIM_SET_CAPTUREPOLARITY(hal->tim,hal->ch,TIM_ICPOLARITY_FALLING);
			
			//State machine switching
			hal->sm = WaitDown;
			break;
		}
		
		case WaitDown:
		{
			//Read count values
			hal->downvalue = HAL_TIM_ReadCapturedValue(hal->tim,hal->ch);
			
			//Calculate distance
			if( hal->upvalue > hal->downvalue )//Spillover calculation
				hal->val = hal->downvalue+9999 - hal->upvalue;
			else
				hal->val = hal->downvalue - hal->upvalue;
			
			if( hal->ch == TIM_CHANNEL_1 ) rc_remote.RX = hal->val;
			else if( hal->ch == TIM_CHANNEL_2 ) rc_remote.LY = hal->val;
			else if( hal->ch == TIM_CHANNEL_3 ) rc_remote.RY = hal->val;
			else if( hal->ch == TIM_CHANNEL_4 ) rc_remote.LX = hal->val;
			
			//Set rising edge capture
			TIM_RESET_CAPTUREPOLARITY(hal->tim,hal->ch);
			TIM_SET_CAPTUREPOLARITY(hal->tim,hal->ch,TIM_ICPOLARITY_RISING);
			
			//State machine switching
			hal->sm = WaitUp;
			
			break;
		}
	}		
}

static RCJoystickChannelType_t* RCJoylist[] = { 
	&JoyLX,&JoyLY,&JoyRX,&JoyRY
};

//Returns the corresponding channel value based on the active timer channel, interrupting usage
static uint32_t get_tim_activate_ch(HAL_TIM_ActiveChannel active_ch)
{
	switch (active_ch) {
		case HAL_TIM_ACTIVE_CHANNEL_1: return TIM_CHANNEL_1;
		case HAL_TIM_ACTIVE_CHANNEL_2: return TIM_CHANNEL_2;
		case HAL_TIM_ACTIVE_CHANNEL_3: return TIM_CHANNEL_3;
		case HAL_TIM_ACTIVE_CHANNEL_4: return TIM_CHANNEL_4;
		default: return 0xFF;
	}
}

#include "RobotControl_task.h"

//Model aircraft handle
void RCJoystick_IC_CaptureCallback(TIM_HandleTypeDef *htim,BaseType_t* woken)
{
	//Match the corresponding ultrasonic waves according to the hardware interface
	for(uint8_t i=0;i<4;i++)
	{
		//Acquire data from different ultrasonic sensors by matching timer and timer channel
		if( RCJoylist[i]->tim == htim && RCJoylist[i]->ch == get_tim_activate_ch(htim->Channel) )
		{
			RCJoystickHandle_Result(RCJoylist[i],woken);
		}
	}
	
	//Reduce update frequency; update only after a joystick channel acquisition completes
	if( htim != JoyRX.tim || get_tim_activate_ch(htim->Channel)!=JoyRX.ch || 
		JoyRX.sm!=WaitDown ) return;
		
	rc_remote.freq = debug->UpdateFreq(&var);
	
	RobotControlCMDType_t cmd = { 
		.cmdsource = RCJOY_CMD,
		0,0,0
	};

    //limiter // Limit
    int LX,LY,RY,RX,Remote_RCvelocity;
	float base_vz = PI/4.0f ;//Z-axis speed reference

	//Stroke threshold
	const uint8_t Yuzhi=100;
	
	//Four channels with raw value limits
	rc_remote.LX = target_limit_int(rc_remote.LX,1000,2000);
	rc_remote.LY = target_limit_int(rc_remote.LY,1000,2000);
	rc_remote.RX = target_limit_int(rc_remote.RX,1000,2000);
	rc_remote.RY = target_limit_int(rc_remote.RY,1000,2000);

    //Front and back direction of left rocker. Control forward and backward.
    //Left stick forward and backward direction. Controls forward and backward movement.
    LX=rc_remote.LY-1500;
    
	//The left joystick's horizontal directions control lateral \
	  movement for omnidirectional mobile vehicles.
    //Left joystick moves left and right; for omnidirectional trolleys, control lateral movement left and right
    LY=rc_remote.LX-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
    //Right stick forward and backward. Throttle/acceleration/deceleration.
    RX=rc_remote.RY-1500;
	
    //Right stick left and right. To control the rotation.
    //Right stick left and right direction. Controls rotation.
    RY=-(rc_remote.RX-1500);//Rotation

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;

	//Throttle adjustment speed Remote_RCvelocity value: 0~1000
    Remote_RCvelocity= RobotControlParam.defalutSpeed + RX;
    if(Remote_RCvelocity<0)Remote_RCvelocity=0;

    //The remote control command of model aircraft is processed
    //Processing remote control commands for model aircraft
	cmd.Vx = LX * (float)Remote_RCvelocity/500.0f;
	// LSR280 supports lateral motion on receiver channel 4 (left stick X).
	if( RobotHardWareParam.CarType == LSR280 )
		cmd.Vy = -LY * (float)Remote_RCvelocity/500.0f;
	else
		cmd.Vy = 0;
	//Reference: base_vz, Own multiplier: RY/500, Value: [-1~1], Throttle multiplier: (Remote_RCvelocity/500.0f), Value: [0,2]
	cmd.Vz = base_vz*((float)RY/500.0f) * ((float)Remote_RCvelocity/500.0f) ; 

    //Unit conversion, mm/s -> m/s
    //Unit conversion, mm/s - > m/s
	cmd.Vx/=1000.0f;
	cmd.Vy/=1000.0f;
	
	//Switches Z-axis speed when reversing, following intuitive control logic
	if(cmd.Vx<0)cmd.Vz=-cmd.Vz;
	
	//Model aircraft remote unlock condition: Keep pushing forward [Forward], with the joystick held for 2 seconds
	static uint16_t filterCounts = 0;
	static uint32_t LastTick = 0;
	static uint8_t RCUnlock = 0;
	if( RCUnlock==0 )
	{
		if( cmd.Vx>0 && ((xTaskGetTickCountFromISR()-LastTick)<50) )
		{
			filterCounts++;
		}
		else filterCounts=0;
		
		if( filterCounts > rc_remote.freq*2 )
		{
			filterCounts = rc_remote.freq*2+10;
			RCUnlock = 1;
		}
		
		//Record the current moment
		LastTick = xTaskGetTickCountFromISR();
	}
	
	//Determines whether to perform queue writes and does not write when idle
	uint8_t writeflag=1;
	static uint8_t idleCount = 0;
	if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
	else idleCount=0;
	if( idleCount>10 ) writeflag=0,idleCount=10;
	
	//Write data when the control conditions are met
	if( writeflag && RCUnlock )
	{
		if( WriteRobotControlQueue(&cmd,woken) )
		{
			filterCounts=0;RCUnlock=0; //If writing fails, the model aircraft needs to be re-unlocked
		}
	}
    
}




