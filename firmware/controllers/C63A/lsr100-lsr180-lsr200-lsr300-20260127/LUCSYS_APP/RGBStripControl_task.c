#include "RGBStripControl_task.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_RGBLight.h"

//APP Include File
#include "AutoRecharge_task.h"
#include "robot_select_init.h"
#include "sensor_ranger.h"

//Light strip control priority from high to low
enum{
	RGB_Charging = 0,                //Charging
	RGB_FindChargingStations = 1,    //Find charging stations
	RGB_LowPower = 2,                //Low battery level
	RGB_RangerWarning = 3,           //Ultrasonic detection of obstacle warnings
	RGB_UserDefined = 4,             //User-defined
	RGB_Default = 5                  //Default state
};

//User-defined RGB status
uint8_t userdefine_rgb[4]={ 0 };

//Charging indicator light marker
static uint8_t charging_color=0;

static uint8_t RGB_PriorityManagement(void)
{
	//Automatically recharge relevant flag slots
	static uint32_t charging_green=0;
	static uint32_t charging_red=0;
	
	//Automatic recharge determination
	if( RobotControlParam.ChargeMode == 1 )
	{
		//Charging two modes switch using the marker
		if( ChargeDev.ChargingFlag==0 )
		{
			charging_green=0;
			charging_red=0;
			charging_color=0;//When not charging, clear the relevant marker positions
		}
		else if( ChargeDev.ChargingFlag==1 && ChargeDev.ChargingCur < 0.65f )
		{
			charging_green++;
			charging_red=0;
			if( charging_green > 5*5 ) charging_color = 1;//Green light always on
		}
		else if( ChargeDev.ChargingFlag==1 && ChargeDev.ChargingCur >= 0.65f )
		{
			charging_green=0;
			charging_red++;
			if( charging_red > 5*2 ) charging_color = 2;//Always on the red light
		}
		
		//Simulation debugging
//		if( g_sys_debug.chargingfull==1 )
//		{
//			ChargeDev.ChargingFlag=1;
//			charging_green++;
//			charging_red=0;
//			if( charging_green > 5*5 ) charging_color = 1; The green light is always on
//		}
//		else if( g_sys_debug.chargingON==1 )
//		{
//			ChargeDev.ChargingFlag=1;
//			charging_green=0;
//			charging_red++;
//			if( charging_red > 5*2 ) charging_color = 2; The red light is always on
//		}
		
		//Return charging or docking-station search state
		if( ChargeDev.ChargingFlag==1 ) return RGB_Charging;
		else return RGB_FindChargingStations;
	}
	
	//Low battery judgment
	if( RobotControlParam.LowPowerFlag ) return RGB_LowPower;
	
	//Ultrasonic warning judgment
	if( RangerHAL_A.dis < 0.5f || RangerHAL_B.dis < 0.5f || RangerHAL_C.dis < 0.5f ||
		RangerHAL_D.dis < 0.5f || RangerHAL_E.dis < 0.5f || RangerHAL_F.dis < 0.5f )
	{
		return RGB_RangerWarning;
	}
	
	//User-defined
	if( userdefine_rgb[0]==1 ) 
	{
		return RGB_UserDefined;
	}
	
	//Default state
	return RGB_Default;

}

void RGBControl_task(void* param)
{
	const uint16_t TaskFreq = 5;
	
	//Obtain the timebase, which can run at a fixed frequency for auxiliary tasks
	TickType_t preTime = xTaskGetTickCount();
	
	//RGB light strip object
	pRGBLightInterface_t rgb = &UserRGBLight;
	
	//Real-time state machine
	uint8_t RGB_Sm = RGB_Default;
	
	//Initialization and self-test of light strips
	rgb->init();
	rgb->SetColor(255,0,0);
	vTaskDelay(1000);
	rgb->SetColor(0,255,0);
	vTaskDelay(1000);
	rgb->SetColor(0,0,255);
	vTaskDelay(1000);
	
	while(1)
	{
		//Get the RGB status of the status lights
		RGB_Sm = RGB_PriorityManagement();
		
		switch( RGB_Sm )
		{
			case RGB_Charging:
				//When about to fully charge, green is available; when charging, red is present
				if( charging_color==2 )
				{
					rgb->SetColorFade(32,0,0);
				}
				else if( charging_color==1 )
				{
					rgb->SetColorFade(0,32,0);
				}
				break;
			case RGB_FindChargingStations:
				//The purple lamp stays on
				rgb->SetColorFade(100,0,128);
				break;
			case RGB_LowPower:
				//Purple light flickers low
				rgb->SetBlink(100,0,128,2000);
				break;
			case RGB_RangerWarning:
				//Yellow warning light
				rgb->SetColorFade(255,128,0);
				break;
			case RGB_UserDefined:
				//User-defined
				rgb->SetColorFade(userdefine_rgb[1],userdefine_rgb[2],userdefine_rgb[3]);
				break;
			case RGB_Default:
				//Set the default state of the light strip
				rgb->turnoff();
				break;
			default:
				break;
		}	
		
		/* Delay the specified frequency */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

