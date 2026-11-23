#include "led_task.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_RGBLight.h"

void RGBControl_task(void *pvParameters);

/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
Function: LED flashing task
Entry parameters: None
Return value: none
**************************************************************************/
void led_task(void *pvParameters)
{
	#if defined DIFF_CAR
	//Light strip models: Use light strip tasks, do not use this task
	if( robot.type == 5 )
	{
		xTaskCreate(RGBControl_task, "rgb_task", LED_STK_SIZE*2,      NULL, LED_TASK_PRIO,      NULL);
		vTaskDelete(NULL);
	}
	#elif defined MEC_CAR
	if( robot.type == 9 )
	{
		xTaskCreate(RGBControl_task, "rgb_task", LED_STK_SIZE*2,      NULL, LED_TASK_PRIO,      NULL);
		vTaskDelete(NULL);
	}
	#endif
	
	
    while(1)
    {
        static u8 led_state=0;//LED on/off settings
		
		//Detects automatic recharge status and modifies LED light colors
		static u8 check_charger_mode;
		static u8 check_Charging;
		
		//Enter auto-docking state; LED is yellow
		if( 0 == check_charger_mode && 1 == charger.AllowRecharge ) LED_SetColor(LED_Yellow);
		
		//Exit auto-docking state; restore default LED color
		if( 1 == check_charger_mode && 0 == charger.AllowRecharge ) LED_SetColor(LED_Red);
		check_charger_mode = charger.AllowRecharge;
		
		if( 1==charger.AllowRecharge ) 
		{	//When charging succeeds in automatic recharge mode, set the LED to purple
			if( 0==check_Charging && 1 == charger.Charging ) LED_SetColor(LED_Purple);
			if( 1==check_Charging && 0 == charger.Charging ) LED_SetColor(LED_Yellow);
			check_Charging = charger.Charging; 
		}

        //LED blinking task
        led_state = !led_state;
        LED_SetState(led_state);
		
        //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function
        //LED blinking task is very simple, with low frequency accuracy requirements, using relative delay functions
        vTaskDelay(SysVal.LED_delay);
    }
}


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
	if( charger.AllowRecharge == 1 ) 
	{
		//Charging two modes switch using the marker
		if( charger.Charging==0 ) 
		{
			charging_green=0;
			charging_red=0;
			charging_color=0;//When not charging, clear the relevant marker positions
		}
		else if( charger.Charging==1 && charger.ChargingCurrent < 0.65f )
		{
			charging_green++;
			charging_red=0;
			if( charging_green > 5*5 ) charging_color = 1;//Green light always on
		}
		else if( charger.Charging==1 && charger.ChargingCurrent >= 0.65f )
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
		if( charger.Charging==1 ) return RGB_Charging; 
		else return RGB_FindChargingStations;
	}
	
	//Low battery judgment
	if( robot.LowPower ) return RGB_LowPower; 
	
	//Ultrasonic warning judgment
	if( s21c_board.rangerA < 0.5f ||s21c_board.rangerB  < 0.5f || s21c_board.rangerC  < 0.5f ||
		s21c_board.rangerD < 0.5f || s21c_board.rangerE < 0.5f || s21c_board.rangerF < 0.5f )
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

void RGBControl_task(void *pvParameters)
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




