#include "led_task.h"

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


