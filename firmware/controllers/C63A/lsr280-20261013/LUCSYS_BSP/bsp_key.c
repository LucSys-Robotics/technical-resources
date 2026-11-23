#include "bsp_key.h"
#include "gpio.h"

static uint8_t keyValue(void)
{
	if( get_HardWareVersion() == HW_1_1 )
		return HAL_GPIO_ReadPin(UserKey_GPIO_Port,UserKey_Pin);
	else
		return HAL_GPIO_ReadPin(UserKey_V1_0_Port,UserKey_V1_0_Pin);
}

static UserKeyState_t key_scan(uint16_t freq)
{
    static uint16_t time_core;//Walking Core
    static uint16_t long_press_time;//Long-press recognition
    static uint8_t press_flag=0;//Button press mark
    static uint8_t check_once=0;//Whether one marker has been identified
	
    float Count_time = (((float)(1.0f/(float)freq))*1000.0f);//Calculate how many milliseconds are needed to calculate 1

    if(check_once)//Once recognition is complete, clear all variables
    {
        press_flag=0;//Completed one recognition, marking zero
        time_core=0;//Completed one recognition, time reset to zero
        long_press_time=0;//Completed one recognition, time reset to zero
    }
    if(check_once&&1 == keyValue()) check_once=0; //After the scan is complete, the button is popped up, and the next scan begins

    if(0==keyValue()&&check_once==0)//Button scanning
    {
        press_flag=1;//The marker is pressed once
        long_press_time++;		  
    }

    if(long_press_time>(uint16_t)(500.0f/Count_time))// Long press for 1 second
    {	
        check_once=1;//tag has been identified
        return long_click; //Long press
    }

    //After the button is pressed once and released, start the kernel timer
    if(press_flag&&1==keyValue())
    {
        time_core++; 
    }		
	
    if(press_flag&&(time_core>(uint16_t)(50.0f/Count_time)&&time_core<(uint16_t)(300.0f/Count_time)))//Pressed again within 50~700ms
    {
        if(0==keyValue()) //If pressed again
        {
            check_once=1;//tag has been identified
            return double_click; //Mark as double-click Multiple errors exist in the
        }
    }
    else if(press_flag&&time_core>(uint16_t)(300.0f/Count_time))
    {
        check_once=1;//tag has been identified
        return single_click; //If not pressed after 800ms, click it
    }

    return key_stateless;
}

//Attach driver
KeyInterface_t UserKey = {
    .getKeyState = key_scan,
};

