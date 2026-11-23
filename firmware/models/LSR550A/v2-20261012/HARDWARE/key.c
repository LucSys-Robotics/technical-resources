#include "key.h"

/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
Function: Initialize by key
Entry parameters: None
Return value: none
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ENABLE_USER_KEY_PIN_CLOCK;
	GPIO_InitStructure.GPIO_Pin = USER_KEY_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
	GPIO_Init(USER_KEY_PORT, &GPIO_InitStructure);
} 


/**************************************************************************
Function: key scan function
Entry parameters: frequency of tasks executed for this function, delay filtering time
Return value: long_click, double_click, single_click, key_stateless (long press, double click, single click, no state)
Author: LUCSYS
**************************************************************************/
u8 KEY_Scan(u16 Frequency,u16 filter_times)
{
    static u16 time_core;//Walking Core
    static u16 long_press_time;//Long-press recognition
    static u8 press_flag=0;//Button press mark
    static u8 check_once=0;//Whether one marker has been identified
    static u16 delay_mini_1;
    static u16 delay_mini_2;
	
    float Count_time = (((float)(1.0f/(float)Frequency))*1000.0f);//Calculate how many milliseconds are needed to calculate 1

    if(check_once)//Once recognition is complete, clear all variables
    {
        press_flag=0;//Completed one recognition, marking zero
        time_core=0;//Completed one recognition, time reset to zero
        long_press_time=0;//Completed one recognition, time reset to zero
        delay_mini_1=0;
        delay_mini_2=0;
    }
    if(check_once&&KEY==1) check_once=0; //After the scan is complete, the button is popped up, and the next scan begins

    if(KEY==0&&check_once==0)//Button scanning
    {
        press_flag=1;//The marker is pressed once
		
        if(++delay_mini_1>filter_times)
        {
            delay_mini_1=0;
            long_press_time++;		
        }
    }

    if(long_press_time>(u16)(500.0f/Count_time))// Long press for 1 second
    {	
        check_once=1;//tag has been identified
        return long_click; //Long press
    }

    //After the button is pressed once and released, start the kernel timer
    if(press_flag&&KEY==1)
    {
        if(++delay_mini_2>filter_times)
        {
            delay_mini_2=0;
            time_core++; 
        }
    }		
	
    if(press_flag&&(time_core>(u16)(50.0f/Count_time)&&time_core<(u16)(300.0f/Count_time)))//Pressed again within 50~700ms
    {
        if(KEY==0) //If pressed again
        {
            check_once=1;//tag has been identified
            return double_click; //Mark as double-click Multiple errors exist in the
        }
    }
    else if(press_flag&&time_core>(u16)(300.0f/Count_time))
    {
        check_once=1;//tag has been identified
        return single_click; //If not pressed after 800ms, click it
    }

    return key_stateless;
}

