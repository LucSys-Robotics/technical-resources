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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//Enable the GPIOB clock
  GPIO_InitStructure.GPIO_Pin = KEY_PIN; //Key corresponding pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//Normal input mode
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOC, &GPIO_InitStructure);//Initialize GPIOB14
} 
/**************************************************************************
Function: Buttons to scan
Input   : none
Output  : none
Function: Scans with key press
Entry parameters: None
Return value: button state: 0-no action; 1-single click
**************************************************************************/
u8 click(void)
{
	//Press the release sign
	//Button release mark
	static u8 flag_key=1;
	
	if(flag_key&&KEY==0)
	{
	 flag_key=0; //The key is pressed // Key pressed
	 return 1;	
	}
	else if(1==KEY)			
		flag_key=1;
	return 0; //No key is pressed // No key pressed
}
/**************************************************************************
Function: Delay function
Input   : none
Output  : none
Function: Delay function
Entry parameters: None
Return value: none
**************************************************************************/
void Delay_ms(void)
{
   int ii,i;    
   for(ii=0;ii<50;ii++)
   {
	   for(i=0;i<50;i++);
	 }	
}
/**************************************************************************
Function: Buttons to scan
Input   : Double click wait time
Output  : Button status: 0- no action, 1- click, 2- double click
Function: Scans with key press
Entry parameter: double-click the wait time
Return value: button state: 0-no action, 1-single click, 2-double click
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;   
    else        Forever_count=0;
	
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click // Double click
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click // Click
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Button scan.Because static variables are used, a function with a different name needs to be defined when the keystroke scan function is used multiple times
Input   : none
Output  : Button status: 0- no action, 1- click, 2- double click
Function: key scan. Because static variables are used, when multiple key scan functions are needed, you need to define a function with a different name
Entry parameters: None
Return value: button state: 0-no action, 1-single click, 2-double click
**************************************************************************/
u8 click_N_Double_MPU6050 (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;  
    else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click // Double click
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click // Click
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Long according to the test
Input   : none
Output  : Key state 0: no action 1: long press 3s
Function: Long press to detect
Entry parameters: None
Return value: button state: 0-no action; 1-long press for 3s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count,Long_Press;

	if(Long_Press==0&&KEY==0)  Long_Press_count++; 
	else                       Long_Press_count=0;

	if(Long_Press_count>15)	//3 seconds // 3 seconds
	{
		Long_Press=1;	
		Long_Press_count=0;
		return 1;
	}				
	 if(Long_Press==1) //Set long-press flag to 1
	{
			Long_Press=0;
	}
	return 0;
}

//Button scan function
//Entry parameters: the task frequency for executing the function, delay filtering time
//Return value: long_click, double_click, single_click, key_stateless
//Typical value: ¢Ù When the function only scans keys, KEY_Scan(3000,200)
//       (2) When there are many other operations in the function, KEY_Scan(200,0)
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

    if(long_press_time>(u16)(600.0f/Count_time))// Long press for 1 second
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
	
    if(press_flag&&(time_core>(u16)(50.0f/Count_time)&&time_core<(u16)(500.0f/Count_time)))//Pressed again within 50~700ms
    {
        if(KEY==0) //If pressed again
        {
            check_once=1;//tag has been identified
            return double_click; //Mark as double-click Multiple errors exist in the
        }
    }
    else if(press_flag&&time_core>(u16)(500.0f/Count_time))
    {
        check_once=1;//tag has been identified
        return single_click; //If not pressed after 800ms, click it
    }

    return key_stateless;
}

