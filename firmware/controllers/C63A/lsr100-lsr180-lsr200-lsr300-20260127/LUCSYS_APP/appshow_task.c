
//HAL Lib Include File
#include "usart.h"

//C Include File
#include <stdio.h>
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_ServoDrive.h"
#include "bsp_icm20948.h"

//APP Include File
#include "BlueTooth_task.h"
#include "robot_select_init.h"
#include "RobotControl_task.h"

static char page1_paramshow[100];   //APP homepage data
static char page2_paramshow[512];  //APP waveform page data
static char page3_paramshow[512];  //APP parameter page data

static int voltage_to_percentage(float voltage) {
    const float V_MIN = 20.3f;  // Minimum voltage (0%)
    const float V_MAX = 23.45f; // Maximum voltage (100%)
    
    // Ensure the voltage is within the effective range
    if (voltage < V_MIN) return 0;
    if (voltage > V_MAX) return 100;
    
    // Linear interpolation calculates percentages and rounds
    int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // Rounded up
	
	if( percentage > 100 ) percentage = 100;
	if( percentage <= 0 ) percentage = 0;
	
    return percentage;
}

void APPshow_task(void* param)
{
	//Obtain the timebase, which can run at a fixed frequency for auxiliary tasks
	TickType_t preTime = xTaskGetTickCount();
	
	//The control frequency for this task, in Hz
	const uint16_t TaskFreq = 20;
	
	//Implement time-sharing display of app data
	uint8_t appshowNum = 0;
	char* NeedShowBuf = NULL;
	
	while(1)
	{	
		int showLeftWheel=0,showRightWheel=0,VolPercen=0;
		
		switch(RobotHardWareParam.CarType)
		{
			case LSR300:case LSR300Mini:case LSR100:
				showLeftWheel = RobotControlParam.MotorA.feedback*1000;
				showRightWheel = RobotControlParam.MotorB.feedback*1000;
				break;
			case LSR200:case LSR200_OUTDOOR:
				showLeftWheel = RobotControlParam.MotorA.feedback*1000;
				showRightWheel = RobotControlParam.MotorD.feedback*1000;
				break;
			case REF2:
				showLeftWheel = RobotControlParam.MotorA.feedback*1000;
				showRightWheel = RobotControlParam.MotorF.feedback*1000;
				break;
		}
		
		//Battery voltage percentage
		VolPercen = voltage_to_percentage(RobotControlParam.Vol);
		
		//APP displays data
		appshowNum++; //Time-sharing display
		if( 1 == appshowNum )
		{
			sprintf(page1_paramshow,"{A%d:%d:%d:%d}$",showLeftWheel,showRightWheel,VolPercen,(int)AttitudeVal.yaw); //Left dial, right dial, battery percentage, angle
			NeedShowBuf = page1_paramshow;
		}
		else if( 2 == appshowNum )
		{
			appshowNum = 0;
			sprintf(page2_paramshow,"{B%d:%d:%d}$",(int)AttitudeVal.pitch,(int)AttitudeVal.roll,(int)AttitudeVal.yaw); //Waveform display
			NeedShowBuf = page2_paramshow;
		}
		
		//APP retrieves data requests
		if( lucsysApp.reportparam == 1 )
		{
			lucsysApp.reportparam = 0;
			sprintf(page3_paramshow,"{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
			 (int)RobotControlParam.defalutSpeed,
			 (int)RobotControlParam.LineDiffParam,
			 (int)DrivePid.SpeedKp,(int)DrivePid.SpeedKi,
			 (int)DrivePid.CurKp,(int)DrivePid.CurKi,
			  0,0,0); 
			NeedShowBuf = page3_paramshow;
		}
		
		//Regular data sending
		if( RobotControlParam.DebugLevel == 0 )
		{
			HAL_UART_Transmit_DMA(&huart2,(uint8_t*)NeedShowBuf,strlen(NeedShowBuf));
		}
		
		/* Delay the specified frequency */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
	
}

