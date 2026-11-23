#include "bsp_icm20948.h"

#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "robot_select_init.h"


void ImuTask(void* param)
{
	//Obtain the time base, used to assist tasks that specify a fixed frequency to run
	TickType_t preTime = xTaskGetTickCount();
	
	//Frequency of control tasks, unit HZ
	const uint16_t TaskFreq = 100;
	
	pIMUInterface_t imu = &UserICM20948;
	
	//Initialize gyroscope
	while(imu->Init())
	{
		vTaskDelay(1000);
	}
	
	//Startup timer
	uint32_t SystemStartCnt = 0;
	
	//Disable the chassis for the first 10 seconds after startup while waiting for IMU zero calibration
	RobotControlParam.softwareEnflag = 1;
	
	while(1)
	{
		imu->Update_9axisVal(&axis_9Val,&axis_9ValOri);//Update gyroscope data; all readings are raw. Software IIC takes 1.0 ms
		imu->UpdateAttitude(axis_9Val,&AttitudeVal);//Updated attitude angles
		
		//Timed for the first 30 seconds
		if( SystemStartCnt < TaskFreq*30 ) SystemStartCnt++;
		
		//After 10 seconds, the IMU zeros to enable the car
		if( SystemStartCnt == TaskFreq*10 )
		{
			//Zeroing is performed when the car is relatively stationary
			if( fabs(axis_9Val.gyro.x)<0.05f && fabs(axis_9Val.gyro.y)<0.05f && fabs(axis_9Val.gyro.z)<0.05f )
			{
				RobotControlParam.LedTickState++;
				imu->UpdateZeroPoint_axis();
			}			
			
			//Zero marking is performed under normal conditions when the vehicle's attitude angle is relatively level
			if( fabs(AttitudeVal.pitch) < 5.0f && fabs(AttitudeVal.roll) < 5.0f )
			{
				RobotControlParam.LedTickState++;
				imu->UpdateZeroPoint_attitude();
			}

			//Check whether all zeros are successful
			if( RobotControlParam.LedTickState!=2 )
			{
				RobotControlParam.LedTickState=0;
				SystemStartCnt=TaskFreq*8;//Conditions not met, wait again
			}
			else
			{
				//Marking zero success enables the chassis
				RobotControlParam.softwareEnflag=0;
			}				
		}
		
		/* Delay the specified frequency */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

