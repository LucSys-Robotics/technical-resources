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
	
	//Disable the chassis for the first 10 seconds after startup while waiting for IMU zero calibration
	RobotControlParam.softwareEnflag = 1;
	
	//Initialize gyroscope
	while(imu->Init())
	{
		vTaskDelay(1000);
	}
	
	//Startup timer
	uint32_t SystemStartCnt = 0;
	
	static uint8_t ZeroMarkState = 0;
	static uint32_t MarkCnt = 0;
	
	while(1)
	{
		imu->Update_9axisVal(&axis_9Val,&axis_9ValOri);//Update gyroscope data; all readings are raw. Software IIC takes 1.0 ms
		imu->UpdateAttitude(axis_9Val,&AttitudeVal);//Updated attitude angles
		
		//Timed for the first 30 seconds
		if( SystemStartCnt < TaskFreq*30 ) SystemStartCnt++;
		
		//After 7 seconds, perform IMU zeroing to enable the car
		if( ZeroMarkState!=2 && SystemStartCnt >= TaskFreq*7 )
		{
			switch( ZeroMarkState )
			{
				case 0:
					//Zeroing is performed only on the plane
					if( fabs(AttitudeVal.pitch) < 5.0f && fabs(AttitudeVal.roll) < 5.0f )
					{
						if( fabs(axis_9Val.gyro.x)<0.05f && fabs(axis_9Val.gyro.y)<0.05f && fabs(axis_9Val.gyro.z)<0.05f )
						{
							MarkCnt++;
							if( MarkCnt > TaskFreq )
							{
								MarkCnt=0;
								imu->UpdateZeroPoint_axis();
								ZeroMarkState=1;
								RobotControlParam.LedTickState=1;
							}
						}
						else MarkCnt=0;
					}
					break;
				case 1:
				{
					//After the original value calibration is completed, wait for the angle to stabilize before zeroing the angle
					static uint8_t lowFre = 0;
					static float lastpitch = 0,lastroll = 0;

					//One 500ms scan
					lowFre++;
					if( lowFre>= TaskFreq/2)
					{
						lowFre=0;
						if( fabs(AttitudeVal.pitch-lastpitch) < 0.1f && fabs(AttitudeVal.roll-lastroll) < 0.1f  )
						{
							MarkCnt++;
							if( MarkCnt > 2 )
							{
								MarkCnt=0;
								imu->UpdateZeroPoint_attitude();
								ZeroMarkState=2;
								RobotControlParam.LedTickState=2;
								RobotControlParam.softwareEnflag = 0;
							}
						}
						lastpitch = AttitudeVal.pitch;
						lastroll = AttitudeVal.roll;
					}
					break;
				}		
				default:
					break;
			}			
		}
		
		/* Delay the specified frequency */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

