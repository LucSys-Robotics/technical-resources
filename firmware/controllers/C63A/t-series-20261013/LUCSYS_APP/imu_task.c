#include "bsp_icm20948.h"

#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "robot_select_init.h"


void ImuTask(void* param)
{
	//获取时基,用于辅助任务能指定固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//控制任务的频率,单位HZ
	const uint16_t TaskFreq = 100;
	
	pIMUInterface_t imu = &UserICM20948;
	
	//开机前10秒失能小车,等待imu标零
	RobotControlParam.softwareEnflag = 1;
	
	//陀螺仪初始化
	while(imu->Init())
	{
		vTaskDelay(1000);
	}
	
	//开机计时
	uint32_t SystemStartCnt = 0;
	
	static uint8_t ZeroMarkState = 0;
	static uint32_t MarkCnt = 0;
	
	while(1)
	{
		imu->Update_9axisVal(&axis_9Val,&axis_9ValOri);//陀螺仪数据更新,获取的数据均为原始数据.软件iic耗时1.0 ms
		imu->UpdateAttitude(axis_9Val,&AttitudeVal);//更新姿态角
		
		//前30秒计时
		if( SystemStartCnt < TaskFreq*30 ) SystemStartCnt++;
		
		//7秒后执行imu标零,使能小车
		if( ZeroMarkState!=2 && SystemStartCnt >= TaskFreq*7 )
		{
			switch( ZeroMarkState )
			{
				case 0:
					//仅在平面上才执行标零
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
					//原始值标定完毕后,等待角度稳定再执行角度标零
					static uint8_t lowFre = 0;
					static float lastpitch = 0,lastroll = 0;

					//500ms扫描1次
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
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

