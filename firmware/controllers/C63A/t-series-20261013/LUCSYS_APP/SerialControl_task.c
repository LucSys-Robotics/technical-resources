/**
 * @file    SerialControl_task.c
 * @brief   解析串口数据任务.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - 此文件主要包含串口数据解析任务,数据来源于ROS或者串口助手。串口中断收到数据后
 *   将会写入数据队列,写入队列后此任务将会被唤醒并解析数据,按一定的格式发送串口指令
 *   可控制小车
 * - 控制小车数据格式：7B 00 00 X轴速度高8位 X轴速度低8位 Y轴速度高8位 Y轴速度低8位
 *                    Z轴速度高8位 Z轴速度低8位 BCC校验位 7D         
 * - 设置自动回充模式：7B 01 00 X轴速度高8位 X轴速度低8位 Y轴速度高8位 Y轴速度低8位
 *                    Z轴速度高8位 Z轴速度低8位 BCC校验位 7D    
 * - 设置灯带颜色：    7B 04 01 R通道值 G通道值 B通道值  00 00 00 BCC校验位 7D
 *              
 * - 关闭灯带：        7B 04 00 00 00 00 00 00 00 7F 7D
 *        
     其中速度的单位是 mm/s
 * - 数据示例 7B 00 00 01 F4 00 00 00 00 8E 7D,串口在收到这11个字节后,小车将
 *   以 X轴为500mm/s 速度前进.
 * @note
 * 
 * 
 */

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_RGBLight.h"

//APP Include File
#include "robot_select_init.h"
#include "RobotControl_task.h"
#include "RGBStripControl_task.h"

//BCC校验函数
uint8_t Calculate_BCC(const uint8_t* checkdata,uint16_t datalen)
{
	char bccval = 0;
	for(uint16_t i=0;i<datalen;i++)
	{
		bccval ^= checkdata[i];
	}
	return bccval;
}

void SerialControlTask(void* param)
{
	extern QueueHandle_t g_xQueueROSserial;
	
	uint8_t recv = 0;     //用于接收队列数据
	
	uint8_t roscmdBuf[20];
	uint8_t roscmdCount = 0;
	const uint8_t cmdLen = 11; //一帧通信数据长度
	

	while( 1 )
	{
		if( pdPASS == xQueueReceive(g_xQueueROSserial,&recv,portMAX_DELAY) ) 
		{
			//支持复位进入BootLoader
			_System_Reset_FromAPP_RTOS(recv);
			
			//支持设置调试等级
			RobotControl_SetDebugLevel(recv);
			
			roscmdBuf[roscmdCount] = recv;
			if( recv == 0x7B || roscmdCount>0 ) roscmdCount++;
			else roscmdCount = 0;
			
			if( cmdLen==roscmdCount )
			{
				roscmdCount = 0;
				
				//检查帧尾与校验位
				if( roscmdBuf[cmdLen-1]==0x7D && roscmdBuf[cmdLen-2]==Calculate_BCC(roscmdBuf,cmdLen-2) )
				{	
					RobotControlCMDType_t cmd = {
						.cmdsource = ROS_CMD,//设置指令来源
						0,0,0
					};

					//速度命令数据解析
					if( roscmdBuf[1]<3 )
					{
						cmd.Vx = (float)((short)(roscmdBuf[3]<<8 | roscmdBuf[4]))/1000.0f;
						cmd.Vy = (float)((short)(roscmdBuf[5]<<8 | roscmdBuf[6]))/1000.0f;
						cmd.Vz = (float)((short)(roscmdBuf[7]<<8 | roscmdBuf[8]))/1000.0f;
					}
					
					//V1协议
					if( roscmdBuf[1]==0 ) 
					{	//正常速度指令
						RobotControlParam.ChargeMode=0;
						RobotControlParam.SecurityLevel = roscmdBuf[2];
					}
					else if( roscmdBuf[1]==1||roscmdBuf[1]==2 )
					{	//上位机请求开启自动回充
						RobotControlParam.ChargeMode=1;
					}
					else if( roscmdBuf[1]==4 )
					{	//灯带设置指令
						if( roscmdBuf[2]==1 )
						{
							userdefine_rgb[0]=1;
							userdefine_rgb[1]=roscmdBuf[3];userdefine_rgb[2]=roscmdBuf[4];userdefine_rgb[3]=roscmdBuf[5];
						}
						else 
						{
							userdefine_rgb[0]=0;
						}
					}
					
					//判断是否要执行队列写入,空闲时不写入(速度全0)
					uint8_t writeflag=1;
					static uint8_t idleCount = 0;
					if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
					else idleCount=0;
					if( idleCount>10 ) writeflag=0,idleCount=10;

					if( writeflag)  WriteRobotControlQueue(&cmd,0);

				}
			}
		}
	}
}

