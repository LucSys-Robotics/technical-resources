/**
 * @file    SerialControl_task.c
 * @brief Parsing serial port data tasks.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - This file mainly contains serial port data parsing tasks, with data sourced from ROS or serial port assistant. After receiving data from serial port interruption
 *   will write to the data queue. After writing to the queue, this task will be awakened and parse the data, sending serial port commands in a specified format
 *   Controllable chassis
 * - Vehicle control data format: 7B 00 00 X-axis speed high 8 bits X-axis speed low 8 bits Y-axis speed high 8 bits Y-axis speed low 8 bits
 *                    Z-axis speed is 8 bits high, Z-axis speed is 8 bits lower, BCC calibration position is 7D
 * - Set automatic recharge mode: 7B 01 00 X-axis speed 8 bits high, X-axis speed 8 bits lower, Y-axis speed 8 bits high, Y-axis speed 8 bits lower
 *                    Z-axis speed is 8 bits high, Z-axis speed is 8 bits lower, BCC calibration position is 7D
 * - Set the strip color: 7B 04 01 R channel value G channel value B channel value 00 00 00 BCC check bit 7D
 *              
 * - Turn off the light strip: 7B 04 00 00 00 00 00 00 00 7F 7D
 *        
     The unit of velocity is mm/s
 * - Data example 7B 00 00 01 F4 00 00 00 00 8E 7D; after the UART receives these 11 bytes, the vehicle will...
 *   Advance at 500mm/s on the X-axis.
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

//BCC check function
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
	
	uint8_t recv = 0;     //Used to receive queue data
	
	uint8_t roscmdBuf[20];
	uint8_t roscmdCount = 0;
	const uint8_t cmdLen = 11; //Length of one frame of communication data
	

	while( 1 )
	{
		if( pdPASS == xQueueReceive(g_xQueueROSserial,&recv,portMAX_DELAY) ) 
		{
			//Supports resetting into BootLoader
			_System_Reset_FromAPP_RTOS(recv);
			
			//Supports setting debug levels
			RobotControl_SetDebugLevel(recv);
			
			roscmdBuf[roscmdCount] = recv;
			if( recv == 0x7B || roscmdCount>0 ) roscmdCount++;
			else roscmdCount = 0;
			
			if( cmdLen==roscmdCount )
			{
				roscmdCount = 0;
				
				//Check frame tails and check bits
				if( roscmdBuf[cmdLen-1]==0x7D && roscmdBuf[cmdLen-2]==Calculate_BCC(roscmdBuf,cmdLen-2) )
				{	
					RobotControlCMDType_t cmd = {
						.cmdsource = ROS_CMD,//Set the command source
						0,0,0
					};

					//Parse velocity command data
					if( roscmdBuf[1]<3 )
					{
						cmd.Vx = (float)((short)(roscmdBuf[3]<<8 | roscmdBuf[4]))/1000.0f;
						cmd.Vy = (float)((short)(roscmdBuf[5]<<8 | roscmdBuf[6]))/1000.0f;
						cmd.Vz = (float)((short)(roscmdBuf[7]<<8 | roscmdBuf[8]))/1000.0f;
					}
					
					//V1 protocol
					if( roscmdBuf[1]==0 ) 
					{	//Normal speed command
						RobotControlParam.ChargeMode=0;
						RobotControlParam.SecurityLevel = roscmdBuf[2];
					}
					else if( roscmdBuf[1]==1||roscmdBuf[1]==2 )
					{	//The host computer requests to enable automatic recharge
						RobotControlParam.ChargeMode=1;
					}
					else if( roscmdBuf[1]==4 )
					{	//Strip setting instructions
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
					
					//Determines whether to perform queue writes; does not write when idle (speed set to zero)
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

