#include "can.h"

//C Include File
#include <stdio.h>
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "queue.h"

//BSP Include File
#include "bsp_can.h"
#include "bsp_ServoDrive.h"

//APP Include File
#include "RobotControl_task.h"

extern QueueHandle_t g_xQueueCANopenCallback;
extern QueueHandle_t g_xQueueAutoRecharge;
extern QueueHandle_t g_xQueueHeartBeatMsg;

static void WheelSpeedCalculation(uint8_t NodeId,float Lrpm,float Rrpm);

// CAN FIFO0 interrupts, associated with CAN1
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t recvbuffer[8]={0};			    //Receive cache array
	HAL_StatusTypeDef HAL_RetVal;		    //Enumeration of state judgments
	CAN_RxHeaderTypeDef RxMsg;              //Receive structure
	HAL_RetVal=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMsg,recvbuffer);//Receive data from the mailbox
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//CAN terminal received successfully
	if (HAL_OK==HAL_RetVal)
	{
		if( RxMsg.IDE == CAN_ID_STD )
		{
			//Supports 0x181 location control using CAN
			if( RxMsg.StdId==0x181 )
			{
				RobotControlCMDType_t cmd = {
					.cmdsource = CAN_CMD,
					0,0,0
				};
				
				cmd.Vx = (float)((short)(recvbuffer[0]<<8|recvbuffer[1]))/1000.0f;
				cmd.Vy = (float)((short)(recvbuffer[2]<<8|recvbuffer[3]))/1000.0f;
				cmd.Vz = (float)((short)(recvbuffer[4]<<8|recvbuffer[5]))/1000.0f;
				
				//Idle judgment
				uint8_t writeflag=1;
				static uint8_t idleCount = 0;
				if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
				else idleCount=0;
				if( idleCount>10 ) writeflag=0,idleCount=10;
				
				//Write to queue when not idle
				if( writeflag ) WriteRobotControlQueue(&cmd,&xHigherPriorityTaskWoken);	
			}
			
			//Automatically recharge data write queue
			else if( RxMsg.StdId==0x182 )
			{
				CANmsgType_t data={0};
				data.id = RxMsg.StdId;
				memcpy(data.buffer,recvbuffer,8);
				if( g_xQueueAutoRecharge!=NULL )
					xQueueOverwriteFromISR(g_xQueueAutoRecharge,&data,&xHigherPriorityTaskWoken);
			}

		}
		
		//Extend frame ID data reception
		else if(RxMsg.IDE == CAN_ID_EXT)
		{
//			printf("CAN1 ExtId:%X\r\n",RxMsg.ExtId);
//			for(uint8_t i=0;i<RxMsg.DLC;i++)
//				printf("%2X\t",recvbuffer[i]);
//			printf("\r\n\r\n");
		}
	}
	
	//Initiate scheduling based on specific circumstances
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



// CAN FIFO1 interrupts, associated with CAN2
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t recvbuffer[8]={0};			    //Receive cache array
	HAL_StatusTypeDef HAL_RetVal;		    //Enumeration of state judgments
	CAN_RxHeaderTypeDef RxMsg;              //Receive structure
	HAL_RetVal=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&RxMsg,recvbuffer);//Receive data from the mailbox
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//CAN terminal received successfully
	if (HAL_OK==HAL_RetVal)
	{
		//Standard frame format
		if( RxMsg.IDE == CAN_ID_STD )
		{
			//CANopen-TSDO client response content and write it to the queue
			if( RxMsg.StdId >= 0x581 && RxMsg.StdId<=0x5FF )
			{
				CANmsgType_t data={0};
				data.id = RxMsg.StdId;
				memcpy(data.buffer,recvbuffer,8);
				
				if(g_xQueueCANopenCallback!=NULL)
					xQueueOverwriteFromISR(g_xQueueCANopenCallback,&data,&xHigherPriorityTaskWoken);
			}
			
			//CANopen heartbeat packet
			else if( RxMsg.StdId >= 0x701 && RxMsg.StdId<=0x77F )
			{	
				ServoDriver_OnLineCheck(RxMsg.StdId&0x7F);
			}
			
			else if( RxMsg.StdId == 0x190 ) //Speed feedback calculation for drive 1
			{
				int tmpL = (int)recvbuffer[3]<<24|recvbuffer[2]<<16|recvbuffer[1]<<8|recvbuffer[0];   //Unit: 0.1 rpm
				int tmpR = -((int)recvbuffer[7]<<24|recvbuffer[6]<<16|recvbuffer[5]<<8|recvbuffer[4]);//Unit: 0.1 rpm
				WheelSpeedCalculation(1,(float)tmpL*0.1f,(float)tmpR*0.1f);
			}
			else if( RxMsg.StdId == 0x191 ) //Speed feedback calculation for drive No. 2
			{
				int tmpL = (int)recvbuffer[3]<<24|recvbuffer[2]<<16|recvbuffer[1]<<8|recvbuffer[0];
				int tmpR = -((int)recvbuffer[7]<<24|recvbuffer[6]<<16|recvbuffer[5]<<8|recvbuffer[4]);
				WheelSpeedCalculation(2,(float)tmpL*0.1f,(float)tmpR*0.1f);
			}
			else if( RxMsg.StdId == 0x192 ) //Speed feedback calculation for drive 3
			{
				int tmpL = (int)recvbuffer[3]<<24|recvbuffer[2]<<16|recvbuffer[1]<<8|recvbuffer[0];
				int tmpR = -((int)recvbuffer[7]<<24|recvbuffer[6]<<16|recvbuffer[5]<<8|recvbuffer[4]);
				WheelSpeedCalculation(3,(float)tmpL*0.1f,(float)tmpR*0.1f);
			}
			
			//Other messages
			else 
			{
//				printf("CAN2 StdId:%X\r\n",RxMsg.StdId);
//				for(uint8_t i=0;i<RxMsg.DLC;i++)
//					printf("%2X\t",recvbuffer[i]);
//				printf("\r\n\r\n");
			}

		}

	}
	
	//Initiate scheduling based on specific circumstances
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
		
//Wheel speed feedback calculation for different vehicle models
static void WheelSpeedCalculation(uint8_t NodeId,float Lrpm,float Rrpm)
{
	//Calculate the correction factor
	float LeftWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,0);
	
	//Converts rpm to the linear speed of the wheel
	float Lvel = rpm_to_linearVel(Lrpm,RobotHardWareParam.WheelPerimeter)/LeftWheelDiff;
	float Rvel = rpm_to_linearVel(Rrpm,RobotHardWareParam.WheelPerimeter)/RightWheelDiff;
	
	switch(RobotHardWareParam.CarType)
	{
		//Single-drive models
		case S300:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedback = Lvel;
				RobotControlParam.MotorB.feedback = Rvel;
			}
			break;
		//Single-drive models
		case S300Mini:case S100:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedback = -Lvel;
				RobotControlParam.MotorB.feedback = -Rvel;
			}
			break;
		//Dual drive
		case S200:case S200_OUTDOOR:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedback = Lvel;
				RobotControlParam.MotorD.feedback = Rvel;
			}
			else if( NodeId==2 )
			{
				RobotControlParam.MotorB.feedback = Lvel;
				RobotControlParam.MotorC.feedback = Rvel;
			}
			break;
		//3 drivers
		case S260:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedback = Lvel;
				RobotControlParam.MotorF.feedback = Rvel;
			}
			else if( NodeId==2 )
			{
				RobotControlParam.MotorB.feedback = Lvel;
				RobotControlParam.MotorE.feedback = Rvel;
			}
			else if( NodeId==3 )
			{
				RobotControlParam.MotorC.feedback = Lvel;
				RobotControlParam.MotorD.feedback = Rvel;
			}
			break;
	}
}
