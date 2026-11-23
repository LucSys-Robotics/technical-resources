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

#define WHEEL_CURRENT_SCALE      0.1f
#define WHEEL_TEMPERATURE_SCALE  0.1f

static void WheelSpeedCalculation(uint8_t NodeId,float Lrpm,float Rrpm);
static int16_t ReadInt16LE(const uint8_t* data);
static void WheelTelemetryCalculation(uint8_t NodeId,int16_t Lcurrent,int16_t Rcurrent,int16_t Ltemperature,int16_t Rtemperature);

// CAN FIFO0中断，关联CAN1
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t recvbuffer[8]={0};			    //接收缓存数组
	HAL_StatusTypeDef HAL_RetVal;		    //判断状态的枚举
	CAN_RxHeaderTypeDef RxMsg;              //接收结构体
	HAL_RetVal=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMsg,recvbuffer);//接收邮箱中的数据
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//CAN终端接收成功
	if (HAL_OK==HAL_RetVal)
	{
		if( RxMsg.IDE == CAN_ID_STD )
		{
			//支持0x181位置使用CAN控制
			if( RxMsg.StdId==0x181 )
			{
				RobotControlCMDType_t cmd = {
					.cmdsource = CAN_CMD,
					0,0,0
				};
				
				cmd.Vx = (float)((short)(recvbuffer[0]<<8|recvbuffer[1]))/1000.0f;
				cmd.Vy = (float)((short)(recvbuffer[2]<<8|recvbuffer[3]))/1000.0f;
				cmd.Vz = (float)((short)(recvbuffer[4]<<8|recvbuffer[5]))/1000.0f;
				
				//空闲判断
				uint8_t writeflag=1;
				static uint8_t idleCount = 0;
				if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
				else idleCount=0;
				if( idleCount>10 ) writeflag=0,idleCount=10;
				
				//非空闲写入队列
				if( writeflag ) WriteRobotControlQueue(&cmd,&xHigherPriorityTaskWoken);	
			}
			
			//自动回充的数据写入队列
			else if( RxMsg.StdId==0x182 )
			{
				CANmsgType_t data={0};
				data.id = RxMsg.StdId;
				memcpy(data.buffer,recvbuffer,8);
				if( g_xQueueAutoRecharge!=NULL )
					xQueueOverwriteFromISR(g_xQueueAutoRecharge,&data,&xHigherPriorityTaskWoken);
			}

		}
		
		//扩展帧ID数据接收
		else if(RxMsg.IDE == CAN_ID_EXT)
		{
//			printf("CAN1 ExtId:%X\r\n",RxMsg.ExtId);
//			for(uint8_t i=0;i<RxMsg.DLC;i++)
//				printf("%2X\t",recvbuffer[i]);
//			printf("\r\n\r\n");
		}
	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



// CAN FIFO1中断，关联CAN2
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t recvbuffer[8]={0};			    //接收缓存数组
	HAL_StatusTypeDef HAL_RetVal;		    //判断状态的枚举
	CAN_RxHeaderTypeDef RxMsg;              //接收结构体
	HAL_RetVal=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&RxMsg,recvbuffer);//接收邮箱中的数据
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//CAN终端接收成功
	if (HAL_OK==HAL_RetVal)
	{
		//标准帧格式
		if( RxMsg.IDE == CAN_ID_STD )
		{
			//CANopen-TSDO客户端应答内容,写入队列
			if( RxMsg.StdId >= 0x581 && RxMsg.StdId<=0x5FF )
			{
				CANmsgType_t data={0};
				data.id = RxMsg.StdId;
				memcpy(data.buffer,recvbuffer,8);
				
				if(g_xQueueCANopenCallback!=NULL)
					xQueueOverwriteFromISR(g_xQueueCANopenCallback,&data,&xHigherPriorityTaskWoken);
			}
			
			//CANopen心跳报文
			else if( RxMsg.StdId >= 0x701 && RxMsg.StdId<=0x77F )
			{	
				ServoDriver_OnLineCheck(RxMsg.StdId&0x7F);
			}
			
			else if( RxMsg.StdId == 0x190 ) //1号驱动器的转速反馈计算
			{
				int tmpL = (int)recvbuffer[3]<<24|recvbuffer[2]<<16|recvbuffer[1]<<8|recvbuffer[0];   //单位0.1rpm
				int tmpR = -((int)recvbuffer[7]<<24|recvbuffer[6]<<16|recvbuffer[5]<<8|recvbuffer[4]);//单位0.1rpm
				WheelSpeedCalculation(1,(float)tmpL,(float)tmpR);
			}
			else if( RxMsg.StdId == 0x191 ) //2号驱动器的转速反馈计算
			{
				int tmpL = (int)recvbuffer[3]<<24|recvbuffer[2]<<16|recvbuffer[1]<<8|recvbuffer[0];
				int tmpR = -((int)recvbuffer[7]<<24|recvbuffer[6]<<16|recvbuffer[5]<<8|recvbuffer[4]);
				WheelSpeedCalculation(2,(float)tmpL,(float)tmpR);
			}
			else if( RxMsg.StdId == 0x192 ) //3号驱动器的转速反馈计算
			{
				int tmpL = (int)recvbuffer[3]<<24|recvbuffer[2]<<16|recvbuffer[1]<<8|recvbuffer[0];
				int tmpR = -((int)recvbuffer[7]<<24|recvbuffer[6]<<16|recvbuffer[5]<<8|recvbuffer[4]);
				WheelSpeedCalculation(3,(float)tmpL,(float)tmpR);
			}
			else if( RxMsg.StdId == 0x193 ) //4号驱动器的转速反馈计算
			{
				int tmpL = (int)recvbuffer[3]<<24|recvbuffer[2]<<16|recvbuffer[1]<<8|recvbuffer[0];
				int tmpR = -((int)recvbuffer[7]<<24|recvbuffer[6]<<16|recvbuffer[5]<<8|recvbuffer[4]);
				WheelSpeedCalculation(4,(float)tmpL,(float)tmpR);
			}
			else if( (RxMsg.StdId == 0x292 || RxMsg.StdId == 0x293) && RxMsg.DLC >= 8 )
			{
				int16_t tmpLCurrent = ReadInt16LE(&recvbuffer[0]);
				int16_t tmpRCurrent = ReadInt16LE(&recvbuffer[2]);
				int16_t tmpLTemperature = ReadInt16LE(&recvbuffer[4]);
				int16_t tmpRTemperature = ReadInt16LE(&recvbuffer[6]);
				uint8_t nodeId = (uint8_t)(RxMsg.StdId - 0x28F);

				WheelTelemetryCalculation(nodeId,tmpLCurrent,tmpRCurrent,tmpLTemperature,tmpRTemperature);
			}
			//其他报文
			else 
			{
//				printf("CAN2 StdId:%X\r\n",RxMsg.StdId);
//				for(uint8_t i=0;i<RxMsg.DLC;i++)
//					printf("%2X\t",recvbuffer[i]);
//				printf("\r\n\r\n");
			}

		}

	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
		
//不同车型的轮速反馈计算
static void WheelSpeedCalculation(uint8_t NodeId,float Lrpm,float Rrpm)
{
	//计算纠偏系数
	float LeftWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,0);
	
	//将rpm转换为车轮的线速度
	float Lvel = rpm_to_linearVel(Lrpm*0.1f,RobotHardWareParam.WheelPerimeter)/LeftWheelDiff;
	float Rvel = rpm_to_linearVel(Rrpm*0.1f,RobotHardWareParam.WheelPerimeter)/RightWheelDiff;
	
	switch(RobotHardWareParam.CarType)
	{
		//单驱动车型
		case S300:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedbackSpeed = Lvel;
				RobotControlParam.MotorB.feedbackSpeed = Rvel;
			}
			break;
		//单驱动车型
		case S300Mini:case S100:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedbackSpeed = -Lvel;
				RobotControlParam.MotorB.feedbackSpeed = -Rvel;
			}
			break;
		//双驱动
		case S200:case S200_OUTDOOR:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedbackSpeed = Lvel;
				RobotControlParam.MotorD.feedbackSpeed = Rvel;
			}
			else if( NodeId==2 )
			{
				RobotControlParam.MotorB.feedbackSpeed = Lvel;
				RobotControlParam.MotorC.feedbackSpeed = Rvel;
			}
			break;
		//3驱动
		case S260:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.feedbackSpeed = Lvel;
				RobotControlParam.MotorF.feedbackSpeed = Rvel;
			}
			else if( NodeId==2 )
			{
				RobotControlParam.MotorB.feedbackSpeed = Lvel;
				RobotControlParam.MotorE.feedbackSpeed = Rvel;
			}
			else if( NodeId==3 )
			{
				RobotControlParam.MotorC.feedbackSpeed = Lvel;
				RobotControlParam.MotorD.feedbackSpeed = Rvel;
			}
			break;
			
		//四轮四转
		case T280:
			if( NodeId==1 )
			{
				RobotControlParam.MotorA.ori_pos = Lrpm;
				RobotControlParam.MotorB.ori_pos = -Rrpm;
			}
			else if( NodeId==2 )
			{
				RobotControlParam.MotorC.ori_pos = Lrpm;
				RobotControlParam.MotorD.ori_pos = -Rrpm;
			}
			else if( NodeId==3 )
			{
				RobotControlParam.MotorA.feedbackSpeed = -Lvel;
				RobotControlParam.MotorB.feedbackSpeed = Rvel;
			}
			else if( NodeId==4 )
			{
				RobotControlParam.MotorC.feedbackSpeed = -Lvel;
				RobotControlParam.MotorD.feedbackSpeed = Rvel;
			}
			break;
	}
}

static int16_t ReadInt16LE(const uint8_t* data)
{
	return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

// Driver3 controls wheel A/B and Driver4 controls wheel C/D on T280.
// Drive commands for all four wheel motors are inverted, so invert current
// feedback as well to keep its sign consistent with wheel feedbackSpeed.
static void WheelTelemetryCalculation(uint8_t NodeId,int16_t Lcurrent,int16_t Rcurrent,int16_t Ltemperature,int16_t Rtemperature)
{
	float leftCurrent = -(float)Lcurrent * WHEEL_CURRENT_SCALE;
	float rightCurrent = -(float)Rcurrent * WHEEL_CURRENT_SCALE;
	float leftTemperature = (float)Ltemperature * WHEEL_TEMPERATURE_SCALE;
	float rightTemperature = (float)Rtemperature * WHEEL_TEMPERATURE_SCALE;

	if( RobotHardWareParam.CarType != T280 ) return;

	if( NodeId == 3 )
	{
		RobotControlParam.MotorA.feedbackCurrent = leftCurrent;
		RobotControlParam.MotorA.feedbackTemperature = leftTemperature;
		RobotControlParam.MotorB.feedbackCurrent = rightCurrent;
		RobotControlParam.MotorB.feedbackTemperature = rightTemperature;
	}
	else if( NodeId == 4 )
	{
		RobotControlParam.MotorC.feedbackCurrent = leftCurrent;
		RobotControlParam.MotorC.feedbackTemperature = leftTemperature;
		RobotControlParam.MotorD.feedbackCurrent = rightCurrent;
		RobotControlParam.MotorD.feedbackTemperature = rightTemperature;
	}
}
