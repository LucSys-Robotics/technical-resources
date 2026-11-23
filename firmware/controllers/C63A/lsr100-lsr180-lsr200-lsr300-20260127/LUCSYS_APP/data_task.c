#include "usart.h"

//C Include File
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_icm20948.h"
#include "bsp_can.h"

//APP Include File
#include "robot_select_init.h"
#include "sensor_ranger.h"
#include "AutoRecharge_task.h"

//Check function
extern uint8_t Calculate_BCC(const uint8_t* checkdata,uint16_t datalen);
//Specify the serial port for communication with ROS
static UART_HandleTypeDef *serial = &huart4;

//Define the frame head, frame, tail, and packet length of each packet
#define BaseFRAME_HEAD 0x7B
#define BaseFRAME_TAIL 0x7D
#define BaseFRAME_LEN   24

#define RangerFRAME_HEAD 0xFA
#define RangerFRAME_TAIL 0xFC
#define RangerFRAME_LEN 19

#define ChargeFRAME_HEAD 0x7C
#define ChargeFRAME_TAIL 0x7F
#define ChargeFRAME_LEN  8

void RobotDataTransmitTask(void* param)
{
	//Obtain the timebase, which can run at a fixed frequency for auxiliary tasks
	TickType_t preTime = xTaskGetTickCount();
	
	//The control frequency for this task, in Hz
	const uint16_t TaskFreq = 20;

	//Define the data to be sent
	uint8_t basebuffer[BaseFRAME_LEN];
	uint8_t rangerbuffer[RangerFRAME_LEN];
	uint8_t autorechargerbuffer[ChargeFRAME_LEN];
	//Automatically recharge CAN frame packets
	uint8_t chargerDevCANFrame[8] = { 0 };
	
	pCANInterface_t candev = &UserCAN1Dev;
	while(1)
	{
		//Basic 24-byte data packet for the chassis
		basebuffer[0] = BaseFRAME_HEAD;
		basebuffer[BaseFRAME_LEN-1] = BaseFRAME_TAIL;
		basebuffer[1] = RobotControlParam.en_flag;
		basebuffer[2] = (short)(RobotControlParam.feedbackVx*1000)>>8;
		basebuffer[3] = (short)(RobotControlParam.feedbackVx*1000);
		basebuffer[4] = (short)(RobotControlParam.feedbackVy*1000)>>8;
		basebuffer[5] = (short)(RobotControlParam.feedbackVy*1000);
		basebuffer[6] = (short)(RobotControlParam.feedbackVz*1000)>>8;
		basebuffer[7] = (short)(RobotControlParam.feedbackVz*1000);
		basebuffer[8] = (short)(axis_9ValOri.accel.y)>>8;
		basebuffer[9] = (short)(axis_9ValOri.accel.y);
		basebuffer[10] = (short)(-axis_9ValOri.accel.x)>>8;
		basebuffer[11] = (short)(-axis_9ValOri.accel.x);
		basebuffer[12] = (short)(axis_9ValOri.accel.z)>>8;
		basebuffer[13] = (short)(axis_9ValOri.accel.z);
		basebuffer[14] = (short)(axis_9ValOri.gyro.y)>>8;
		basebuffer[15] = (short)(axis_9ValOri.gyro.y);
		basebuffer[16] = (short)(-axis_9ValOri.gyro.x)>>8;
		basebuffer[17] = (short)(-axis_9ValOri.gyro.x);
		if( RobotControlParam.en_flag==0 )
		{
			basebuffer[18]=0;
			basebuffer[19]=0;
		}
		else
		{
			basebuffer[18] = (short)(axis_9ValOri.gyro.z)>>8;
			basebuffer[19] = (short)(axis_9ValOri.gyro.z);
		}
		basebuffer[20] = (short)(RobotControlParam.Vol*1000)>>8;
		basebuffer[21] = (short)(RobotControlParam.Vol*1000);
		basebuffer[22] = Calculate_BCC(basebuffer,22);
		
		//Ultrasonic data packets
		rangerbuffer[0] = RangerFRAME_HEAD;
		rangerbuffer[RangerFRAME_LEN-1] = RangerFRAME_TAIL;
		rangerbuffer[1] = (short)(RangerHAL_A.dis*1000)>>8;
		rangerbuffer[2] = (short)(RangerHAL_A.dis*1000);
		rangerbuffer[3] = (short)(RangerHAL_B.dis*1000)>>8;
		rangerbuffer[4] = (short)(RangerHAL_B.dis*1000);
		rangerbuffer[5] = (short)(RangerHAL_C.dis*1000)>>8;
		rangerbuffer[6] = (short)(RangerHAL_C.dis*1000);
		rangerbuffer[7] = (short)(RangerHAL_D.dis*1000)>>8;
		rangerbuffer[8] = (short)(RangerHAL_D.dis*1000);
		rangerbuffer[9] = (short)(RangerHAL_E.dis*1000)>>8;
		rangerbuffer[10] = (short)(RangerHAL_E.dis*1000);
		rangerbuffer[11] = (short)(RangerHAL_F.dis*1000)>>8;
		rangerbuffer[12] = (short)(RangerHAL_F.dis*1000);
		rangerbuffer[13] = 0;
		rangerbuffer[14] = 0;
		rangerbuffer[15] = 0;
		rangerbuffer[16] = 0;
		rangerbuffer[17] = Calculate_BCC(rangerbuffer,17);
		
		//Automatic recharge of data packets
		autorechargerbuffer[0] = ChargeFRAME_HEAD;
		autorechargerbuffer[ChargeFRAME_LEN-1] = ChargeFRAME_TAIL;
		autorechargerbuffer[1] = (short)ChargeDev.ChargingCur>>8;
		autorechargerbuffer[2] = (short)ChargeDev.ChargingCur;
		autorechargerbuffer[3] = ChargeDev.RedNum;
		autorechargerbuffer[4] = ChargeDev.ChargingFlag;
		autorechargerbuffer[5] = RobotControlParam.ChargeMode;
		autorechargerbuffer[6] = Calculate_BCC(autorechargerbuffer,6);
		
		//Send corresponding data depending on the vehicle model
		if( RobotHardWareParam.CarType == LSR300 || RobotHardWareParam.CarType == LSR300Mini ||
 			RobotHardWareParam.CarType == LSR200_OUTDOOR || RobotHardWareParam.CarType == REF2 )
		{
			uint8_t tmp[BaseFRAME_LEN+RangerFRAME_LEN+ChargeFRAME_LEN]={0};
			uint8_t *ptr = tmp;
			
			//Basic 24-byte packet
			memcpy(ptr,basebuffer,BaseFRAME_LEN);
			ptr+=BaseFRAME_LEN;
			
			//Ultrasonic data packets
			memcpy(ptr,rangerbuffer,RangerFRAME_LEN);
			ptr+=RangerFRAME_LEN;
			
			//Automatic recharge of data packets
			memcpy(ptr,autorechargerbuffer,ChargeFRAME_LEN);
			
			//Start DMA sending
			HAL_UART_Transmit_DMA(serial,tmp,sizeof(tmp)/sizeof(tmp[0]));
			
			//Serial port 1 sends
			if( RobotControlParam.DebugLevel == 0 )
				HAL_UART_Transmit_DMA(&huart1,tmp,sizeof(tmp)/sizeof(tmp[0]));
			
			
			//Simulate ultrasound-free conditions
//			uint8_t tmp[BaseFRAME_LEN+ChargeFRAME_LEN]={0};
//			uint8_t *ptr = tmp;
//			
//			//Basic 24-byte packet
//			memcpy(ptr,basebuffer,BaseFRAME_LEN);
//			ptr+=BaseFRAME_LEN;
//			
//			//Automatic recharge of data packets
//			memcpy(ptr,autorechargerbuffer,ChargeFRAME_LEN);
//			
//			//Start DMA sending
//			HAL_UART_Transmit_DMA(serial,tmp,sizeof(tmp)/sizeof(tmp[0]));
//			
//			//Serial port 1 sends
//			if( RobotControlParam.DebugLevel == 0 )
//				HAL_UART_Transmit_DMA(&huart1,tmp,sizeof(tmp)/sizeof(tmp[0]));

			//Emulates only 24-byte packets
//			uint8_t tmp[BaseFRAME_LEN]={0};
//			uint8_t *ptr = tmp;
//			
//			//Basic 24-byte packet
//			memcpy(ptr,basebuffer,BaseFRAME_LEN);
//			
//			//Start DMA sending
//			HAL_UART_Transmit_DMA(serial,tmp,sizeof(tmp)/sizeof(tmp[0]));
//			
//			//Serial port 1 sends
//			if( RobotControlParam.DebugLevel == 0 )
//				HAL_UART_Transmit_DMA(&huart1,tmp,sizeof(tmp)/sizeof(tmp[0]));
		}
		else if( RobotHardWareParam.CarType == LSR200 || RobotHardWareParam.CarType == LSR100 )
		{
			uint8_t tmp[BaseFRAME_LEN+ChargeFRAME_LEN]={0};
			uint8_t *ptr = tmp;
			
			//Basic 24-byte packet
			memcpy(ptr,basebuffer,BaseFRAME_LEN);
			ptr+=BaseFRAME_LEN;
			
			//Automatic recharge of data packets
			memcpy(ptr,autorechargerbuffer,ChargeFRAME_LEN);

			//ROS serial send
			HAL_UART_Transmit_DMA(serial,tmp,sizeof(tmp)/sizeof(tmp[0]));
			
			//Serial port 1 sends
			if( RobotControlParam.DebugLevel == 0 )
				HAL_UART_Transmit_DMA(&huart1,tmp,sizeof(tmp)/sizeof(tmp[0]));
		}
		
		//CAN data sending. Sending basic 24-byte data via frame id 0x101-0x103
		for(uint8_t i=0;i<3;i++)
		{
			candev->sendStd(0x100+(i+1),basebuffer+i*8,8);
		}
		
		//Automatic recharge packet sending. Feedback flags via frame ID 0x105 and setting docking speed
		chargerDevCANFrame[0] = RobotControlParam.ChargeMode;
		chargerDevCANFrame[1] = (short)(ChargeDev.dockerX*1000)>>8;
		chargerDevCANFrame[2] = (short)(ChargeDev.dockerX*1000);
		chargerDevCANFrame[3] = (short)(ChargeDev.dockerY*1000)>>8;
		chargerDevCANFrame[4] = (short)(ChargeDev.dockerY*1000);
		chargerDevCANFrame[5] = (short)(ChargeDev.dockerZ*1000)>>8;
		chargerDevCANFrame[6] = (short)(ChargeDev.dockerZ*1000);
		chargerDevCANFrame[7] = 0;
		candev->sendStd(0x105,chargerDevCANFrame,8);
		
		//Automatic recharge control feature sending (TODO: remove and optimize this section)
		//Purpose: Used to recognize the movement characteristics of the chassis in charging equipment and to slow down during docking and charging
		uint8_t chargerControlSend[8]={0};
		switch(RobotHardWareParam.CarType)
		{
			case LSR300:case LSR300Mini:case LSR100:
				if( RobotControlParam.MotorA.feedback>0 ) chargerControlSend[0]=1;
				else if( RobotControlParam.MotorA.feedback<0 ) chargerControlSend[0]=2;
				else chargerControlSend[0]=0;
			
				if( RobotControlParam.MotorB.feedback>0 ) chargerControlSend[1]=1;
				else if( RobotControlParam.MotorB.feedback<0 ) chargerControlSend[1]=2;
				else chargerControlSend[1]=0;
				break;
			case LSR200:
				if( RobotControlParam.MotorA.feedback>0 ) chargerControlSend[0]=1;
				else if( RobotControlParam.MotorA.feedback<0 ) chargerControlSend[0]=2;
				else chargerControlSend[0]=0;
			
				if( RobotControlParam.MotorD.feedback>0 ) chargerControlSend[1]=1;
				else if( RobotControlParam.MotorD.feedback<0 ) chargerControlSend[1]=2;
				else chargerControlSend[1]=0;
				break;
			//Models with rear ultrasonic sensors do not require charging equipment to detect wheel movement. Automatic deceleration
			case REF2:case LSR200_OUTDOOR:
				chargerControlSend[0]=0;
				chargerControlSend[1]=0;
				break;
		}
		candev->sendStd(0x185,chargerControlSend,8);
		
		
		/* Delay the specified frequency */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
	
}

