/**
 * @file    reportErr_task.c
 * @brief Vehicle self-inspection and commissioning documents
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-08-08
 * @version 1.0.0
 *
 * @details
 * - This file is related to chassis debugging and is managed by the task ReportErrTask. When the user sets the debug level by entering "LOG1~3" on the serial port or Bluetooth end,
 * The debugging task will be triggered, and the vehicle will autonomously report error information, chassis status, and other details.
 * @note
 * 
 * 
 */

#include "system.h"


void Bluetooth_Transmit(uint8_t* buffer,uint8_t size,uint16_t timeout)
{
	for(uint8_t i=0;i<size;i++)
	{
		uart4_send(buffer[i]);
	}
}

//Interface for external data reporting, used for debugging
static void report_interface(char* buffer,uint16_t size)
{
	for(uint32_t k=0;k<size;k+=2)
	{
		Bluetooth_Transmit((uint8_t*)&buffer[k],2,500);
		vTaskDelay( pdMS_TO_TICKS(50) );
	}
//	Bluetooth_Transmit_DMA(&huart2,(uint8_t*)buffer,size);
//	Bluetooth_Transmit_DMA(&huart1,(uint8_t*)buffer,size);
//	vTaskDelay( pdMS_TO_TICKS(10) );
}



//Used to temporarily store output data
static char err_info[512] = { 0 };

void ReportErrTask(void* param)
{

	while(1)
	{
		//Block waiting for task notification
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
		vTaskDelay(500);
		
		while( Debug_Flag!=0 )
		{
			extern uint8_t Received_IP[4];       
			extern uint8_t IP_Frame_Valid;   
			//If the vehicle is actively disabled, the cause will be given priority
			const char* head = "{#\r\n";
			const char* end = "}$";
			
			// IP reporting logic porting
			if(IP_Frame_Valid)  // IP request is closed and IP frames are valid
			{
				// Step 1: send the common header ({#\r\n)
				Bluetooth_Transmit((uint8_t*)head,3,500);
				vTaskDelay(2);
				// Step 2: Send IP content (retain only core information, no head/end)
				sprintf(err_info,"IP:%d.%d.%d.%d\r\n", 
						(uint8_t)Received_IP[0], 
						(uint8_t)Received_IP[1], 
						(uint8_t)Received_IP[2], 
						(uint8_t)Received_IP[3]);
				report_interface(err_info, strlen(err_info));
				vTaskDelay(2);
				// Step 3: Send a unified end(}$)
				Bluetooth_Transmit((uint8_t*)end,2,500);
				vTaskDelay(500);
			}
			else  // IP unknown
			{
				// Align the format as well: head + content + end
				Bluetooth_Transmit((uint8_t*)head,3,500);
				vTaskDelay(2);
				sprintf(err_info,"IP:unknow.\r\n");
				report_interface(err_info, strlen(err_info));
				vTaskDelay(2);
				Bluetooth_Transmit((uint8_t*)end,2,500);
				vTaskDelay(500);
			}
			
			if( robot.LowPower )
			{
				sprintf(err_info,"%s","{#小车电量低，请充电}$");
				report_interface(err_info,strlen(err_info));
				vTaskDelay(500);
				if( Debug_Flag==0 ) break;
			}
			
//			//Enable switch
//			if( RobotControlParam.ErrNum!=0 )
//			{
//				Bluetooth_Transmit(&huart2,(uint8_t*)head,3,500);
//				for(uint8_t i=0;i<sizeof(errorMap)/sizeof(errorMap[0]);i++)
//				{
//					if( Get_RobotErrorCode(errorMap[i].errCode) )
//					{
//						sprintf(err_info,"%s",errorMap[i].errMessage);
//						report_interface(err_info,strlen(err_info));
//					}
//				}
//				vTaskDelay(2);
//				Bluetooth_Transmit(&huart2,(uint8_t*)end,2,500);
//				vTaskDelay(3000);
//				if( Debug_Flag==0 ) break;
//			}
//			else if( RobotControlParam.ErrNum==0 && RobotControlParam.DebugLevel == 1 )
//			{ // Only shows normal status when debug level is 1
//				sprintf(err_info,"%s","{#rnThe cart enable status is normal}$");
//				report_interface(err_info,strlen(err_info));
//				vTaskDelay(3000);
//				if( RobotControlParam.DebugLevel==0 ) break;
//			}

			//Detailed self-inspection parameter report for the chassis
			if( Debug_Flag !=0 )
			{
				Bluetooth_Transmit((uint8_t*)head,3,500);
				
				sprintf(err_info,"小车状态\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Model information
				#if defined AKM_CAR
				sprintf(err_info,"    当前车型：Akm ，%02d\r\n",robot.type);
				#elif  defined DIFF_CAR
				sprintf(err_info,"    当前车型：Diff，%02d\r\n",robot.type);
				#elif  defined MEC_CAR
				sprintf(err_info,"    当前车型：Mec ，%02d\r\n",robot.type);
				#elif  defined _4WD_CAR
				sprintf(err_info,"    当前车型：4WD ，%02d\r\n",robot.type);
				#elif  defined OMNI_CAR
				sprintf(err_info,"    当前车型：OMNI，%02d\r\n",robot.type);
				#endif
				
				report_interface(err_info,strlen(err_info));
				
//				switch( RobotHardWareParam.CarType )
//				{
//					case S300: sprintf(err_info,"S300\r\n"); break;
//					case S300Mini: sprintf(err_info,"S300 Mini \r\n"); break;
//					case S200: sprintf(err_info,"S200\r\n"); break;
//					case S200_OUTDOOR: sprintf(err_info,"S200 OUTDOOR\r\n"); break;
//					case S260: sprintf(err_info,"S260\r\n"); break;
//					case S100: sprintf(err_info,"S100\r\n"); break;
//				}
//				report_interface(err_info,strlen(err_info));
				
				//Hardware version information
				sprintf(err_info,"    C50C版本：");
				report_interface(err_info,strlen(err_info));
				sprintf(err_info,(char*)getHW_Ver(SysVal.HardWare_Ver));
				report_interface(err_info,strlen(err_info));
				sprintf(err_info,"\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Voltage information
				sprintf(err_info,"    当前电压：%.2f V",robot.voltage);
				size_t len = strlen(err_info);
				if (len % 2 != 0) {
					strcat(err_info, " \r\n"); // If the length is odd, add a space
				}
				else strcat(err_info, "\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Emergency stop switch information
				sprintf(err_info,"    急停开关：");
				report_interface(err_info,strlen(err_info));
				if( EN == 0 ) 
					sprintf(err_info,"按下\r\n");
				else
					sprintf(err_info,"弹起\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Enable status information
				sprintf(err_info,"    使能状态：");
				report_interface(err_info,strlen(err_info));
				if( robot_control.FlagStop == 0 )
					sprintf(err_info,"使能\r\n");
				else
					sprintf(err_info,"失能\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Ultrasonic obstacle avoidance information
				sprintf(err_info,"    超声波传感器：");
				report_interface(err_info,strlen(err_info));
				if( SysVal.HardWare_Ranger==0 )
					sprintf(err_info,"无设备\r\n");
				else
					sprintf(err_info,"设备存在\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Automatic recharge mode
				sprintf(err_info,"    自动回充模式：");
				report_interface(err_info,strlen(err_info));
				if( charger.AllowRecharge == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Control status
				sprintf(err_info,"    控制方式：");
				report_interface(err_info,strlen(err_info));
				
				if( Get_Control_Mode(_APP_Control) )
					sprintf(err_info," APP\r\n");
				else if( Get_Control_Mode(_RC_Control) )
					sprintf(err_info," R-C航模\r\n");
				else if( Get_Control_Mode(_PS2_Control) )
					sprintf(err_info,"PS2 \r\n");
				else if( Get_Control_Mode(_ROS_Control) )
					sprintf(err_info,"ROS \r\n");
				else if( Get_Control_Mode(_CAN_Control) )
					sprintf(err_info," CAN\r\n");
				else if( Get_Control_Mode(_USART_Control) )
					sprintf(err_info,"串口1 \r\n");
				report_interface(err_info,strlen(err_info));
				
				vTaskDelay(2);
				Bluetooth_Transmit((uint8_t*)end,2,500);
				vTaskDelay(1500);
				if( Debug_Flag==0 ) break;
				
				Bluetooth_Transmit((uint8_t*)head,3,500);
				sprintf(err_info,"自动回充设备状态\r\n");
				report_interface(err_info,strlen(err_info));
				
				if( SysVal.HardWare_charger==0 )
				{
					sprintf(err_info,"    设备已离线\r\n");
					report_interface(err_info,strlen(err_info));
				}
				else
				{
					//Online status
					sprintf(err_info,"    在线状态：");
					report_interface(err_info,strlen(err_info));
					if( SysVal.HardWare_charger == 1 )
						sprintf(err_info,"在线\r\n");
					else
						sprintf(err_info,"离线\r\n");
					report_interface(err_info,strlen(err_info));
					
					//Charging status
					sprintf(err_info,"    充电状态：");
					report_interface(err_info,strlen(err_info));
					if( charger.Charging==1 )
						sprintf(err_info,"充电中\r\n");
					else
						sprintf(err_info,"未充电\r\n");
					report_interface(err_info,strlen(err_info));
					
					//Current value
					sprintf(err_info,"    测量电流值：%.2fA",(float)charger.ChargingCurrent/1000.0f);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // If the length is odd, add a space
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
					
					//Voltage value
//					sprintf(err_info," Measurement voltage value: %.2fV",ChargeDev.ChargingVol);
//					len = strlen(err_info);
//					if (len % 2 != 0) {
//						strcat(err_info, " rn"); // if the length is odd, append a space
//					}
//					else strcat(err_info, "\r\n");
//					report_interface(err_info,strlen(err_info));
					
					//Infrared signal status
					sprintf(err_info,"    红外信号状态：%d 个信号，%d %d %d %d ",charger.RED_STATE,
						charger.L_A,charger.L_B,charger.R_B,charger.R_A);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // If the length is odd, add a space
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
				}

				vTaskDelay(2);
				Bluetooth_Transmit((uint8_t*)end,2,500);
				vTaskDelay(1500);
				if( Debug_Flag==0 ) break;
			}

		}
	}
}


