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
//HAL Include File
#include "usart.h"

//C include File
#include <stdio.h>
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_ServoDrive.h"

//APP Include File
#include "RobotControl_task.h"
#include "AutoRecharge_task.h"

//Debug task handles
extern TaskHandle_t g_reportErrTaskHandle;


// Generic error-message mapping table
typedef struct {
    uint32_t errCode;         // Error code
    const char* errMessage;  // Error information
} ErrorMap;

//Error codes of the chassis and corresponding error messages
static const ErrorMap errorMap[] = {
    {errCode_LowPower,        "因小车电压过低，小车已失能\r\n"},
    {errCode_StopKeyEn,       "因急停开关被按下，小车已失能\r\n"},
    {errCode_SoftWareStop,    "因软件急停被设置，小车已失能\r\n"},
    {errCode_Driver1_offline, "因 1号驱动器离线，小车已失能\r\n"},
    {errCode_Driver1_Err,     "因 1号驱动器报错，小车已失能\r\n"},
    {errCode_Driver2_offline, "因 2号驱动器离线，小车已失能\r\n"},
    {errCode_Driver2_Err,     "因 2号驱动器报错，小车已失能\r\n"},
    {errCode_Driver3_offline, "因 3号驱动器离线，小车已失能\r\n"},
    {errCode_Driver3_Err,     "因 3号驱动器报错，小车已失能\r\n"},
    {errCode_Driver4_offline, "因 4号驱动器离线，小车已失能\r\n"},
    {errCode_Driver4_Err,     "因 4号驱动器报错，小车已失能\r\n"},
    {errCode_SteeringHome,    "因转向回零超时，小车已失能\r\n"},
};


//Error codes are defined internally on the servo drive
enum{
	DriverOverVol =          (1<<0), //Driver overvoltage
	DriverUnderVol =         (1<<1), //Driver undervoltage
	ServoMotorOverCur =      (1<<2), //Motor overcurrent
	ServoMotorOverLoad =     (1<<3), //Motor overload
	ServoMotorCurOverD =     (1<<4), //Current deviation exceeds tolerance
	ServoMotorEncoderOverD = (1<<5), //Encoder deviation
	ServoMotorSpeedErr =     (1<<6), //Speed deviation exceeds tolerance
	ServoMotorVolErr =       (1<<7), //Reference voltage error
	DrvierEEPROMErr =        (1<<8), //Driver EEPROM read/write error
	ServoMotorHALLErr =      (1<<9), //Hall sensor error
	ServoMotorOverTemp =     (1<<10), //Motor overheating
	ServoMotorEncoderErr =   (1<<11)  //Encoder error
};

//Driver error codes and corresponding messages
static const ErrorMap ServoErrMap[] = {
    {DriverOverVol,          "驱动器过压\r\n"},
    {DriverUnderVol,         "驱动器欠压\r\n"},
    {ServoMotorOverCur,      "电机过流\r\n"},
    {ServoMotorOverLoad,     "电机过载\r\n"},
    {ServoMotorCurOverD,     "电流超差\r\n"},
    {ServoMotorEncoderOverD, "编码器值超差\r\n"},
    {ServoMotorSpeedErr,     "速度超差\r\n"},
    {ServoMotorVolErr,       "参考电压出错\r\n"},
    {DrvierEEPROMErr,        "驱动器EEPROM读写出错\r\n"},
	{ServoMotorHALLErr,      "霍尔线未插或接触不良\r\n"},
	{ServoMotorOverTemp,     "电机温度超过保护阈值\r\n"},
	{ServoMotorEncoderErr,   "编码器数据错误\r\n"},
};


//Interface for external data reporting, used for debugging
static void report_interface(char* buffer,uint16_t size)
{
	for(uint32_t k=0;k<size;k+=2)
	{
		HAL_UART_Transmit(&huart2,(uint8_t*)&buffer[k],2,500);
		HAL_UART_Transmit(&huart1,(uint8_t*)&buffer[k],2,500);
		vTaskDelay( pdMS_TO_TICKS(50) );
	}
//	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)buffer,size);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)buffer,size);
//	vTaskDelay( pdMS_TO_TICKS(10) );
}

//Used to temporarily store output data
static char err_info[512] = { 0 };

void ReportErrTask(void* param)
{
	
	//Debug level
	// 1: Report only chassis disable reasons (send "LOG1" through the app or serial port 1 to set debug level 1)
	// 2: Display information related to the car and automatic recharge (APP or serial port 1, send "LOG2" to set debug level 2)
	// 3: Displays cart, automatic recharge, and status information of all servo drives (APP or serial port 1, send "LOG3" to set debug level 3)
	// Since Chinese reporting is used, it is necessary to ensure that the string and Chinese transitions always maintain an even number to avoid errors in Chinese character parsing by the app or serial assistant
	
	while(1)
	{
		//Block waiting for task notification
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
		while( RobotControlParam.DebugLevel!=0 )
		{
			//If the vehicle is actively disabled, the cause will be given priority
			const char* head = "{#\r\n";
			const char* end = "}$";
			
			if( RobotControlParam.LowPowerFlag )
			{
				sprintf(err_info,"%s","{#\r\n小车电量低，请充电}$");
				report_interface(err_info,strlen(err_info));
				vTaskDelay(500);
				if( RobotControlParam.DebugLevel==0 ) break;
			}
			
			if( RobotControlParam.ErrNum!=0 )
			{
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				for(uint8_t i=0;i<sizeof(errorMap)/sizeof(errorMap[0]);i++)
				{
					if( Get_RobotErrorCode(errorMap[i].errCode) )
					{
						sprintf(err_info,"%s",errorMap[i].errMessage);
						report_interface(err_info,strlen(err_info));
					}
				}
				vTaskDelay(2);
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(3000);
				if( RobotControlParam.DebugLevel==0 ) break;
			}
			else if( RobotControlParam.ErrNum==0 && RobotControlParam.DebugLevel == 1 )
			{	//Only when debug level 1 is indicated as normal
				sprintf(err_info,"%s","{#\r\n小车使能状态正常}$");
				report_interface(err_info,strlen(err_info));
				vTaskDelay(3000);
				if( RobotControlParam.DebugLevel==0 ) break;
			}

			//Detailed self-inspection parameter report for the chassis
			if( RobotControlParam.DebugLevel > 1 )
			{
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				
				sprintf(err_info,"小车状态\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Model information
				sprintf(err_info,"    当前车型：");
				report_interface(err_info,strlen(err_info));
				switch( RobotHardWareParam.CarType )
				{
					case LSR300: sprintf(err_info,"LSR300\r\n"); break;
					case LSR300Mini: sprintf(err_info,"LSR300 Mini \r\n"); break;
					case LSR200: sprintf(err_info,"LSR200\r\n"); break;
					case LSR200_OUTDOOR: sprintf(err_info,"LSR200 OUTDOOR\r\n"); break;
					case REF2: sprintf(err_info,"REF2\r\n"); break;
					case LSR100: sprintf(err_info,"LSR100\r\n"); break;
				}
				report_interface(err_info,strlen(err_info));
				
				//Hardware version information
				sprintf(err_info,"    C63A版本：");
				report_interface(err_info,strlen(err_info));
				if( get_HardWareVersion() == HW_1_0 )
					sprintf(err_info,"V1.0\r\n");
				else if( get_HardWareVersion() == HW_1_1  )
					sprintf(err_info,"V1.1\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Voltage information
				sprintf(err_info,"    当前电压：%.2f V",RobotControlParam.Vol);
				size_t len = strlen(err_info);
				if (len % 2 != 0) {
					strcat(err_info, " \r\n"); // If the length is odd, add a space
				}
				else strcat(err_info, "\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Emergency stop switch information
				sprintf(err_info,"    急停开关：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.Enkeystate == 0 )
					sprintf(err_info,"按下\r\n");
				else
					sprintf(err_info,"弹起\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Enable status information
				sprintf(err_info,"    使能状态：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.en_flag == 1 )
					sprintf(err_info,"使能");
				else
					sprintf(err_info,"失能，错误码：%d",RobotControlParam.ErrNum);
				len = strlen(err_info);
				if (len % 2 != 0) {
					strcat(err_info, " \r\n"); // If the length is odd, add a space
				}
				else strcat(err_info, "\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Parking mode information
				sprintf(err_info,"    驻车模式：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.ParkingMode == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//IMU linear compensation
				sprintf(err_info,"    IMU 直线补偿：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.ImuAssistedFlag == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Ultrasonic obstacle avoidance information
				sprintf(err_info,"    超声波避障模式：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.RangerAvoidEN == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Automatic recharge mode
				sprintf(err_info,"    自动回充模式：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.ChargeMode == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Emergency mode
				sprintf(err_info,"    紧急模式：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.EmergencyMode == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//Control status
				sprintf(err_info,"    控制方式：");
				report_interface(err_info,strlen(err_info));
				switch( RobotControl_CMDsource )
				{
					case NONE_CMD: sprintf(err_info,"无\r\n");break;
					case BootLoader: sprintf(err_info,"系统控制\r\n");break;
					case GamePad_CMD: sprintf(err_info,"PS2 \r\n");break;
					case ROS_CMD: sprintf(err_info,"ROS \r\n");break;
					case RCJOY_CMD: sprintf(err_info," R-C航模\r\n");break;
					case APP_CMD: sprintf(err_info," APP\r\n");break;
					case CAN_CMD: sprintf(err_info," CAN\r\n");break;
					case Charger_CMD: sprintf(err_info,"回充装备\r\n");break;
					case UnKnownCMD: sprintf(err_info,"未经允许的控制\r\n");break;
				}
				report_interface(err_info,strlen(err_info));
				
				vTaskDelay(2);
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(1500);
				if( RobotControlParam.DebugLevel==0 ) break;
				
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				sprintf(err_info,"自动回充设备状态\r\n");
				report_interface(err_info,strlen(err_info));
				
				if( ChargeDev.online==0 )
				{
					sprintf(err_info,"    设备已离线\r\n");
					report_interface(err_info,strlen(err_info));
				}
				else
				{
					//Online status
					sprintf(err_info,"    在线状态：");
					report_interface(err_info,strlen(err_info));
					if( ChargeDev.online == 1 )
						sprintf(err_info,"在线\r\n");
					else
						sprintf(err_info,"离线\r\n");
					report_interface(err_info,strlen(err_info));
					
					//Charging status
					sprintf(err_info,"    充电状态：");
					report_interface(err_info,strlen(err_info));
					if( ChargeDev.ChargingFlag==1 )
						sprintf(err_info,"充电中\r\n");
					else
						sprintf(err_info,"未充电\r\n");
					report_interface(err_info,strlen(err_info));
					
					//Current value
					sprintf(err_info,"    测量电流值：%.2fA",(float)ChargeDev.ChargingCur/1000.0f);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // If the length is odd, add a space
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
					
					//Voltage value
					sprintf(err_info,"    测量电压值：%.2fV",ChargeDev.ChargingVol);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // If the length is odd, add a space
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
					
					//Infrared signal status
					sprintf(err_info,"    红外信号状态：%d 个信号，%d %d %d %d ",ChargeDev.RedNum,
						ChargeDev.L_A,ChargeDev.L_B,ChargeDev.R_B,ChargeDev.R_A);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // If the length is odd, add a space
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
				}

				vTaskDelay(2);
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(1500);
				if( RobotControlParam.DebugLevel==0 ) break;
			}
			
			//Highest debugging level, indicating all status of the hub motor
			if( RobotControlParam.DebugLevel > 2 )
			{
				for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
				{
					HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
					
					sprintf(err_info,"%2d号驱动器状态：\r\n",g_ServoDriveList[i]->NodeID);
					report_interface(err_info,strlen(err_info));
					
					//Simplified error content when offline
					if( g_ServoDriveList[i]->onlineFlag==0 )
					{
						sprintf(err_info,"    驱动已离线\r\n");
						report_interface(err_info,strlen(err_info));
					}
					else
					{
						sprintf(err_info,"    驱动器地址：%2d\r\n",g_ServoDriveList[i]->NodeID);
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    软件版本号：%6d\r\n",g_ServoDriveList[i]->SoftWareVer);
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    CAN 通信质量：%.2f%%",g_ServoDriveList[i]->comm_qua);
						size_t len = strlen(err_info);
						if (len % 2 != 0) {
							strcat(err_info, " \r\n"); // If the length is odd, add a space
						}
						else strcat(err_info, "\r\n");
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    在线状态：");
						report_interface(err_info,strlen(err_info));
						if( g_ServoDriveList[i]->onlineFlag==1 )
							sprintf(err_info,"在线\r\n");
						else
							sprintf(err_info,"离线\r\n");
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    使能状态：");
						report_interface(err_info,strlen(err_info));
						if( g_ServoDriveList[i]->motorEnflag==1 )
							sprintf(err_info,"使能\r\n");
						else
							sprintf(err_info,"失能\r\n");
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    报错状态：");
						report_interface(err_info,strlen(err_info));
						if( g_ServoDriveList[i]->errFlag==1 )
							sprintf(err_info,"存在报错\r\n");
						else
							sprintf(err_info,"无报错\r\n");
						report_interface(err_info,strlen(err_info));
						
						//Display error content when an error occurs
						if( g_ServoDriveList[i]->errFlag==1 )
						{
							if( g_ServoDriveList[i]->LeftMotorErr!=0 )
							{
								sprintf(err_info,"    左电机存在以下错误：\r\n");
								report_interface(err_info,strlen(err_info));
								for(uint8_t kk=0;kk<sizeof(ServoErrMap)/sizeof(ServoErrMap[0]);kk++)
								{
									if( (g_ServoDriveList[i]->LeftMotorErr)&(ServoErrMap[kk].errCode) )
									{
										sprintf(err_info,"    * %s",ServoErrMap[kk].errMessage);
										report_interface(err_info,strlen(err_info));
									}
								}
							}
							
							if( g_ServoDriveList[i]->RightMotorErr!=0 )
							{
								sprintf(err_info,"    右电机存在以下错误：\r\n");
								report_interface(err_info,strlen(err_info));
								for(uint8_t kk=0;kk<sizeof(ServoErrMap)/sizeof(ServoErrMap[0]);kk++)
								{
									if( (g_ServoDriveList[i]->RightMotorErr)&(ServoErrMap[kk].errCode) )
									{
										sprintf(err_info,"    * %s",ServoErrMap[kk].errMessage);
										report_interface(err_info,strlen(err_info));
									}
								}
							}
						}
					}

					vTaskDelay(2);
					HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
					vTaskDelay(1500);
					if( RobotControlParam.DebugLevel==0 ) break;
				}
			}

		}
	}
}


