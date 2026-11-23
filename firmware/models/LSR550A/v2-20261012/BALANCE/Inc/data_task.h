#ifndef __DATA_TASK_H
#define __DATA_TASK_H

#include "system.h"

#define DATA_TASK_PRIO		4             //Task priority //Task Priority
#define DATA_STK_SIZE 		512           //Task stack size //Task Stack Size
#define DATA_TASK_RATE      RATE_20_HZ   //Task frequency


//Basic 24-byte frame header, frame tail, and data length
#define FRAME_HEADER      0X7B //Frame_header // Frame header
#define FRAME_TAIL        0X7D //Frame_tail // End of frame
#define SEND_DATA_SIZE    24

#define FRAME_HEADER_MOVEIT 0XAA //Data frame head that controls the movement of the robotic arm
#define FRAME_TAIL_MOVEIT 0XBB //Data frame tails that control the movement of the robotic arm


//Automatically recharge frame head, frame, and data length
#define AutoCharge_HEADER      0X7C //Frame_header // Frame header
#define AutoCharge_TAIL        0X7F //Frame_tail // End of frame
#define AutoCharge_DATA_SIZE    8

//IMU three-axis data
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes // 2 bytes
	short Y_data; //2 bytes // 2 bytes
	short Z_data; //2 bytes // 2 bytes
}Mpu6050_Data;

//Robot triaxial speed
typedef struct __Robot_Data_ 
{
	short X_speed; //2 bytes // 2 bytes
	short Y_speed; //2 bytes // 2 bytes
	short Z_speed; //2 bytes // 2 bytes
}Robot_Vel;

//Send data structure definitions
typedef struct _SEND_DATA_  
{
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1 byte
		Robot_Vel    Vel;            //6 bytes
		Mpu6050_Data Accelerometer;  //6 bytes // 6 bytes
		Mpu6050_Data Gyroscope;      //6 bytes // 6 bytes
		short Power_Voltage;        //2 bytes // 2 bytes
		unsigned char Frame_Tail;   //1 bytes //1 byte
	}Sensor_Str;
	
	unsigned char buffer[SEND_DATA_SIZE];//The buffer actually used to send data
}SEND_DATA;

//Send data structure definitions
typedef struct _SEND_AutoCharge_DATA_  
{
	unsigned char buffer[AutoCharge_DATA_SIZE];
	struct _AutoCharge_Str_
	{
		unsigned char Frame_Header; //1 bytes //1 byte
		short Charging_Current;	    //2 bytes // 2 bytes
		unsigned char RED;          //1 bytes //1 byte
		unsigned char Charging;     //1 bytes //1 byte
		unsigned char yuliu;		//1 bytes //1 byte
		unsigned char Frame_Tail;   //1 bytes //1 byte
	}AutoCharge_Str;
}SEND_AutoCharge_DATA;

//Use variables externally
uint8_t Check_BCC(const uint8_t *data, uint16_t length);
void data_task(void *pvParameters);

extern TaskHandle_t data_TaskHandle ;

//Internal use function
static float* Kinematics_akm_diff(float motorA,float motorB);
static float* Kinematics_omni(float motorA,float motorB,float motorC);
static float* Kinematics_mec_4wd(float motorA,float motorB,float motorC,float motorD);
static void data_transition(void);
static void Usart1_SendTask(void);
static void Usart3_SendTask(void);
static void CAN1_SendTask(void);

#endif
