#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"

extern u8 usart_sent[21];
extern u8 usart_sent_current[21];//Send current
extern float Step_SPEED;
#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  4

#define FRAME_HEADER      0X7B //Frame_header // Frame header
#define FRAME_TAIL        0X7D //Frame_tail // End of frame
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

#define FRAME_HEADER_MOVEIT 0XAA //Data frame head that controls the movement of the robotic arm
#define FRAME_TAIL_MOVEIT 0XBB //Data frame tails that control the movement of the robotic arm

#define AutoCharge_HEADER      0X7C //Frame_header // Frame header
#define AutoCharge_TAIL        0X7F //Frame_tail // End of frame
#define AutoCharge_DATA_SIZE    8

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****Structure used to store three-axis gyroscope and accelerometer data*/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes // 2 bytes
	short Y_data; //2 bytes // 2 bytes
	short Z_data; //2 bytes // 2 bytes
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******Structure for sending data via serial port*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1 byte
		short X_speed;	            //2 bytes // 2 bytes
		short Y_speed;              //2 bytes // 2 bytes
		short Z_speed;              //2 bytes // 2 bytes
		short Power_Voltage;        //2 bytes // 2 bytes
		Mpu6050_Data Accelerometer; //6 bytes // 6 bytes
		Mpu6050_Data Gyroscope;     //6 bytes // 6 bytes
		unsigned char Frame_Tail;   //1 bytes //1 byte
	}Sensor_Str;
}SEND_DATA;

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

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1 byte
		float X_speed;	            //4 bytes // 4 bytes
		float Y_speed;              //4 bytes // 4 bytes
		float Z_speed;              //4 bytes // 4 bytes
		unsigned char Frame_Tail;   //1 bytes //1 byte
	}Control_Str;
}RECEIVE_DATA;

extern u8 	fun_flag;

void data_task(void *pvParameters);
void data_transition(void);
void USART1_SEND(void);
void USART3_SEND(void);
void CAN_SEND(void);
void uart1_init(u32 bound);
void uart4_init(u32 bound);
void uart3_init(u32 bound);
int USART1_IRQHandler(void);
int UART4_IRQHandler(void);
int USART3_IRQHandler(void);
void CAN_data_transition(void);
float XYZ_Target_Speed_transition(u8 High,u8 Low);
void usart1_send(u8 data);
void usart4_send(u8 data);
void usart3_send(u8 data);
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode);
u8 AT_Command_Capture(u8 uart_recv);
void _System_Reset_(u8 uart_recv);

#if Mec
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C,float Encoder_D);
#elif Omni
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C);
#elif Diff
void Motion_analysis_transformation(float Encoder_A,float Encoder_B);
#endif


#endif

