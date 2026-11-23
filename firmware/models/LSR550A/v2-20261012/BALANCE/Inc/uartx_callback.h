#ifndef __UARTX_CALLBACK_H
#define __UARTX_CALLBACK_H 

#include "system.h"

//The length of data the robot receives control commands
#define RECEIVE_DATA_SIZE 11

//The structure where the robot receives control commands
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

//Internal function
static float XYZ_Target_Speed_transition(u8 High,u8 Low);
static u8 AT_Command_Capture(u8 uart_recv);
static void _System_Reset_(u8 uart_recv);

#endif

